// ============================================================================
// three_sipp_es_pmp.sv — RISC-V Physical Memory Protection (PMP) checker.
//
// Combinational access check for one memory reference (fetch / load / store).
// Implements the standard PMP matching modes per region — OFF / TOR / NA4 /
// NAPOT — with R/W/X permission bits and the L (lock) bit. This is the ISA
// feature behind M/U memory isolation in Ibex/OpenTitan, Hazard3, etc.
//
//   * M-mode bypasses a region's permissions UNLESS that region is locked (L=1).
//   * The lowest-numbered matching region wins (RISC-V priority rule).
//   * No region matches: M-mode is allowed; U-mode is denied (when >=1 region
//     is implemented). With NPMP=0 the whole check is a pass-through.
//   * Smepmp mseccfg.MML=1 (machine mode lockdown): permissions follow the
//     Smepmp 1.0 truth table. L=1 rules are M-mode-only, L=0 rules are
//     U-mode-only, and four {L,R,W,X} encodings define shared regions. M-mode
//     may not execute from memory that matches no rule.
//
// Config/address CSRs are passed in as flat vectors (one 8-bit cfg + one 32-bit
// addr per region) so the holder (the CSR file) owns the architectural state.
// ============================================================================


module three_sipp_es_pmp #(
  parameter int NPMP = 8
)(
  input  wire [8*NPMP-1:0]  cfg,       // {pmpNcfg, ..., pmp0cfg}
  input  wire [32*NPMP-1:0] addrreg,   // {pmpaddrN, ..., pmpaddr0}
  input  wire [31:0]        addr,      // byte address of the access
  input  wire               priv_m,    // 1 = M-mode, 0 = U-mode
  input  wire               mmwp,      // mseccfg.MMWP: M-mode whitelist (no-match=deny)
  input  wire               mml,       // mseccfg.MML: Smepmp machine mode lockdown
  input  wire               do_r,      // load
  input  wire               do_w,      // store
  input  wire               do_x,      // instruction fetch
  output reg                fault      // 1 = access denied -> access-fault trap
);
  integer i;
  reg [NPMP-1:0] match, allow;
  reg            matched;
  reg            sel_allow;

  // Smepmp truth table with MML=1: {L,R,W,X} -> {M r,w,x, U r,w,x}
  function automatic logic [5:0] mml_perm(input logic [3:0] lrwx);
    unique case (lrwx)
      4'b0000: mml_perm = 6'b000_000;   // inaccessible
      4'b0001: mml_perm = 6'b000_001;   // U execute-only
      4'b0010: mml_perm = 6'b110_100;   // shared data: M RW, U R
      4'b0011: mml_perm = 6'b110_110;   // shared data: M RW, U RW
      4'b0100: mml_perm = 6'b000_100;   // U read-only
      4'b0101: mml_perm = 6'b000_101;   // U read/execute
      4'b0110: mml_perm = 6'b000_110;   // U read/write
      4'b0111: mml_perm = 6'b000_111;   // U read/write/execute
      4'b1000: mml_perm = 6'b000_000;   // locked inaccessible
      4'b1001: mml_perm = 6'b001_000;   // M execute-only
      4'b1010: mml_perm = 6'b001_001;   // locked shared code: M X, U X
      4'b1011: mml_perm = 6'b101_001;   // locked shared code: M RX, U X
      4'b1100: mml_perm = 6'b100_000;   // M read-only
      4'b1101: mml_perm = 6'b101_000;   // M read/execute
      4'b1110: mml_perm = 6'b110_000;   // M read/write
      4'b1111: mml_perm = 6'b100_100;   // locked shared data: M R, U R
    endcase
  endfunction

  always_comb begin
    logic [7:0]  c;  logic [1:0] A;  logic L, X, W, R, perm;
    logic [5:0]  mp;  logic [2:0] rwx;
    logic [31:0] pa, pa_prev, m, aw;
    aw = {2'b00, addr[31:2]};                 // address in 4-byte units (bits 33:2)
    match = '0; allow = '0;
    for (i = 0; i < NPMP; i = i + 1) begin
      c  = cfg[i*8 +: 8];
      A  = c[4:3]; L = c[7]; X = c[2]; W = c[1]; R = c[0];
      pa      = addrreg[i*32 +: 32];
      pa_prev = (i == 0) ? 32'd0 : addrreg[(i-1)*32 +: 32];
      m       = pa ^ (pa + 32'd1);            // NAPOT size mask (low run of 1s)
      unique case (A)
        2'd0: match[i] = 1'b0;                              // OFF
        2'd1: match[i] = (aw >= pa_prev) && (aw < pa);      // TOR
        2'd2: match[i] = (aw == pa);                        // NA4
        2'd3: match[i] = ((aw & ~m) == (pa & ~m));          // NAPOT
      endcase
      // every requested access type must be permitted (an AMO reads and writes)
      perm     = (!do_x | X) & (!do_r | R) & (!do_w | W);
      mp       = mml_perm({L, R, W, X});
      rwx      = priv_m ? mp[5:3] : mp[2:0];
      if (mml)
        allow[i] = (!do_r | rwx[2]) & (!do_w | rwx[1]) & (!do_x | rwx[0]);
      else
        allow[i] = (priv_m && !L) ? 1'b1 : perm;            // M bypasses unless locked
    end
    // lowest matching region wins (reverse scan so index 0 is last write)
    matched = 1'b0; sel_allow = 1'b0;
    for (i = NPMP-1; i >= 0; i = i - 1)
      if (match[i]) begin matched = 1'b1; sel_allow = allow[i]; end
    // no match: U always denied; M denied under MMWP (whitelist policy), and
    // under MML M-mode may not execute from unmatched memory
    fault = matched ? ~sel_allow : (~priv_m | mmwp | (mml & do_x));
  end
endmodule
`default_nettype wire
