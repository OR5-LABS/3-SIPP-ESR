// ============================================================================
// three_sipp_es_core.sv — RV32IM, 3-stage in-order pipeline (F | X | W).
//
// 3-SIPP-ESR is a compact, high-performance embedded-class core: a short 3-stage
// pipeline with full forwarding and a dynamic branch predictor, in the space of
// an ARM Cortex-M-style microcontroller.
//
//   F : fetch (PC -> instr)
//   X : decode + regfile read + ALU/branch/CSR/muldiv + memory-address
//   W : memory access + writeback
//
// Because load data is produced in W on the same cycle a dependent instruction
// is in X, a single W->X forwarding network covers ALL hazards — there is no
// load-use stall. Only the multi-cycle M-extension stalls the front-end.
// Branches/jumps/traps resolve in X with a 1-cycle flush.
//
// Reuses the verified family leaf cells (ALU/muldiv/CSR/regfile).
// ============================================================================
`include "three_sipp_es_pkg.sv"

module three_sipp_es_core
  import three_sipp_es_pkg::*;
#(
  parameter logic [XLEN-1:0] RESET_PC = 32'h0000_0000,
  // SECURE=1 adds User privilege mode + an 8-region PMP (M/U memory isolation,
  // Ibex-secure / OpenTitan class). Default 0 keeps the M-mode-only
  // footprint byte-identical, so the shared RV32IMAC compliance build is unchanged
  // and 3-SIPP-ESR stays 61/61.
  parameter bit              SECURE    = 1'b0,
  // Dynamic branch prediction (gshare BHT + BTB), ON by default. Halfword-granular
  // (pc[1] in the index) so RVC 4-byte branches at 2-byte-aligned PCs can't alias a
  // neighbouring compressed instr into a false BTB hit. BPRED=0 = static predict-
  // not-taken. Same design/verification path as three_sipp_es_core (see its comment).
  parameter bit              BPRED     = 1,
  parameter int unsigned     BHT_BITS  = 8,   // 2^8 = 256 2-bit gshare counters
  parameter int unsigned     BTB_BITS  = 6,   // 2^6 = 64 direct-mapped BTB entries
  parameter int unsigned     GHR_BITS  = 8    // global history length
)(
  input  logic              clk,
  input  logic              rst,
  output logic [XLEN-1:0]   imem_addr,
  input  logic [XLEN-1:0]   imem_rdata,
  output logic [XLEN-1:0]   dmem_addr,
  output logic              dmem_re,
  output logic              dmem_we,
  output logic [3:0]        dmem_be,
  output logic [XLEN-1:0]   dmem_wdata,
  input  logic [XLEN-1:0]   dmem_rdata,
  input  logic              irq_timer,    // CLINT/PLIC pending lines (tie 0 if unused)
  input  logic              irq_soft,
  input  logic              irq_ext,
  output logic              retire_valid,
  output logic [XLEN-1:0]   retire_pc,
  output logic [XLEN-1:0]   retire_instr,
  output logic              retire_rd_we,
  output logic [4:0]        retire_rd,
  output logic [XLEN-1:0]   retire_rd_val,
  // ---- RISC-V external debug (tie haltreq / ar_valid = 0 if no DM) ---------
  input  logic              dbg_haltreq,
  input  logic              dbg_resumereq,
  output logic              dbg_halted,
  input  logic              dbg_ar_valid,
  input  logic              dbg_ar_write,
  input  logic              dbg_ar_csr,
  input  logic [11:0]       dbg_ar_regno,
  input  logic [XLEN-1:0]   dbg_ar_wdata,
  output logic [XLEN-1:0]   dbg_ar_rdata,
  output logic              dbg_ar_done
`ifdef RISCV_FORMAL
  ,
  // RISC-V Formal Interface (riscv-formal). Sampled when rvfi_valid=1.
  output logic              rvfi_valid,
  output logic [63:0]       rvfi_order,
  output logic [31:0]       rvfi_insn,
  output logic              rvfi_trap,
  output logic              rvfi_halt,
  output logic              rvfi_intr,
  output logic [1:0]        rvfi_mode,
  output logic [1:0]        rvfi_ixl,
  output logic [4:0]        rvfi_rs1_addr,
  output logic [4:0]        rvfi_rs2_addr,
  output logic [XLEN-1:0]   rvfi_rs1_rdata,
  output logic [XLEN-1:0]   rvfi_rs2_rdata,
  output logic [4:0]        rvfi_rd_addr,
  output logic [XLEN-1:0]   rvfi_rd_wdata,
  output logic [XLEN-1:0]   rvfi_pc_rdata,
  output logic [XLEN-1:0]   rvfi_pc_wdata,
  output logic [XLEN-1:0]   rvfi_mem_addr,
  output logic [3:0]        rvfi_mem_rmask,
  output logic [3:0]        rvfi_mem_wmask,
  output logic [XLEN-1:0]   rvfi_mem_rdata,
  output logic [XLEN-1:0]   rvfi_mem_wdata
`endif
);
  // ---- pipeline registers --------------------------------------------------
  // F/X
  logic [XLEN-1:0] fx_pc, fx_instr;
  logic            fx_valid;
  logic [2:0]      fx_len;          // bytes of the FX instruction (2 or 4)
  logic            straddle;        // mid 2-beat fetch of a word-straddling 32b instr
  logic [15:0]     strad_lo;        // low half captured in beat 1
  // branch-prediction bits carried F -> X (describe the fetched instruction)
  logic                fx_pred_taken;   // predicted taken at F
  logic [XLEN-1:0]     fx_pred_target;  // predicted next-PC at F
  logic [GHR_BITS-1:0] fx_ghist;        // GHR snapshot at fetch
  // X/W
  logic [XLEN-1:0] xw_pc, xw_instr, xw_result, xw_store_data;
  logic [4:0]      xw_rd;
  logic            xw_valid, xw_reg_we, xw_mem_re, xw_mem_we;
  logic [1:0]      xw_mem_width, xw_addr_lo;
  logic            xw_mem_unsigned;
  logic [3:0]      xw_be;
  logic            xw_is_amo, xw_is_lr, xw_is_sc, xw_is_amo_rmw;
  logic [4:0]      xw_amo_f5;
  logic [XLEN-1:0] xw_amo_addr, xw_amo_b;   // AMO address (rs1) + operand (rs2)

  logic            redirect, md_stall;
  logic [XLEN-1:0] redirect_target;
  logic            mem_beat_stall;   // W is doing the 1st of two misaligned beats
  logic            beat2;            // registered: W is driving the 2nd beat
  logic [31:0]     ld_w0;            // captured first word of a misaligned load
  wire             fe_stall  = md_stall | mem_beat_stall;
  // X-stage side effects are suppressed while X is frozen (muldiv or misaligned 2nd beat)
  wire             ex_freeze = md_stall | mem_beat_stall;

  // ---- RISC-V external debug (drain-to-halt on this 3-stage pipeline) -------
  logic            dbg_mode;        // halted in debug mode
  logic            halt_req;        // halt requested; fetch frozen, pipe draining
  logic            step_active;     // resumed with step; halt after one fetch
  logic [XLEN-1:0] dpc, dscratch0;
  logic            dcsr_ebreakm, dcsr_step;
  logic [2:0]      dcsr_cause;
  logic            dbg_ar_busy;
  logic            dpc_cap_valid;   // dpc captured from an ebreak (its own PC)
  logic [XLEN-1:0] dpc_cap;
  assign dbg_halted = dbg_mode;
  // EBREAK enters debug (not a trap) when dcsr.ebreakm is set
  wire ebreak_to_debug = fx_valid && (fx_instr==32'h00100073) && dcsr_ebreakm && !dbg_mode;
  // freeze the front-end (no new fetch) while halting/halted/step-1-done/ebreak
  wire step_halt_now = step_active && fx_valid && !dbg_mode;
  wire dbg_freeze = dbg_mode | halt_req | (dbg_haltreq & ~dbg_mode) |
                    ebreak_to_debug | step_halt_now;
  wire dbg_resume_now = dbg_mode & dbg_resumereq;
  wire pipe_drained = !fx_valid && !xw_valid;   // nothing left in X or W
  // debug abstract-access (borrow idle regfile/CSR ports while halted)
  wire        dbg_do     = dbg_mode && dbg_ar_valid && !dbg_ar_busy;
  wire        dbg_gpr_we = dbg_do && dbg_ar_write && !dbg_ar_csr;
  wire        dbg_csr_local = (dbg_ar_regno==12'h7b0)||(dbg_ar_regno==12'h7b1)||(dbg_ar_regno==12'h7b2);
  wire        dbg_csr_we = dbg_do && dbg_ar_write && dbg_ar_csr && !dbg_csr_local;
  wire [XLEN-1:0] dcsr_val = {4'd4, 12'd0, dcsr_ebreakm, 6'd0,
                              dcsr_cause, 3'd0, dcsr_step, 2'b11};

  // ==========================================================================
  // F stage
  // ==========================================================================
  logic [XLEN-1:0] pc;
  // straddle beat re-reads the next aligned word; otherwise read the word at PC
  assign imem_addr = straddle ? ({pc[XLEN-1:2], 2'b00} + 32'd4) : pc;

  // ---- Dynamic branch predictor: gshare BHT (2-bit counters) + BTB ----------
  // Lookup is combinational at F; update is at X where the branch/jal resolves.
  // JALR is unpredicted (register target). Halfword-granular indexing (pc[1] in
  // the index) — see the three_sipp_es_core note: without it, an RVC compressed instr
  // aliases a straddling 4-byte branch's BTB entry, gets a false hit, is predicted
  // taken, and (being a non-branch) is never corrected in X -> pipeline diverges.
  localparam int unsigned BHT_ENTRIES = (1 << BHT_BITS);
  localparam int unsigned BTB_ENTRIES = (1 << BTB_BITS);
  localparam int unsigned BTB_TAGW    = XLEN - BTB_BITS - 1;
  logic [1:0]           bht [0:BHT_ENTRIES-1];
  logic                 btb_valid [0:BTB_ENTRIES-1];
  logic [BTB_TAGW-1:0]  btb_tag   [0:BTB_ENTRIES-1];
  logic [XLEN-1:0]      btb_tgt   [0:BTB_ENTRIES-1];
  logic                 btb_isjal [0:BTB_ENTRIES-1];
  logic [GHR_BITS-1:0]  ghist;

  wire [BHT_BITS-1:0] bht_rd_idx = pc[BHT_BITS:1] ^ ghist[BHT_BITS-1:0];
  wire [BTB_BITS-1:0] btb_rd_idx = pc[BTB_BITS:1];
  wire [BTB_TAGW-1:0] btb_rd_tag = pc[XLEN-1:BTB_BITS+1];
  wire                btb_hit    = BPRED && btb_valid[btb_rd_idx] &&
                                   (btb_tag[btb_rd_idx] == btb_rd_tag);
  wire                bht_taken  = bht[bht_rd_idx][1];
  wire                predict_taken  = btb_hit && (btb_isjal[btb_rd_idx] || bht_taken);
  wire [XLEN-1:0]     predict_target = btb_tgt[btb_rd_idx];

  // predictor-update signals (driven from X, consumed by the update always_ff)
  logic                bp_upd_en, bp_upd_taken, bp_upd_isjal;
  logic [XLEN-1:0]     bp_upd_pc, bp_upd_target;
  logic [GHR_BITS-1:0] bp_upd_ghist;

  // RVC: decompress the halfword at PC
  wire [15:0] fetch_half = pc[1] ? imem_rdata[31:16] : imem_rdata[15:0];
  wire [31:0] c_instr32;
  wire        c_is_comp, c_illegal;
  three_sipp_es_rvc u_rvc (.instr16(fetch_half), .instr32(c_instr32),
                     .is_compressed(c_is_comp), .decomp_illegal(c_illegal));

  // a 32-bit instruction at an odd halfword needs a second fetch beat
  wire need_straddle  = !c_is_comp && pc[1] && !straddle;
  wire [31:0] fetched_instr = straddle  ? {imem_rdata[15:0], strad_lo}
                            : c_is_comp ? (c_illegal ? 32'h0 : c_instr32)
                            : imem_rdata;
  wire [2:0]  fetched_len   = (straddle || !c_is_comp) ? 3'd4 : 3'd2;

  // ---- Return Address Stack (RAS) — predicts function returns (same design as
  //      three_sipp_es_core; correctness-safe since X recomputes the real jalr_t) -----
  localparam int unsigned RAS_N = 8, RAS_PW = 3;
  logic [XLEN-1:0]  ras [0:RAS_N-1];
  logic [RAS_PW-1:0] ras_ptr; logic [RAS_PW:0] ras_cnt;
  wire  [RAS_PW-1:0] ras_top_idx = ras_ptr - 1'b1;
  wire               ras_valid   = (ras_cnt != 0);
  wire [6:0] f_op  = fetched_instr[6:0];
  wire [4:0] f_rd  = fetched_instr[11:7];
  wire [4:0] f_rs1 = fetched_instr[19:15];
  wire f_rd_link   = (f_rd  == 5'd1) || (f_rd  == 5'd5);
  wire f_rs1_link  = (f_rs1 == 5'd1) || (f_rs1 == 5'd5);
  wire f_is_jalr   = (f_op == 7'b1100111);
  wire f_is_jal    = (f_op == 7'b1101111);
  wire f_is_ret    = BPRED && f_is_jalr && f_rs1_link && !f_rd_link;
  wire f_is_call   = BPRED && (f_is_jal || f_is_jalr) && f_rd_link;
  wire            ras_predict      = f_is_ret && ras_valid;
  wire            predict_taken_f  = ras_predict ? 1'b1 : predict_taken;
  wire [XLEN-1:0] predict_target_f = ras_predict ? ras[ras_top_idx] : predict_target;

  always_ff @(posedge clk) begin
    if (rst) begin
      pc <= RESET_PC; fx_valid <= 1'b0; straddle <= 1'b0; strad_lo <= 16'h0; fx_len <= 3'd4;
      fx_pred_taken <= 1'b0; fx_pred_target <= '0; fx_ghist <= '0;
    end else if (redirect) begin
      pc <= redirect_target; fx_valid <= 1'b0; straddle <= 1'b0;
    end else if (dbg_resume_now) begin
      pc <= dpc; fx_valid <= 1'b0; straddle <= 1'b0;         // resume: refetch at dpc
    end else if (dbg_freeze) begin
      fx_valid <= 1'b0;                                       // halting/halted: stop fetch (hold pc)
    end else if (fe_stall) begin
      // hold PC, FX and the straddle state while EX waits on muldiv / W's 2nd beat
    end else if (need_straddle) begin
      strad_lo <= imem_rdata[31:16];   // capture low half, fetch next word next cycle
      straddle <= 1'b1;
      fx_valid <= 1'b0;                // bubble this cycle (no instruction produced)
    end else begin
      fx_valid <= 1'b1;
      fx_pc    <= pc;
      fx_instr <= fetched_instr;
      fx_len   <= fetched_len;
      fx_pred_taken  <= predict_taken_f;
      fx_pred_target <= predict_target_f;
      fx_ghist       <= ghist;
      straddle <= 1'b0;
      // predicted-taken control flow steers the next fetch to the predicted target
      pc       <= predict_taken_f ? predict_target_f : (pc + {29'd0, fetched_len});
    end
  end

  // ---- Decoupled Return Address Stack (RAS) storage update -----------------
  // Decoupling RAS into its own clocked process isolates its write-enable logic
  // from the main F-stage PC steering and pipeline stall networks.
  always_ff @(posedge clk) begin
    if (rst) begin
      ras_ptr <= '0;
      ras_cnt <= '0;
    end else if (!fe_stall && !redirect && !dbg_freeze && !need_straddle) begin
      if (f_is_call) begin
        ras[ras_ptr] <= pc + {29'd0, fetched_len};
        ras_ptr      <= ras_ptr + 1'b1;
        if (ras_cnt != RAS_N) ras_cnt <= ras_cnt + 1'b1;
      end else if (f_is_ret && ras_valid) begin
        ras_ptr      <= ras_ptr - 1'b1;
        ras_cnt      <= ras_cnt - 1'b1;
      end
    end
  end

  // ==========================================================================
  // X stage — decode + execute
  // ==========================================================================
  wire [2:0]  funct3 = fx_instr[14:12];
  wire [4:0]  rs1    = fx_instr[19:15];
  wire [4:0]  rs2    = fx_instr[24:20];
  wire [4:0]  rd     = fx_instr[11:7];
  wire [11:0] sys_imm12 = fx_instr[31:20];

  // immediate generator (shared leaf cell)
  logic [XLEN-1:0] imm_i, imm_s, imm_b, imm_u, imm_j, id_imm;
  three_sipp_es_immgen u_imm (.instr(fx_instr),
    .imm_i(imm_i), .imm_s(imm_s), .imm_b(imm_b), .imm_u(imm_u), .imm_j(imm_j));

  logic [XLEN-1:0] rdata1, rdata2, wb_value;   // wb_value driven by W stage
  logic            rf_we;
  logic [4:0]      rf_wa;
  // debug GPR access steals the idle read/write ports while halted
  wire [4:0]       rf_ra1_eff = (dbg_mode && dbg_ar_valid && !dbg_ar_csr)
                                ? dbg_ar_regno[4:0] : rs1;
  wire             rf_we_eff  = dbg_gpr_we | rf_we;
  wire [4:0]       rf_wa_eff  = dbg_gpr_we ? dbg_ar_regno[4:0] : rf_wa;
  wire [XLEN-1:0]  rf_wd_eff  = dbg_gpr_we ? dbg_ar_wdata      : wb_value;
  three_sipp_es_regfile u_rf (
    .clk(clk), .ra1(rf_ra1_eff), .ra2(rs2), .rd1(rdata1), .rd2(rdata2),
    .we(rf_we_eff), .wa(rf_wa_eff), .wd(rf_wd_eff)
  );

  // decode (shared base RV32IMC decoder leaf cell)
  alu_op_e d_alu_op; br_op_e d_br_op; md_op_e d_md_op; wb_sel_e d_wb_sel;
  logic d_use_pc, d_use_imm, d_reg_we, d_mem_re, d_mem_we, d_mem_unsigned;
  logic [1:0] d_mem_width;
  logic d_is_branch, d_is_jal, d_is_jalr, d_is_md, d_is_csr;
  logic d_is_ecall, d_is_ebreak, d_is_mret, d_illegal, d_uses_rs2;

  three_sipp_es_decode u_dec (
    .instr(fx_instr), .imm_i(imm_i), .imm_s(imm_s), .imm_b(imm_b),
    .imm_u(imm_u), .imm_j(imm_j),
    .alu_op(d_alu_op), .br_op(d_br_op), .md_op(d_md_op), .wb_sel(d_wb_sel),
    .use_pc(d_use_pc), .use_imm(d_use_imm), .reg_we(d_reg_we),
    .mem_re(d_mem_re), .mem_we(d_mem_we), .mem_unsigned(d_mem_unsigned),
    .mem_width(d_mem_width), .is_branch(d_is_branch), .is_jal(d_is_jal),
    .is_jalr(d_is_jalr), .is_md(d_is_md), .is_csr(d_is_csr),
    .is_ecall(d_is_ecall), .is_ebreak(d_is_ebreak), .is_mret(d_is_mret),
    .illegal(d_illegal), .uses_rs2(d_uses_rs2), .id_imm(id_imm)
  );

  // ---- RV32 'A' (atomics) inline decode (three_sipp_es_decode untouched) ----------
  // AMO opcode 0101111, funct3=010 (word). Address = rs1 (op1); rd = old memory.
  // No load-use stall needed: 3-SIPP-ESR forwards only W->X, and an AMO stays in W
  // (frozen for its store beat) while a dependent waits in X — so W->X forwarding
  // delivers the AMO result naturally.
  wire        is_amo     = (fx_instr[6:0]==7'b0101111) && (funct3==3'b010);
  wire [4:0]  amo_f5     = fx_instr[31:27];
  wire        is_lr      = is_amo && (amo_f5==5'b00010);   // LR.W
  wire        is_sc      = is_amo && (amo_f5==5'b00011);   // SC.W
  wire        is_amo_rmw = is_amo && !is_lr && !is_sc;      // AMO<op>.W
  wire        amo_valid  = is_amo &&
      ((amo_f5==5'b00010 && rs2==5'd0) ||                   // LR.W (rs2 must be 0)
       (amo_f5==5'b00011) || (amo_f5==5'b00001) ||          // SC / SWAP
       (amo_f5==5'b00000) || (amo_f5==5'b00100) ||          // ADD / XOR
       (amo_f5==5'b01100) || (amo_f5==5'b01000) ||          // AND / OR
       (amo_f5==5'b10000) || (amo_f5==5'b10100) ||          // MIN / MAX
       (amo_f5==5'b11000) || (amo_f5==5'b11100));           // MINU / MAXU

  // FENCE.I instruction (Zifencei extension) triggers pipeline flush and refetch
  wire        is_fencei  = (fx_instr[6:0] == 7'b0001111) && (funct3 == 3'b001);

  // ---- RV32 'N' (user-level trap) URET, inline (three_sipp_es_decode untouched) ----
  // URET = 0x00200073 (funct12=0x002, funct3=0). The shared decoder flags it
  // illegal (it only knows MRET); 3-SIPP-ESR recognises it here when SECURE (the N
  // overlay below owns the user-trap return). Non-SECURE => stays illegal.
  wire        is_uret = SECURE && (fx_instr == 32'h00200073);
  wire        illegal_eff = (d_illegal && !amo_valid && !is_uret);

  // ---- W-stage final writeback value (needed for forwarding into X) --------
  // A misaligned access (word at off!=0, or half at off=3) crosses a word
  // boundary and takes a 2nd memory beat; the pipeline freezes for that cycle.
  wire mem_mis = xw_valid && (xw_mem_re || xw_mem_we) &&
                 ((xw_mem_width==2'd2 && xw_addr_lo!=2'b00) ||
                  (xw_mem_width==2'd1 && xw_addr_lo==2'b11));

  // ---- RV32 'A' atomics in W ----------------------------------------------
  // AMO<op> = 2-beat read-modify-write (load->ld_w0, store op(ld_w0,rs2)); LR is
  // a 1-beat load that arms the reservation; SC stores rs2 iff the reservation
  // matches (rd = 0 ok / 1 fail). Address = rs1 (word-aligned).
  logic            resv_valid;
  logic [XLEN-1:0] resv_addr;
  wire             amo_here    = xw_valid && xw_is_amo;
  wire [XLEN-1:0]  amo_addr_al = {xw_amo_addr[XLEN-1:2], 2'b00};
  wire             sc_ok       = resv_valid && (resv_addr == amo_addr_al);
  wire             amo_rmw_b1  = amo_here && xw_is_amo_rmw && !beat2;  // load beat
  wire             amo_rmw_b2  = amo_here && xw_is_amo_rmw && beat2;   // store beat
  wire             amo_do_load = amo_here && (xw_is_lr || amo_rmw_b1);
  wire             amo_do_store= amo_here && ((xw_is_sc && sc_ok) || amo_rmw_b2);
  logic [XLEN-1:0] amo_res;
  always_comb unique case (xw_amo_f5)
    5'b00001: amo_res = xw_amo_b;                                             // SWAP
    5'b00000: amo_res = ld_w0 + xw_amo_b;                                     // ADD
    5'b00100: amo_res = ld_w0 ^ xw_amo_b;                                     // XOR
    5'b01100: amo_res = ld_w0 & xw_amo_b;                                     // AND
    5'b01000: amo_res = ld_w0 | xw_amo_b;                                     // OR
    5'b10000: amo_res = ($signed(ld_w0) < $signed(xw_amo_b)) ? ld_w0 : xw_amo_b; // MIN
    5'b10100: amo_res = ($signed(ld_w0) > $signed(xw_amo_b)) ? ld_w0 : xw_amo_b; // MAX
    5'b11000: amo_res = (ld_w0 < xw_amo_b) ? ld_w0 : xw_amo_b;                // MINU
    5'b11100: amo_res = (ld_w0 > xw_amo_b) ? ld_w0 : xw_amo_b;                // MAXU
    default:  amo_res = xw_amo_b;
  endcase

  assign mem_beat_stall = (mem_mis && !beat2) || amo_rmw_b1;

  always_ff @(posedge clk) begin
    if (rst) resv_valid <= 1'b0;
    else if (amo_here && xw_is_lr)        begin resv_valid <= 1'b1; resv_addr <= amo_addr_al; end
    else if (amo_here && xw_is_sc)        resv_valid <= 1'b0;
    else if (xw_valid && xw_mem_we)       resv_valid <= 1'b0;
  end

  // load assembly: {word1,word0} >> off*8, then width-extract
  wire [63:0] ld_comb = beat2 ? {dmem_rdata, ld_w0} : {32'b0, dmem_rdata};
  wire [63:0] ld_sh   = ld_comb >> {xw_addr_lo, 3'b000};
  wire [7:0]  w_b = ld_sh[7:0];
  wire [15:0] w_h = ld_sh[15:0];
  wire [XLEN-1:0] w_load =
      (xw_mem_width==2'd0) ? (xw_mem_unsigned ? {24'b0,w_b} : {{24{w_b[7]}},w_b}) :
      (xw_mem_width==2'd1) ? (xw_mem_unsigned ? {16'b0,w_h} : {{16{w_h[15]}},w_h}) :
      ld_sh[31:0];
  // AMO writeback: LR -> loaded word; SC -> 0 (ok) / 1 (fail); AMO<op> -> old word
  wire [XLEN-1:0] amo_wb = xw_is_lr ? dmem_rdata
                         : xw_is_sc ? (sc_ok ? 32'd0 : 32'd1)
                         :            ld_w0;
  wire [XLEN-1:0] w_wb = amo_here    ? amo_wb
                       : xw_mem_re    ? w_load
                       :                xw_result;

  always_ff @(posedge clk) begin
    if (rst) begin beat2 <= 1'b0; ld_w0 <= 32'b0; end
    else begin
      if (mem_beat_stall) ld_w0 <= dmem_rdata;   // capture word0 (load)
      beat2 <= mem_beat_stall;
    end
  end

  // ---- forwarding: only source is the W stage -----------------------------
  wire fwd1 = xw_valid && xw_reg_we && (xw_rd!=5'd0) && (xw_rd==rs1);
  wire fwd2 = xw_valid && xw_reg_we && (xw_rd!=5'd0) && (xw_rd==rs2);
  wire [XLEN-1:0] op1 = fwd1 ? w_wb : rdata1;
  wire [XLEN-1:0] op2 = fwd2 ? w_wb : rdata2;

  // ---- ALU -----------------------------------------------------------------
  wire [XLEN-1:0] alu_a = d_use_pc  ? fx_pc  : op1;
  wire [XLEN-1:0] alu_b = d_use_imm ? id_imm : op2;
  logic [XLEN-1:0] alu_y;
  three_sipp_es_alu u_alu (.op(d_alu_op), .a(alu_a), .b(alu_b), .y(alu_y));

  logic br_taken;
  three_sipp_es_branch u_br (.br_op(d_br_op), .a(op1), .b(op2), .taken(br_taken));

  // ---- multiply/divide -----------------------------------------------------
  logic md_busy, md_done, md_inflight; logic [XLEN-1:0] md_result;
  three_sipp_es_muldiv u_md (
    .clk(clk), .rst(rst),
    .start(fx_valid && d_is_md && !md_inflight && !md_busy),
    .op(d_md_op), .a(op1), .b(op2),
    .busy(md_busy), .done(md_done), .result(md_result)
  );
  assign md_stall = fx_valid && d_is_md && !md_done;
  always_ff @(posedge clk) begin
    if (rst)          md_inflight <= 1'b0;
    else if (md_done) md_inflight <= 1'b0;
    else if (fx_valid && d_is_md && !md_busy && !md_inflight) md_inflight <= 1'b1;
  end

  // effective CSR address multiplexes normal instruction CSR with debug abstract access
  wire [11:0] csr_addr_eff = dbg_mode ? dbg_ar_regno : sys_imm12;
  wire [11:0] u_csr_addr    = csr_addr_eff;   // N user CSRs use the same addr mux
  wire [11:0] trig_csr_addr = csr_addr_eff;   // trigger CSRs use the same addr mux

  // ==========================================================================
  // N-extension: user-level traps (uepc/ucause/utvec/uscratch/utval/ustatus/
  // uie/uip + URET + medeleg delegation to U). INLINED, SECURE-only.
  // ==========================================================================
  // The shared three_sipp_es_csr models only M-mode; it cannot be extended (byte-
  // identical family build). So the N architectural state and the delegate-to-U
  // trap/return semantics live entirely here, layered over the M-mode CSR:
  //   * A synchronous exception taken in U-mode whose cause bit is set in medeleg
  //     is delivered to U-mode: save uepc/ucause/utval, ustatus.UPIE<=UIE, UIE<=0,
  //     redirect to utvec, and priv STAYS U. Because priv doesn't change, the
  //     shared CSR's trap_set is simply suppressed for delegated traps — no need
  //     to touch its priv logic at all.
  //   * URET: ustatus.UIE<=UPIE, UPIE<=1, redirect to uepc, priv stays U.
  //   * medeleg is M-only (addr[9:8]==11) so U writes to it already fault via the
  //     existing csr_priv_fault path.
  //   * The user CSRs (addr[9:8]==00) are U-readable/writable; they never touch
  //     the shared CSR (routed off it, like the trigger CSRs).
  // Non-SECURE builds elaborate none of this (USERTRAPS=0): behaviour identical.
  localparam bit USERTRAPS = SECURE;
  localparam logic [11:0] CSR_USTATUS  = 12'h000;
  localparam logic [11:0] CSR_UIE      = 12'h004;
  localparam logic [11:0] CSR_UTVEC    = 12'h005;
  localparam logic [11:0] CSR_USCRATCH = 12'h040;
  localparam logic [11:0] CSR_UEPC     = 12'h041;
  localparam logic [11:0] CSR_UCAUSE   = 12'h042;
  localparam logic [11:0] CSR_UTVAL    = 12'h043;
  localparam logic [11:0] CSR_UIP      = 12'h044;
  localparam logic [11:0] CSR_MEDELEG  = 12'h302;

  logic [XLEN-1:0] utvec_r, uepc_r, ucause_r, utval_r, uscratch_r, medeleg_r;
  logic            ustatus_uie, ustatus_upie;   // ustatus.UIE (bit0), UPIE (bit4)
  // uie/uip user-interrupt masks are modelled as storage only (no U-mode IRQ
  // delivery in this in-order embedded core — user-timer/soft/ext lines are not
  // routed to U). They read/write-back so software sees architectural CSRs.
  logic [XLEN-1:0] uie_r, uip_r;

  wire is_ucsr = USERTRAPS &&
       ((u_csr_addr==CSR_USTATUS)||(u_csr_addr==CSR_UIE)||(u_csr_addr==CSR_UTVEC)||
        (u_csr_addr==CSR_USCRATCH)||(u_csr_addr==CSR_UEPC)||(u_csr_addr==CSR_UCAUSE)||
        (u_csr_addr==CSR_UTVAL)||(u_csr_addr==CSR_UIP)||(u_csr_addr==CSR_MEDELEG));

  // N CSR read data (overrides the shared csr_rdata for these addresses)
  logic [XLEN-1:0] ucsr_rdata;
  always_comb begin
    unique case (u_csr_addr)
      CSR_USTATUS : ucsr_rdata = {27'b0, ustatus_upie, 3'b0, ustatus_uie};
      CSR_UIE     : ucsr_rdata = uie_r;
      CSR_UTVEC   : ucsr_rdata = utvec_r;
      CSR_USCRATCH: ucsr_rdata = uscratch_r;
      CSR_UEPC    : ucsr_rdata = uepc_r;
      CSR_UCAUSE  : ucsr_rdata = ucause_r;
      CSR_UTVAL   : ucsr_rdata = utval_r;
      CSR_UIP     : ucsr_rdata = uip_r;
      CSR_MEDELEG : ucsr_rdata = medeleg_r;
      default     : ucsr_rdata = '0;
    endcase
  end

  // ==========================================================================
  // Hardware triggers (RISC-V Debug "Sdtrig" / mcontrol6) — INLINED
  // ==========================================================================
  // E-class ships TRIGGERS=2. Rather than instantiate the external
  // three_sipp_es_trigger cell (which is NOT in the shared compliance file-list, so
  // adding it would break the byte-identical build), the same mcontrol6 trigger
  // register file + equal-match logic is inlined here, entirely local to
  // 3-SIPP-ESR. It mirrors three_sipp_es_trigger.sv exactly (see that cell's header for
  // the full mcontrol6 tdata1 layout). Two trigger slots, each selectable as an
  // EXECUTE (PC-match) breakpoint or a LOAD/STORE (address-match) watchpoint.
  // A match with action=1 requests Debug-Mode entry (like ebreak-to-debug);
  // action=0 raises a breakpoint exception. State never touches the shared
  // three_sipp_es_csr, so it elaborates cleanly.
  //
  //   tselect (0x7A0) — selects which slot tdata1/tdata2 addresses.
  //   tdata1  (0x7A1) — per-slot mcontrol6 control (type in [31:28]=6).
  //   tdata2  (0x7A2) — match value (a PC for execute, an address for l/s).
  //   tinfo   (0x7A4) — read-only: bit6 set (mcontrol6 supported).
  //
  // mcontrol6 tdata1 bits used: [31:28]=type(6) [21]=hit0(sticky)
  //   [15:12]=action(0=exc,1=debug) [6]=m(match in M-mode)
  //   [2]=execute [1]=store [0]=load.
  localparam int unsigned NTRIG = 2;
  localparam logic [3:0]  TTYPE_MC6 = 4'h6;
  localparam logic [11:0] CSR_TSELECT = 12'h7A0;
  localparam logic [11:0] CSR_TDATA1  = 12'h7A1;
  localparam logic [11:0] CSR_TDATA2  = 12'h7A2;
  localparam logic [11:0] CSR_TINFO   = 12'h7A4;

  logic [$clog2(NTRIG)-1:0] tselect;
  logic [XLEN-1:0]          tdata1_cfg [0:NTRIG-1];
  logic [NTRIG-1:0]         trig_hit0;
  logic [XLEN-1:0]          tdata2 [0:NTRIG-1];

  wire [XLEN-1:0] tdata1 [0:NTRIG-1];
  generate
    for (genvar g = 0; g < NTRIG; g++) begin : g_tdata1
      assign tdata1[g] = {tdata1_cfg[g][31:22], trig_hit0[g], tdata1_cfg[g][20:0]};
    end
  endgenerate

  wire is_trig_csr = (trig_csr_addr==CSR_TSELECT)||(trig_csr_addr==CSR_TDATA1)||
                     (trig_csr_addr==CSR_TDATA2)||(trig_csr_addr==CSR_TINFO);

  // trigger CSR read data (overrides the shared csr_rdata for these addresses)
  logic [XLEN-1:0] trig_csr_rdata;
  always_comb begin
    unique case (trig_csr_addr)
      CSR_TSELECT: trig_csr_rdata = {{(XLEN-$clog2(NTRIG)){1'b0}}, tselect};
      CSR_TDATA1 : trig_csr_rdata = tdata1[tselect];
      CSR_TDATA2 : trig_csr_rdata = tdata2[tselect];
      CSR_TINFO  : trig_csr_rdata = 32'h0000_0041; // type6 supported + info-valid
      default    : trig_csr_rdata = '0;
    endcase
  end

  // ---- CSR -----------------------------------------------------------------
  wire  [1:0]      csr_fn = funct3[1:0];
  wire             csr_imm_mode = funct3[2];
  wire  [XLEN-1:0] csr_src = csr_imm_mode ? {27'b0, rs1} : op1;
  logic [XLEN-1:0] csr_rdata, csr_wval;
  // effective CSR read: trigger CSRs (0x7A0..0x7A2/0x7A4) come from the local
  // trigger file, N user CSRs (ustatus/uie/utvec/... + medeleg) from the local N
  // overlay, everything else from the shared three_sipp_es_csr.
  wire [XLEN-1:0] csr_rd_eff = is_trig_csr ? trig_csr_rdata :
                               is_ucsr     ? ucsr_rdata     : csr_rdata;
  always_comb unique case (csr_fn)
    2'b01:  csr_wval = csr_src;
    2'b10:  csr_wval = csr_rd_eff |  csr_src;
    2'b11:  csr_wval = csr_rd_eff & ~csr_src;
    default: csr_wval = csr_rd_eff;
  endcase
  wire csr_do_write = d_is_csr &&
       !((csr_fn != 2'b01) && (csr_imm_mode ? (rs1==5'd0) : (rs1==5'd0)));
  wire [XLEN-1:0] csr_wdata_eff = dbg_mode ? dbg_ar_wdata : csr_wval;
  wire [XLEN-1:0] trig_wdata = csr_wdata_eff;

  // ---- privilege + PMP (security: M/U memory isolation) --------------------
  // SECURE-only. The shared three_sipp_es_csr owns priv/PMP architectural state; two
  // combinational three_sipp_es_pmp checkers validate the fetch address (fx_pc) and the
  // load/store address (alu_y) — both known here in X, exactly where every other
  // synchronous trap resolves. Default (SECURE=0) ties everything off: priv stays
  // M, no PMP logic, behaviour byte-identical.
  localparam int NPMP = 8;
  wire [1:0]   cur_priv;
  wire         fetch_m, data_m, mmwp_w, mml_w;
  wire [127:0] pmpcfg_w;
  wire [511:0] pmpaddr_w;
  wire acc_fetch_fault, acc_load_fault, acc_store_fault;
  // Privileged-operation faults from U-mode (all illegal-instruction traps):
  //  - access to a CSR above User level (address bits [9:8] != 00)
  //  - MRET (a trap-return instruction; only legal in M-mode)
  // In any mode, a CSR instruction that writes a read-only CSR (address bits
  // [11:10] == 11, e.g. cycle / mhartid) is an illegal instruction.
  // Checked here, not in the shared CSR/decode leaf cells, so they stay untouched.
  wire priv_low        = SECURE && (cur_priv != 2'b11);
  wire csr_priv_fault  = priv_low && d_is_csr && (sys_imm12[9:8] != 2'b00);
  wire mret_priv_fault = priv_low && d_is_mret;
  wire csr_ro_fault    = d_is_csr && csr_do_write && (sys_imm12[11:10] == 2'b11);
  wire priv_fault      = csr_priv_fault | mret_priv_fault | csr_ro_fault;
  // Data access checked by the PMP: loads/stores use the ALU address; AMO / LR /
  // SC address memory with rs1 (op1) and are checked the same way (LR = read,
  // SC = write, AMO<op> = read + write, reported as a store/AMO access fault).
  wire [XLEN-1:0] ls_addr = amo_valid ? op1 : alu_y;
  wire            ls_re   = d_mem_re | (amo_valid & !is_sc);
  wire            ls_we   = d_mem_we | (amo_valid & !is_lr);
  generate if (SECURE) begin : g_pmp
    wire pmp_fetch_fault, pmp_data_fault;
    three_sipp_es_pmp #(.NPMP(NPMP)) u_pmp_if (    // instruction-fetch check
      .cfg(pmpcfg_w[8*NPMP-1:0]), .addrreg(pmpaddr_w[32*NPMP-1:0]),
      .addr(fx_pc), .priv_m(fetch_m), .mmwp(mmwp_w), .mml(mml_w), .do_r(1'b0), .do_w(1'b0), .do_x(1'b1),
      .fault(pmp_fetch_fault)
    );
    three_sipp_es_pmp #(.NPMP(NPMP)) u_pmp_ls (    // load/store check (post-address)
      .cfg(pmpcfg_w[8*NPMP-1:0]), .addrreg(pmpaddr_w[32*NPMP-1:0]),
      .addr(ls_addr), .priv_m(data_m), .mmwp(mmwp_w), .mml(mml_w),
      .do_r(ls_re), .do_w(ls_we), .do_x(1'b0),
      .fault(pmp_data_fault)
    );
    assign acc_fetch_fault = fx_valid && pmp_fetch_fault;
    assign acc_load_fault  = fx_valid && ls_re && !ls_we && pmp_data_fault;
    assign acc_store_fault = fx_valid && ls_we && pmp_data_fault;
  end else begin : g_nopmp
    assign acc_fetch_fault = 1'b0;
    assign acc_load_fault  = 1'b0;
    assign acc_store_fault = 1'b0;
  end endgenerate
  wire acc_fault = acc_fetch_fault | acc_load_fault | acc_store_fault;

  // per-slot decode helpers
  function automatic logic slot_en    (input [XLEN-1:0] d1);
    slot_en = (d1[31:28]==TTYPE_MC6) && d1[6];               // type6 + match-in-M
  endfunction
  function automatic logic slot_exec  (input [XLEN-1:0] d1); slot_exec  = d1[2]; endfunction
  function automatic logic slot_store (input [XLEN-1:0] d1); slot_store = d1[1]; endfunction
  function automatic logic slot_load  (input [XLEN-1:0] d1); slot_load  = d1[0]; endfunction
  function automatic logic slot_action(input [XLEN-1:0] d1); slot_action= d1[12]; endfunction

  // ---- combinational match over both slots (equal-match only) --------------
  // chk_valid gates on a real X-stage instruction that is NOT already frozen,
  // architecturally trapping, or in/entering debug. Execute matches fx_pc;
  // load/store matches the data address alu_y.
  wire trig_chk_valid = fx_valid && !ex_freeze && !dbg_mode;
  wire trig_mem_re    = fx_valid && d_mem_re;
  wire trig_mem_we    = fx_valid && d_mem_we;

  logic            trig_exec, trig_ldst, trig_action_w;
  logic [XLEN-1:0] trig_tval_w;
  logic [NTRIG-1:0] trig_fire_mask;
  always_comb begin
    trig_exec = 1'b0; trig_ldst = 1'b0; trig_action_w = 1'b0; trig_tval_w = '0;
    trig_fire_mask = '0;
    for (int tg = 0; tg < NTRIG; tg = tg + 1) begin
      if (trig_chk_valid && slot_en(tdata1[tg])) begin
        // execute (PC) match
        if (slot_exec(tdata1[tg]) && (fx_pc == tdata2[tg])) begin
          if (!trig_exec && !trig_ldst) begin
            trig_exec = 1'b1; trig_action_w = slot_action(tdata1[tg]); trig_tval_w = fx_pc;
          end
          trig_fire_mask[tg] = 1'b1;
        end
        // load/store (address) match
        if (((slot_load(tdata1[tg])  && trig_mem_re) ||
             (slot_store(tdata1[tg]) && trig_mem_we)) &&
            (alu_y == tdata2[tg])) begin
          if (!trig_exec && !trig_ldst) begin
            trig_ldst = 1'b1; trig_action_w = slot_action(tdata1[tg]); trig_tval_w = alu_y;
          end
          trig_fire_mask[tg] = 1'b1;
        end
      end
    end
  end

  wire trig_fire     = trig_exec || trig_ldst;
  wire trig_to_debug = trig_fire &&  trig_action_w;   // action=1 -> enter debug
  wire trig_to_exc   = trig_fire && !trig_action_w;   // action=0 -> breakpoint exc

  // ---- traps ---------------------------------------------------------------
  // illegal (incl. U-mode M-CSR access) | ecall (priv-aware cause) | ebreak |
  // PMP access faults (fetch=1 / load=5 / store=7). A fetch fault takes priority
  // (the instruction never legitimately executes); mtval carries the faulting
  // address (fetch->pc, load/store->addr) or the faulting instruction word.
  // A hardware-trigger with action=1 (trig_to_debug) enters Debug Mode and is
  // handled by the debug FSM (never becomes an architectural trap). A trigger
  // with action=0 (trig_to_exc) raises a breakpoint exception, folded in here.
  wire illegal_all = illegal_eff | priv_fault;
  wire arch_trap = fx_valid && (illegal_all | d_is_ecall |
                                (d_is_ebreak & ~dcsr_ebreakm) | acc_fault);
  wire ex_trap = (arch_trap || trig_to_exc) && !trig_to_debug;
  wire [3:0] ex_cause = acc_fetch_fault ? 4'd1  :                    // instr access-fault
                        illegal_all      ? CAUSE_ILLEGAL :
                        d_is_ecall        ? (cur_priv==2'b00 ? 4'd8 : CAUSE_ECALL_M) :
                        d_is_ebreak       ? CAUSE_BREAKPOINT :
                        acc_load_fault    ? 4'd5  :                   // load access-fault
                        acc_store_fault   ? 4'd7  :                   // store access-fault
                        trig_to_exc       ? CAUSE_BREAKPOINT : CAUSE_ILLEGAL;
  wire [XLEN-1:0] ex_tval = acc_fetch_fault              ? fx_pc :
                            (acc_load_fault|acc_store_fault) ? ls_addr :
                            d_is_ebreak                   ? fx_pc :
                            illegal_all                   ? fx_instr :
                            trig_to_exc                   ? trig_tval_w : 32'b0;
  logic [XLEN-1:0] mtvec_w, mepc_w;
  wire             irq_req;
  wire [3:0]       irq_cause;
  // take an interrupt on a simple committing instruction (mepc = next sequential).
  // Debug takes priority — no interrupts while halting/halted. URET is a return
  // instruction: don't preempt it (mirrors the d_is_mret exclusion).
  wire take_irq = fx_valid && !ex_freeze && irq_req && !ex_trap && !dbg_freeze &&
                  !d_is_mret && !is_uret &&
                  !d_is_csr && !d_is_jal && !d_is_jalr && !d_is_branch;
  wire [XLEN-1:0] irq_epc = fx_pc + {29'd0, fx_len};

  // ---- N-extension: delegate-to-U decision ---------------------------------
  // A synchronous exception taken in U-mode is delivered to U iff its cause bit
  // is set in medeleg. (Interrupts are never delegated here — no U-mode IRQ
  // delivery in this core.) A delegated trap keeps priv=U, so the shared CSR's
  // M-mode trap machinery is simply suppressed (deleg gates trap_set below).
  wire deleg_trap = USERTRAPS && (cur_priv==2'b00) && ex_trap && !ex_freeze &&
                    medeleg_r[ex_cause];
  // shared (M-mode) trap fires for any non-delegated exception or a taken IRQ
  wire m_trap_set = (ex_trap && !ex_freeze && !deleg_trap) || take_irq;

  // the effective CSR write-enable (normal committing CSR instr, or debug write)
  wire csr_we_eff   = dbg_mode ? dbg_csr_we
                              : (fx_valid && csr_do_write && !ex_trap && !ex_freeze);
  // trigger CSR writes are handled by the local trigger file; route the
  // write there when the target is a trigger CSR, and keep it off the shared CSR.
  wire trig_csr_we = csr_we_eff && is_trig_csr;
  // N user-CSR writes are handled by the local N block; route them there and keep
  // them off the shared CSR (which returns 0 for these addresses).
  wire ucsr_we = csr_we_eff && is_ucsr;
  // trigger + N reads override the shared CSR read (both normal-EX and debug paths)
  assign dbg_ar_rdata = !dbg_ar_csr           ? rdata1    :  // GPR (rf_ra1 = regno)
                        (dbg_ar_regno==12'h7b0) ? dcsr_val :
                        (dbg_ar_regno==12'h7b1) ? dpc      :
                        (dbg_ar_regno==12'h7b2) ? dscratch0:
                        is_trig_csr             ? trig_csr_rdata :
                        is_ucsr                 ? ucsr_rdata :
                        csr_rdata;                            // CSR (addr = regno)

  // ---- trigger CSR write + sticky hit0 -------------------------------------
  // Write-enable is qualified the same way the shared CSR write is: a committing
  // CSR instruction in normal mode, or a debug abstract CSR write while halted.
  always_ff @(posedge clk) begin
    if (rst) begin
      tselect <= '0;
      trig_hit0 <= '0;
      for (int tg = 0; tg < NTRIG; tg = tg + 1) begin
        tdata1_cfg[tg] <= {TTYPE_MC6, 28'd0};   // type=6, all disabled
        tdata2[tg]     <= '0;
      end
    end else begin
      if (trig_csr_we) begin
        unique case (trig_csr_addr)
          CSR_TSELECT: if (trig_wdata < NTRIG) tselect <= trig_wdata[$clog2(NTRIG)-1:0];
          CSR_TDATA1 : begin
            tdata1_cfg[tselect] <= {TTYPE_MC6, trig_wdata[27:0]}; // force type6
            trig_hit0[tselect]  <= trig_wdata[21];                 // software write
          end
          CSR_TDATA2 : tdata2[tselect] <= trig_wdata;
          default    : ;
        endcase
      end
      // sticky hit0 (bit 21) on the slot(s) that fired and were acted upon
      if (trig_fire) begin
        for (int tg = 0; tg < NTRIG; tg = tg + 1)
          if (trig_fire_mask[tg]) trig_hit0[tg] <= 1'b1;
      end
    end
  end

  // ---- N-extension architectural state: write + delegated-trap + URET -------
  generate if (USERTRAPS) begin : g_ntrap
    always_ff @(posedge clk) begin
      if (rst) begin
        utvec_r<='0; uepc_r<='0; ucause_r<='0; utval_r<='0; uscratch_r<='0;
        medeleg_r<='0; uie_r<='0; uip_r<='0; ustatus_uie<=1'b0; ustatus_upie<=1'b0;
      end else begin
        // priority: a delegated trap / URET this cycle over a software CSR write
        if (deleg_trap) begin
          uepc_r       <= fx_pc;
          ucause_r     <= {28'b0, ex_cause};
          utval_r      <= ex_tval;
          ustatus_upie <= ustatus_uie;   // save UIE
          ustatus_uie  <= 1'b0;          // disable U interrupts in handler
        end else if (fx_valid && is_uret && !ex_freeze) begin
          ustatus_uie  <= ustatus_upie;  // restore UIE
          ustatus_upie <= 1'b1;
        end else if (ucsr_we) begin
          unique case (u_csr_addr)
            CSR_USTATUS : begin ustatus_uie<=csr_wdata_eff[0]; ustatus_upie<=csr_wdata_eff[4]; end
            CSR_UIE     : uie_r      <= csr_wdata_eff;
            CSR_UTVEC   : utvec_r    <= csr_wdata_eff;
            CSR_USCRATCH: uscratch_r <= csr_wdata_eff;
            CSR_UEPC    : uepc_r     <= csr_wdata_eff;
            CSR_UCAUSE  : ucause_r   <= csr_wdata_eff;
            CSR_UTVAL   : utval_r    <= csr_wdata_eff;
            CSR_UIP     : uip_r      <= csr_wdata_eff;
            CSR_MEDELEG : medeleg_r  <= csr_wdata_eff;
            default     : ;
          endcase
        end
      end
    end
  end else begin : g_no_ntrap
    // tie N state to 0 so reads are clean and delegation never fires
    always_comb begin
      utvec_r='0; uepc_r='0; ucause_r='0; utval_r='0; uscratch_r='0;
      medeleg_r='0; uie_r='0; uip_r='0; ustatus_uie=1'b0; ustatus_upie=1'b0;
    end
  end endgenerate

  three_sipp_es_csr #(.MISA_VAL((32'b01<<30)|(1<<8)|(1<<12)|(1<<2)|(1<<1)|(1<<0)),  // RV32IMAC + Zbb (B)
                .U_MODE(SECURE), .PMP_REGIONS(SECURE ? NPMP : 0)) u_csr (   // secure opt
    .clk(clk), .rst(rst),
    .csr_addr(csr_addr_eff), .csr_rdata(csr_rdata),
    .csr_we(csr_we_eff && !is_trig_csr && !is_ucsr),
    .csr_wdata(csr_wdata_eff),
    .priv_o(cur_priv), .fetch_m_o(fetch_m), .data_m_o(data_m), .mmwp_o(mmwp_w), .mml_o(mml_w),
    .pmpcfg_o(pmpcfg_w), .pmpaddr_o(pmpaddr_w),
    // delegated-to-U exceptions are handled by the local N block, not the M CSR
    .trap_set(m_trap_set),
    .trap_cause(ex_trap ? ex_cause : irq_cause),
    .trap_interrupt(take_irq),
    .trap_epc(ex_trap ? fx_pc : irq_epc), .trap_tval(ex_tval),
    .mret(fx_valid && d_is_mret && !mret_priv_fault && !ex_freeze), .retire(xw_valid && !mem_beat_stall),
    .irq_timer(irq_timer), .irq_soft(irq_soft), .irq_ext(irq_ext),
    .irq_req(irq_req), .irq_cause(irq_cause),
    .mtvec_o(mtvec_w), .mepc_o(mepc_w)
  );

  // ---- redirect ------------------------------------------------------------
  wire [XLEN-1:0] jalr_t = (op1 + id_imm) & ~32'd1;
  wire [XLEN-1:0] btgt   = fx_pc + id_imm;
  wire [XLEN-1:0] seq_nx = fx_pc + {29'd0, fx_len};
  // actual vs predicted next-PC for the branch/jal in X
  wire            br_actual_taken = d_is_jal || (d_is_branch && br_taken);
  wire [XLEN-1:0] br_actual_next  = br_actual_taken ? btgt : seq_nx;
  wire [XLEN-1:0] pred_next       = fx_pred_taken ? fx_pred_target : seq_nx;
  wire            mispredict = BPRED && (d_is_branch || d_is_jal) &&
                               (br_actual_next != pred_next);
  always_comb begin
    redirect = 1'b0; redirect_target = '0;
    if (fx_valid && !ex_freeze) begin
      // delegated-to-U exceptions vector to utvec; non-delegated to mtvec.
      if      (ex_trap)                 begin redirect=1; redirect_target = deleg_trap ? utvec_r : mtvec_w; end
      else if (is_uret)                 begin redirect=1; redirect_target=uepc_r; end
      else if (d_is_mret)               begin redirect=1; redirect_target=mepc_w; end
      else if (d_is_jalr) begin  // RAS-predicted returns: flush only on misprediction
        redirect = BPRED ? (jalr_t != (fx_pred_taken ? fx_pred_target : seq_nx)) : 1'b1;
        redirect_target = jalr_t;
      end
      else if (d_is_jal || d_is_branch) begin
        // predicted control flow: with BPRED, flush only on misprediction; else
        // (static) redirect whenever actually taken. Target is the resolved next-PC.
        redirect        = BPRED ? mispredict : br_actual_taken;
        redirect_target = br_actual_next;
      end
      else if (is_fencei) begin
        redirect        = 1'b1;
        redirect_target = seq_nx;
      end
      else if (take_irq)                begin redirect=1; redirect_target=mtvec_w; end
    end
  end

  // ---- branch-predictor update signals (driven from X) ---------------------
  always_comb begin
    bp_upd_en     = fx_valid && !ex_freeze && !ex_trap && (d_is_branch || d_is_jal);
    bp_upd_taken  = br_actual_taken;
    bp_upd_pc     = fx_pc;
    bp_upd_target = btgt;                 // branch/jal are PC-relative
    bp_upd_isjal  = d_is_jal;
    bp_upd_ghist  = fx_ghist;
  end

  // ---- branch-predictor state update (BHT gshare + BTB + GHR) --------------
  generate if (BPRED) begin : g_bpred
    wire [BHT_BITS-1:0] bht_wr_idx = bp_upd_pc[BHT_BITS:1] ^ bp_upd_ghist[BHT_BITS-1:0];
    wire [BTB_BITS-1:0] btb_wr_idx = bp_upd_pc[BTB_BITS:1];
    wire [BTB_TAGW-1:0] btb_wr_tag = bp_upd_pc[XLEN-1:BTB_BITS+1];
    integer bi;
    always_ff @(posedge clk) begin
      if (rst) begin
        ghist <= '0;
        for (bi = 0; bi < BHT_ENTRIES; bi = bi + 1) bht[bi] = 2'b01;       // weakly N-T (blocking: Verilator 5.020 BLKLOOPINIT)
        for (bi = 0; bi < BTB_ENTRIES; bi = bi + 1) btb_valid[bi] = 1'b0;
      end else if (bp_upd_en) begin
        if (d_is_branch) begin   // conditional: train the 2-bit counter + history
          if (bp_upd_taken) bht[bht_wr_idx] <= (bht[bht_wr_idx]==2'b11) ? 2'b11 : bht[bht_wr_idx]+2'b01;
          else              bht[bht_wr_idx] <= (bht[bht_wr_idx]==2'b00) ? 2'b00 : bht[bht_wr_idx]-2'b01;
          ghist <= {ghist[GHR_BITS-2:0], bp_upd_taken};
        end
        if (bp_upd_taken) begin  // install/refresh the target on a taken control-flow instr
          btb_valid[btb_wr_idx] <= 1'b1;
          btb_tag  [btb_wr_idx] <= btb_wr_tag;
          btb_tgt  [btb_wr_idx] <= bp_upd_target;
          btb_isjal[btb_wr_idx] <= bp_upd_isjal;
        end
      end
    end
  end endgenerate

  // ---- X result (non-mem) + store data/be ----------------------------------
  logic [XLEN-1:0] x_result;
  always_comb unique case (d_wb_sel)
    WB_PC4 : x_result = fx_pc + {29'd0, fx_len};   // link = PC + (2 or 4)
    WB_CSR : x_result = csr_rd_eff;
    WB_MD  : x_result = md_result;
    default: x_result = alu_y;
  endcase

  wire [1:0] st_off = alu_y[1:0];
  wire [3:0] x_be = !d_mem_we ? 4'b0000 :
                    (d_mem_width==2'd0) ? (4'b0001 << st_off) :
                    (d_mem_width==2'd1) ? (st_off[1] ? 4'b1100 : 4'b0011) : 4'b1111;
  wire [XLEN-1:0] x_store = (d_mem_width==2'd0) ? {4{op2[7:0]}} :
                           (d_mem_width==2'd1) ? {2{op2[15:0]}} : op2;

  // EBREAK-to-debug is squashed (converted to a halt, never advances to W)
  wire x_commit = fx_valid && !md_stall && !ebreak_to_debug;

  // ==========================================================================
  // X/W register
  // ==========================================================================
  always_ff @(posedge clk) begin
    if (rst) xw_valid <= 1'b0;
    else if (mem_beat_stall) begin
      xw_valid <= xw_valid;      // hold the misaligned load/store in W for its 2nd beat
    end else begin
      xw_valid        <= x_commit;
      xw_pc           <= fx_pc;
      xw_instr        <= fx_instr;
      xw_result       <= x_result;
      xw_store_data   <= x_store;
      xw_be           <= x_be;
      xw_addr_lo      <= alu_y[1:0];
      xw_rd           <= rd;
      xw_reg_we       <= (d_reg_we || amo_valid) && !ex_trap;   // AMO/LR/SC write rd
      xw_mem_re       <= d_mem_re && !ex_trap;
      xw_mem_we       <= d_mem_we && !ex_trap;
      xw_mem_width    <= d_mem_width;
      xw_mem_unsigned <= d_mem_unsigned;
      xw_is_amo       <= amo_valid && !ex_trap;
      xw_is_lr        <= is_lr;
      xw_is_sc        <= is_sc;
      xw_is_amo_rmw   <= is_amo_rmw;
      xw_amo_f5       <= amo_f5;
      xw_amo_addr     <= op1;   // AMO address = rs1
      xw_amo_b        <= op2;   // AMO operand/SC data = rs2
    end
  end

  // ==========================================================================
  // W stage — memory + writeback
  // ==========================================================================
  // raw store magnitude recovered from the (replicated) store data, placed in an
  // 8-byte window at the byte offset — low word on beat0, high word on beat2.
  wire [31:0] st_raw = (xw_mem_width==2'd0) ? {24'b0, xw_store_data[7:0]} :
                       (xw_mem_width==2'd1) ? {16'b0, xw_store_data[15:0]} :
                                              xw_store_data;
  wire [63:0] st_win = {32'b0, st_raw} << {xw_addr_lo, 3'b000};
  wire [7:0]  be_win = (((xw_mem_width==2'd0)?8'h01:
                         (xw_mem_width==2'd1)?8'h03:8'h0F)) << xw_addr_lo;

  assign dmem_addr  = amo_here ? amo_addr_al
                               : {xw_result[XLEN-1:2], 2'b00} + (beat2 ? 32'd4 : 32'd0);
  assign dmem_re    = (xw_valid && xw_mem_re) || amo_do_load;
  assign dmem_we    = (xw_valid && xw_mem_we) || amo_do_store;
  assign dmem_be    = amo_here ? 4'b1111 : (beat2 ? be_win[7:4] : be_win[3:0]);
  assign dmem_wdata = amo_rmw_b2              ? amo_res     // AMO<op>: op(old,rs2)
                    : (amo_here && xw_is_sc)  ? xw_amo_b    // SC: store rs2
                    : (beat2 ? st_win[63:32] : st_win[31:0]);

  assign wb_value = w_wb;                 // (also forwarded into X)
  assign rf_we    = xw_valid && xw_reg_we && (xw_rd != 5'd0) && !mem_beat_stall;
  assign rf_wa    = xw_rd;

  assign retire_valid  = xw_valid && !mem_beat_stall;
  assign retire_pc     = xw_pc;
  assign retire_instr  = xw_instr;
  assign retire_rd_we  = rf_we;
  assign retire_rd     = xw_rd;
  assign retire_rd_val = w_wb;

  // ---- debug controller: drain-to-halt / resume / single-step --------------
  always_ff @(posedge clk) begin
    if (rst) begin
      dbg_mode<=1'b0; halt_req<=1'b0; step_active<=1'b0;
      dpc<='0; dscratch0<='0; dcsr_ebreakm<=1'b0; dcsr_step<=1'b0; dcsr_cause<=3'd0;
      dbg_ar_busy<=1'b0; dbg_ar_done<=1'b0; dpc_cap_valid<=1'b0; dpc_cap<='0;
    end else begin
      dbg_ar_done <= 1'b0;
      // abstract Access-Register servicing (while halted)
      if (dbg_do) begin
        dbg_ar_busy <= 1'b1;
        dbg_ar_done <= 1'b1;
        if (dbg_ar_write && dbg_ar_csr) unique case (dbg_ar_regno)
          12'h7b0: begin dcsr_ebreakm<=dbg_ar_wdata[15]; dcsr_step<=dbg_ar_wdata[2]; end
          12'h7b1: dpc       <= dbg_ar_wdata;
          12'h7b2: dscratch0 <= dbg_ar_wdata;
          default: ;
        endcase
      end
      if (!dbg_ar_valid) dbg_ar_busy <= 1'b0;

      // request a halt (external, ebreak, or one-step-done); fetch is already
      // frozen combinationally via dbg_freeze this same cycle.
      if (!dbg_mode) begin
        if (dbg_haltreq)     halt_req <= 1'b1;
        if (ebreak_to_debug) begin halt_req<=1'b1; dpc_cap<=fx_pc; dpc_cap_valid<=1'b1; end
        if (step_halt_now)   halt_req <= 1'b1;
      end

      // enter debug mode once the pipeline has drained
      if (halt_req && !dbg_mode && pipe_drained) begin
        dbg_mode      <= 1'b1;
        halt_req      <= 1'b0;
        dpc           <= dpc_cap_valid ? dpc_cap : pc;
        dcsr_cause    <= dpc_cap_valid ? 3'd1 : (step_active ? 3'd4 : 3'd3);
        step_active   <= 1'b0;
        dpc_cap_valid <= 1'b0;
      end

      // resume (optionally armed for a single step)
      if (dbg_mode && dbg_resumereq) begin
        dbg_mode    <= 1'b0;
        step_active <= dcsr_step;
      end
    end
  end

  // ---- RISC-V Formal Interface --------------------------------------------
`ifdef RISCV_FORMAL
  // Fields captured at the X->W boundary (alongside the main X/W register), so
  // they describe the *retiring* instruction when xw_valid is high in W.
  logic [4:0]      xw_rs1, xw_rs2;
  logic [XLEN-1:0] xw_rs1v, xw_rs2v, xw_pcw, xw_memaddr;
  logic            xw_trap, xw_entry, xw_uses_rs2;
  logic [1:0]      xw_mwidth;
  logic [63:0]     rvfi_order_r;
  logic            rvfi_intr_r;

  // next PC of the X-stage instruction (== pc_rdata of the next retire)
  // architectural next-PC (independent of whether prediction avoided a flush)
  wire [XLEN-1:0] x_pc_wdata = (d_is_branch || d_is_jal) ? br_actual_next
                             : d_is_jalr ? jalr_t
                             : redirect ? redirect_target : (fx_pc + {29'd0, fx_len});

  always_ff @(posedge clk) begin
    if (rst) begin
      rvfi_order_r <= 64'd0; rvfi_intr_r <= 1'b0;
    end else begin
      if (x_commit) begin
        xw_rs1     <= rs1;        xw_rs2  <= d_uses_rs2 ? rs2 : 5'd0;
        xw_rs1v    <= op1;        xw_rs2v <= d_uses_rs2 ? op2 : 32'd0;
        xw_pcw     <= x_pc_wdata; xw_trap <= ex_trap;
        xw_entry   <= ex_trap || take_irq;
        xw_memaddr <= alu_y;      xw_mwidth <= d_mem_width;
        xw_uses_rs2<= d_uses_rs2;
      end
      if (xw_valid) begin
        rvfi_order_r <= rvfi_order_r + 64'd1;
        rvfi_intr_r  <= xw_entry;   // next retire is the trap/IRQ handler's first
      end
    end
  end

  // load byte mask from width + low address bits
  wire [3:0] rmask = (xw_mwidth==2'd0) ? (4'b0001 << xw_addr_lo) :
                     (xw_mwidth==2'd1) ? (xw_addr_lo[1] ? 4'b1100 : 4'b0011) : 4'b1111;

  assign rvfi_valid     = xw_valid;
  assign rvfi_order     = rvfi_order_r;
  assign rvfi_insn      = xw_instr;
  assign rvfi_trap      = xw_trap;
  assign rvfi_halt      = 1'b0;
  assign rvfi_intr      = rvfi_intr_r;
  assign rvfi_mode      = 2'b11;
  assign rvfi_ixl       = 2'b01;
  assign rvfi_rs1_addr  = xw_rs1;
  assign rvfi_rs2_addr  = xw_rs2;
  assign rvfi_rs1_rdata = xw_rs1v;
  assign rvfi_rs2_rdata = xw_rs2v;
  assign rvfi_rd_addr   = rf_we ? xw_rd : 5'd0;
  assign rvfi_rd_wdata  = rf_we ? w_wb  : 32'd0;
  assign rvfi_pc_rdata  = xw_pc;
  assign rvfi_pc_wdata  = xw_pcw;
  assign rvfi_mem_addr  = {xw_memaddr[XLEN-1:2], 2'b00};
  assign rvfi_mem_rmask = xw_mem_re ? rmask  : 4'd0;
  assign rvfi_mem_wmask = xw_mem_we ? xw_be  : 4'd0;
  assign rvfi_mem_rdata = dmem_rdata;
  assign rvfi_mem_wdata = xw_store_data;
`endif
endmodule
