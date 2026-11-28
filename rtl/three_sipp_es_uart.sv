// ============================================================================
// three_sipp_es_uart.sv -- 3-SIPP-ESR-owned 8-N-1 UART (console).
//
// Owned by 3-SIPP-ESR. Word-addressed register map
// (the SoC presents word-aligned CPU accesses):
//   0x0 W : tx data byte (writing pushes the byte to the TX shift register)
//   0x0 R : status  bit0 = tx_busy (shifter active)
//                   bit1 = rx_ready
//                   bit2 = tx_empty (== !tx_busy)
//                   bit3 = rx_overrun (sticky, cleared on status read)
//   0x4 R : rx data byte  (reading clears rx_ready)
//   0x8 RW: baud_div      (clocks per bit; low byte written each store)
//
// Simulation: writes to tx data ALSO $write the byte to stdout so the sim
// transcript captures the console with no extra harness. This is a real
// synthesizable shifter (start + 8 data + stop, LSB-first) for FPGA; the
// $write is guarded translate_off / SIMULATION so it is sim-only.
// ============================================================================


module three_sipp_es_uart #(
  parameter int CLK_HZ       = 50_000_000,
  parameter int BAUD         = 115_200,
  parameter int BAUD_DIV_RST = CLK_HZ / BAUD
)(
  input  logic       clk, rst,
  input  logic [3:0] addr,     // byte offset within the UART window
  input  logic [7:0] wdata,
  input  logic       we,
  input  logic       re,
  output logic [7:0] rdata,
  output logic       serial_tx,
  input  logic       serial_rx,
  output logic       tx_irq,
  output logic       rx_irq
);
  logic [15:0] baud_div;

  // ---- TX --------------------------------------------------------------------
  logic [9:0]  tx_shift;
  logic [3:0]  tx_bit_count;
  logic [15:0] tx_baud_cnt;
  logic        tx_busy_q;
  wire  tx_load = we && (addr[3:2] == 2'b00) && !tx_busy_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      tx_shift <= 10'h3FF; tx_bit_count <= 4'd0; tx_baud_cnt <= 16'd0;
      tx_busy_q <= 1'b0; serial_tx <= 1'b1; tx_irq <= 1'b0;
    end else begin
      tx_irq <= 1'b0;
      if (tx_load) begin
        tx_shift     <= {1'b1, wdata, 1'b0};   // {stop, data[7:0], start}, LSB first
        tx_bit_count <= 4'd10;
        tx_baud_cnt  <= baud_div - 16'd1;
        tx_busy_q    <= 1'b1;
        // synthesis translate_off
        $write("%c", wdata);
        // synthesis translate_on
      end else if (tx_busy_q) begin
        if (tx_baud_cnt == 16'd0) begin
          serial_tx    <= tx_shift[0];
          tx_shift     <= {1'b1, tx_shift[9:1]};
          tx_bit_count <= tx_bit_count - 4'd1;
          tx_baud_cnt  <= baud_div - 16'd1;
          if (tx_bit_count == 4'd1) begin tx_busy_q <= 1'b0; tx_irq <= 1'b1; end
        end else begin
          tx_baud_cnt <= tx_baud_cnt - 16'd1;
        end
      end
    end
  end

  // ---- RX --------------------------------------------------------------------
  logic [1:0]  rx_sync;
  logic        rx_busy_q;
  logic [3:0]  rx_bit_count;
  logic [15:0] rx_baud_cnt;
  logic [7:0]  rx_shift, rx_data_q;
  logic        rx_ready_q, rx_overrun_q;

  always_ff @(posedge clk) begin
    if (rst) rx_sync <= 2'b11; else rx_sync <= {rx_sync[0], serial_rx};
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      rx_busy_q<=0; rx_bit_count<=0; rx_baud_cnt<=0; rx_shift<=0;
      rx_data_q<=0; rx_ready_q<=0; rx_overrun_q<=0; rx_irq<=0;
    end else begin
      rx_irq <= 1'b0;
      if (re && addr[3:2] == 2'b01) rx_ready_q   <= 1'b0;
      if (re && addr[3:2] == 2'b00) rx_overrun_q <= 1'b0;
      if (!rx_busy_q) begin
        if (rx_sync[1] == 1'b0) begin
          rx_busy_q <= 1'b1; rx_bit_count <= 4'd8;
          rx_baud_cnt <= {1'b0, baud_div[15:1]} + baud_div - 16'd1;
        end
      end else if (rx_baud_cnt == 16'd0) begin
        if (rx_bit_count != 4'd0) begin
          rx_shift <= {rx_sync[1], rx_shift[7:1]};
          rx_bit_count <= rx_bit_count - 4'd1; rx_baud_cnt <= baud_div - 16'd1;
        end else begin
          rx_busy_q <= 1'b0; rx_data_q <= rx_shift;
          if (rx_ready_q) rx_overrun_q <= 1'b1;
          rx_ready_q <= 1'b1; rx_irq <= 1'b1;
        end
      end else rx_baud_cnt <= rx_baud_cnt - 16'd1;
    end
  end

  // ---- baud_div (0x8) --------------------------------------------------------
  // A write loads the full divisor from the byte (zero-extended). Sim programs a
  // tiny divisor for a fast console; realistic FPGA baud comes from BAUD_DIV_RST.
  always_ff @(posedge clk) begin
    if (rst) baud_div <= BAUD_DIV_RST[15:0];
    else if (we && addr[3:2] == 2'b10) baud_div <= {8'h0, wdata};
  end

  always_comb begin
    case (addr[3:2])
      2'b00:   rdata = {4'b0, rx_overrun_q, ~tx_busy_q, rx_ready_q, tx_busy_q};
      2'b01:   rdata = rx_data_q;
      2'b10:   rdata = baud_div[7:0];
      default: rdata = 8'h0;
    endcase
  end
endmodule

`default_nettype wire
