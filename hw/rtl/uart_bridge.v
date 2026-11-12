// UART <-> vcc_core register-bus bridge (8N1, little-endian).
// Commands (host -> FPGA):
//   'W' a[4] d[4]          write word                  reply 'K'
//   'R' a[4]               read word                   reply d[4]
//   'P' a[4] n[4] d[4n]    write n words to the SAME address (point stream)   reply 'K'
//   'M' a[4] n[4]          read n words from a, a+4, ...                     reply d[4n]
// A 1 KiB RX FIFO absorbs back-pressure (e.g. the 32K-cycle table clear at build start).
`timescale 1ns/1ps
module uart_rx #(parameter CLKS_PER_BIT = 109) (
  input wire clk, input wire rst, input wire rx,
  output reg valid, output reg [7:0] data
);
  reg [1:0] sync; reg [15:0] cnt; reg [3:0] bitn; reg busy; reg [7:0] sh;
  always @(posedge clk) begin
    sync <= {sync[0], rx};
    valid <= 1'b0;
    if (rst) begin busy <= 1'b0; sync <= 2'b11; end
    else if (!busy) begin
      if (!sync[1]) begin busy <= 1'b1; cnt <= CLKS_PER_BIT / 2; bitn <= 0; end
    end else if (cnt != 0) cnt <= cnt - 1'b1;
    else begin
      cnt <= CLKS_PER_BIT - 1;
      if (bitn == 0) begin if (sync[1]) busy <= 1'b0; bitn <= 1; end          // false start
      else if (bitn <= 8) begin sh <= {sync[1], sh[7:1]}; bitn <= bitn + 1'b1; end
      else begin busy <= 1'b0; if (sync[1]) begin valid <= 1'b1; data <= sh; end end
    end
  end
endmodule

module uart_tx #(parameter CLKS_PER_BIT = 109) (
  input wire clk, input wire rst, input wire start, input wire [7:0] data,
  output reg tx, output wire busy
);
  reg [15:0] cnt; reg [3:0] bitn; reg act; reg [9:0] sh;
  assign busy = act;
  always @(posedge clk) begin
    if (rst) begin act <= 1'b0; tx <= 1'b1; end
    else if (!act) begin
      tx <= 1'b1;
      if (start) begin act <= 1'b1; sh <= {1'b1, data, 1'b0}; cnt <= 0; bitn <= 0; end
    end else if (cnt != 0) cnt <= cnt - 1'b1;
    else begin
      tx <= sh[0]; sh <= {1'b1, sh[9:1]}; cnt <= CLKS_PER_BIT - 1;
      if (bitn == 10) begin act <= 1'b0; tx <= 1'b1; end
      bitn <= bitn + 1'b1;
    end
  end
endmodule

// Idle timeout: a command left half-received for IDLE_CYCLES (link dropped mid-command)
// is discarded so the next byte is parsed as a fresh command.
module uart_bridge #(parameter CLKS_PER_BIT = 109, parameter IDLE_BITS = 23) (
  input  wire        clk,
  input  wire        rst,
  input  wire        rx,
  output wire        tx,
  output reg         bus_req,
  output reg         bus_we,
  output reg  [23:0] bus_addr,
  output reg  [31:0] bus_wdata,
  input  wire        bus_ack,
  input  wire [31:0] bus_rdata
);
  // ---- RX + FIFO ----
  wire       rx_v; wire [7:0] rx_d;
  uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (.clk(clk), .rst(rst), .rx(rx), .valid(rx_v), .data(rx_d));
  reg [7:0]  fifo [0:1023];
  reg [10:0] wp, rp;
  wire       f_empty = (wp == rp);
  wire [7:0] f_head = fifo[rp[9:0]];
  reg        f_pop;
  always @(posedge clk) begin
    if (rst) begin wp <= 0; rp <= 0; end
    else begin
      if (rx_v) begin fifo[wp[9:0]] <= rx_d; wp <= wp + 1'b1; end
      if (f_pop) rp <= rp + 1'b1;
    end
  end

  // ---- TX ----
  reg        t_start; reg [7:0] t_data; wire t_busy;
  uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (.clk(clk), .rst(rst), .start(t_start), .data(t_data), .tx(tx), .busy(t_busy));

  // ---- command FSM ----
  localparam [3:0] C_CMD = 0, C_ADDR = 1, C_N = 2, C_DATA = 3, C_BUS = 4, C_WAIT = 5, C_TX = 6, C_TXW = 7, C_NEXT = 8;
  reg [3:0]  cs;
  reg [7:0]  cmd;
  reg [1:0]  bi;
  reg [31:0] addr, n, word;
  reg [1:0]  txi;
  reg        txk;   // sending the single 'K'
  reg [IDLE_BITS:0] idle;
  wire mid_rx = (cs == C_ADDR) || (cs == C_N) || (cs == C_DATA);
  always @(posedge clk) begin
    f_pop <= 1'b0; t_start <= 1'b0; bus_req <= 1'b0;
    if (rst) begin cs <= C_CMD; idle <= 0; end
    else if (mid_rx && idle[IDLE_BITS]) begin cs <= C_CMD; idle <= 0; end
    else begin
    idle <= (!mid_rx || !f_empty) ? 0 : idle + 1'b1;
    case (cs)
      C_CMD: if (!f_empty && !f_pop) begin
        f_pop <= 1'b1; cmd <= f_head; bi <= 0;
        if (f_head == "W" || f_head == "R" || f_head == "P" || f_head == "M") cs <= C_ADDR;
      end
      C_ADDR: if (!f_empty && !f_pop) begin
        f_pop <= 1'b1; addr <= {f_head, addr[31:8]}; bi <= bi + 1'b1;
        if (bi == 2'd3) begin
          if (cmd == "W") cs <= C_DATA;
          else if (cmd == "R") begin n <= 32'd1; cs <= C_BUS; end
          else cs <= C_N;
        end
      end
      C_N: if (!f_empty && !f_pop) begin
        f_pop <= 1'b1; n <= {f_head, n[31:8]}; bi <= bi + 1'b1;
        if (bi == 2'd3) cs <= (cmd == "P") ? ((({f_head, n[31:8]}) == 0) ? C_TX : C_DATA) : C_BUS;
        if (bi == 2'd3 && cmd == "P") begin txk <= 1'b1; end
        if (bi == 2'd3 && cmd == "M" && {f_head, n[31:8]} == 0) cs <= C_CMD;
      end
      C_DATA: if (!f_empty && !f_pop) begin
        f_pop <= 1'b1; word <= {f_head, word[31:8]}; bi <= bi + 1'b1;
        if (bi == 2'd3) begin if (cmd == "W") n <= 32'd1; cs <= C_BUS; end
      end
      C_BUS: begin
        bus_req <= 1'b1; bus_we <= (cmd == "W" || cmd == "P"); bus_addr <= addr[23:0]; bus_wdata <= word;
        cs <= C_WAIT;
      end
      C_WAIT: if (bus_ack) begin
        if (cmd == "R" || cmd == "M") begin word <= bus_rdata; txi <= 0; txk <= 1'b0; cs <= C_TX; end
        else begin
          n <= n - 1'b1;
          if (n == 32'd1) begin txk <= 1'b1; cs <= C_TX; end
          else cs <= C_DATA;
        end
      end
      C_TX: if (!t_busy && !t_start) begin
        t_start <= 1'b1;
        if (txk) begin t_data <= "K"; cs <= C_TXW; end
        else begin t_data <= word[8*txi +: 8]; cs <= C_TXW; end
      end
      C_TXW: if (!t_busy && !t_start) begin
        if (txk) cs <= C_CMD;
        else if (txi != 2'd3) begin txi <= txi + 1'b1; cs <= C_TX; end
        else begin
          n <= n - 1'b1;
          if (n == 32'd1) cs <= C_CMD;
          else begin addr <= addr + 32'd4; cs <= C_BUS; end
        end
      end
      default: cs <= C_CMD;
    endcase
    end
  end
endmodule
