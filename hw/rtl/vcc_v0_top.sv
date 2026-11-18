// Integrated FPGA top: our format-v0 build engine + PLE-CSE ple_cse_top (N_CU configurable), driven over UART.
// 100 MHz board clock -> MMCM -> 75 MHz system clock (PLE-CSE closes timing at 75 MHz out of context).
//
// UART bridge byte address map (uart_bridge commands W / R / P / M):
//   0x000000-0x000FFF  PLE host interface (AXI-Lite, see PLE-CSE accel/rtl/ple/ple_top.sv)
//   0x010000 BE_CTRL   W  bit0 start build
//   0x010004 BE_STATUS R  [0] busy [7:4] status (0 ok, 4 capacity, 5 invalid input)
//   0x010008 BE_NPTS   RW number of raw points of the next build
//   0x01000C BE_WBUF   RW environment buffer the next build writes (0/1)
//   0x010010 KEPT  0x010014 NCELLS  0x010018 CULLED  0x01001C CYCLES  0x010020 PROBES   R
//   0x01003C BE_PT     W  raw point stream, binary32 words x, y, z, x, ...
//   0x0100FC ID        R
module vcc_v0_uart_top #(
  parameter int N_CU         = 1,
  parameter int CLKS_PER_BIT = 81,          // 75 MHz / 921600 baud
  parameter int IDLE_BITS    = 23
) (
  input  logic       clk,                   // 100 MHz board clock
  input  logic       rst_btn,
  input  logic       uart_rx,
  output logic       uart_tx,
  output logic [3:0] led
);
  import ple_cse_pkg::*;

  // ---------------- clock: 100 -> 75 MHz ----------------
  logic clk_fb, clk75_u, clk75, locked;
`ifdef SIM_NO_MMCM
  assign clk75 = clk; assign locked = 1'b1;
`else
  MMCME2_BASE #(.CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT_F(9.0), .DIVCLK_DIVIDE(1), .CLKOUT0_DIVIDE_F(12.0)) u_mmcm (
    .CLKIN1(clk), .CLKFBIN(clk_fb), .CLKFBOUT(clk_fb), .CLKOUT0(clk75_u), .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0),
    .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(),
    .CLKOUT6(), .CLKFBOUTB());
  BUFG u_bufg (.I(clk75_u), .O(clk75));
`endif
  logic [3:0] rs = 4'hF;
  always_ff @(posedge clk75) rs <= {rs[2:0], rst_btn | ~locked};
  wire rst = rs[3];

  // ---------------- UART bridge ----------------
  logic        req, we, ack;
  logic [23:0] addr;
  logic [31:0] wd, rd;
  uart_bridge #(.CLKS_PER_BIT(CLKS_PER_BIT), .IDLE_BITS(IDLE_BITS)) u_br (
    .clk(clk75), .rst(rst), .rx(uart_rx), .tx(uart_tx),
    .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wd), .bus_ack(ack), .bus_rdata(rd));

  // ---------------- build engine ----------------
  logic        be_start, be_busy, be_done, be_wr_en, be_wr_buf, w_valid, w_ready, pt_ready;
  logic [31:0] be_npts, be_culled, be_probes, be_cycles, pt_word;
  logic        be_wbuf;
  logic [3:0]  be_status;
  logic [15:0] be_kept;
  logic [12:0] be_cells;
  logic [1:0]  be_wr_region;
  logic [15:0] be_wr_addr;
  logic [127:0] be_wr_data;
  build_engine_v0 #(.HASH_AW(14), .NPT(PT_DEPTH), .NCELL(CELL_DEPTH)) u_be (
    .clk(clk75), .rst(rst), .start(be_start), .npoints(be_npts), .wbuf(be_wbuf),
    .w_valid(w_valid), .w_ready(w_ready), .w_data(pt_word), .pt_valid(1'b0), .pt_ready(pt_ready), .pt_data(96'd0),
    .busy(be_busy), .done_pulse(be_done), .status(be_status), .kept(be_kept), .ncells(be_cells),
    .culled(be_culled), .probes(be_probes), .cycles(be_cycles),
    .env_wr_en(be_wr_en), .env_wr_buf(be_wr_buf), .env_wr_region(be_wr_region), .env_wr_addr(be_wr_addr),
    .env_wr_data(be_wr_data));

  // ---------------- PLE + CSE ----------------
  logic [11:0] awaddr, araddr;
  logic        awvalid, awready, wvalid, wready, bvalid, bready, arvalid, arready, rvalid, rready, irq;
  logic [31:0] wdata, rdata;
  logic [1:0]  bresp, rresp;
  ple_cse_top #(.N_CU(N_CU)) u_pc (
    .clk(clk75), .rst_n(!rst),
    .s_axil_awaddr(awaddr), .s_axil_awvalid(awvalid), .s_axil_awready(awready), .s_axil_wdata(wdata),
    .s_axil_wstrb(4'hF), .s_axil_wvalid(wvalid), .s_axil_wready(wready), .s_axil_bresp(bresp),
    .s_axil_bvalid(bvalid), .s_axil_bready(bready), .s_axil_araddr(araddr), .s_axil_arvalid(arvalid),
    .s_axil_arready(arready), .s_axil_rdata(rdata), .s_axil_rresp(rresp), .s_axil_rvalid(rvalid),
    .s_axil_rready(rready), .irq(irq),
    .env_wr_en(be_wr_en), .env_wr_buf(be_wr_buf), .env_wr_region(env_region_e'(be_wr_region)),
    .env_wr_addr(be_wr_addr), .env_wr_data(be_wr_data));

  // ---------------- bus decode ----------------
  typedef enum logic [2:0] {S_IDLE, S_AXW, S_AXB, S_AXR, S_AXRD, S_PT, S_RD} bst_e;
  bst_e bs;
  logic aw_done, w_done, pt_pend;
  logic [23:0] a;
  wire  is_ple = (addr[23:12] == 12'h000);
  assign w_valid = pt_pend;

  always_ff @(posedge clk75) begin
    if (rst) begin
      bs <= S_IDLE; ack <= 0; be_start <= 0; be_npts <= 0; be_wbuf <= 0; pt_pend <= 0;
      awvalid <= 0; wvalid <= 0; bready <= 0; arvalid <= 0; rready <= 0;
    end else begin
      ack <= 0; be_start <= 0;
      if (pt_pend && w_ready) pt_pend <= 0;
      unique case (bs)
        S_IDLE: if (req) begin
          a <= addr;
          if (is_ple) begin
            if (we) begin awaddr <= addr[11:0]; wdata <= wd; awvalid <= 1; wvalid <= 1; aw_done <= 0; w_done <= 0; bs <= S_AXW; end
            else begin araddr <= addr[11:0]; arvalid <= 1; bs <= S_AXR; end
          end else if (we) begin
            unique case (addr[7:0])
              8'h00: begin be_start <= wd[0]; ack <= 1; end
              8'h08: begin be_npts <= wd; ack <= 1; end
              8'h0C: begin be_wbuf <= wd[0]; ack <= 1; end
              8'h3C: begin pt_word <= wd; pt_pend <= 1; bs <= S_PT; end
              default: ack <= 1;
            endcase
          end else bs <= S_RD;
        end
        S_PT: if (!pt_pend || w_ready) begin ack <= 1; bs <= S_IDLE; end
        S_AXW: begin
          if (awvalid && awready) begin awvalid <= 0; aw_done <= 1; end
          if (wvalid && wready) begin wvalid <= 0; w_done <= 1; end
          if ((aw_done || (awvalid && awready)) && (w_done || (wvalid && wready))) begin bready <= 1; bs <= S_AXB; end
        end
        S_AXB: if (bvalid) begin bready <= 0; ack <= 1; bs <= S_IDLE; end
        S_AXR: if (arready) begin arvalid <= 0; rready <= 1; bs <= S_AXRD; end
        S_AXRD: if (rvalid) begin rd <= rdata; rready <= 0; ack <= 1; bs <= S_IDLE; end
        S_RD: begin
          unique case (a[7:0])
            8'h04: rd <= {24'd0, be_status, 3'd0, be_busy};
            8'h08: rd <= be_npts;
            8'h0C: rd <= {31'd0, be_wbuf};
            8'h10: rd <= {16'd0, be_kept};
            8'h14: rd <= {19'd0, be_cells};
            8'h18: rd <= be_culled;
            8'h1C: rd <= be_cycles;
            8'h20: rd <= be_probes;
            8'hFC: rd <= {8'h5C, 8'hB0, 8'(N_CU), 8'd75};   // tag, "build v0", N_CU, MHz
            default: rd <= 32'd0;
          endcase
          ack <= 1; bs <= S_IDLE;
        end
        default: bs <= S_IDLE;
      endcase
    end
  end

  logic [25:0] hb;
  always_ff @(posedge clk75) hb <= hb + 1'b1;
  assign led = {hb[25], irq, u_pc.u_ple.busy, be_busy};
endmodule
