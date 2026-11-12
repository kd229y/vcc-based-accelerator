// Board-level wrappers.
//   vcc_uart_top : any FPGA board with a USB-UART (Arty / Nexys / Basys / Genesys ...)
//   vcc_axil_top : Zynq-7000 / Zynq UltraScale+ (PS drives AXI4-Lite; optional AXI-Stream points from AXI DMA)
`timescale 1ns/1ps
module vcc_uart_top #(
  parameter CLK_HZ  = 100_000_000,
  parameter BAUD    = 921_600,
  parameter IDLE_BITS = 23,          // ~84 ms @100 MHz: drop half-received commands
  parameter HASH_AW = 14,
  parameter CELL_AW = 12,
  parameter MAXN_AW = 9,
  parameter RAM_STYLE = "block"
) (
  input  wire clk,
  input  wire rst_btn,       // active-high push button (tie low if unused)
  input  wire uart_rx,
  output wire uart_tx,
  output wire [3:0] led
);
  reg [3:0] rs = 4'hF;
  always @(posedge clk) rs <= {rs[2:0], rst_btn};
  wire rst = rs[3];
  wire req, we, ack, irq, s_ready; wire [23:0] addr; wire [31:0] wd, rd;
  uart_bridge #(.CLKS_PER_BIT(CLK_HZ / BAUD), .IDLE_BITS(IDLE_BITS)) u_br (
    .clk(clk), .rst(rst), .rx(uart_rx), .tx(uart_tx),
    .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wd), .bus_ack(ack), .bus_rdata(rd));
  vcc_core #(.HASH_AW(HASH_AW), .CELL_AW(CELL_AW), .MAXN_AW(MAXN_AW), .RAM_STYLE(RAM_STYLE)) u_core (
    .clk(clk), .rst(rst), .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wd),
    .bus_ack(ack), .bus_rdata(rd), .s_valid(1'b0), .s_ready(s_ready), .s_data(128'd0), .irq(irq));
  reg [25:0] hb;
  always @(posedge clk) hb <= hb + 1'b1;
  assign led = {hb[25], ~uart_rx, u_core.b_busy, u_core.p_busy};
endmodule

module vcc_axil_top #(
  parameter HASH_AW = 15,
  parameter CELL_AW = 12,
  parameter MAXN_AW = 10,
  parameter RAM_STYLE = "block"
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:s_axis, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000" *)
  input  wire        aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        aresetn,
  // AXI4-Lite slave (24-bit byte address space)
  input  wire [23:0] s_axi_awaddr,
  input  wire        s_axi_awvalid,
  output reg         s_axi_awready,
  input  wire [31:0] s_axi_wdata,
  input  wire [3:0]  s_axi_wstrb,
  input  wire        s_axi_wvalid,
  output reg         s_axi_wready,
  output reg  [1:0]  s_axi_bresp,
  output reg         s_axi_bvalid,
  input  wire        s_axi_bready,
  input  wire [23:0] s_axi_araddr,
  input  wire        s_axi_arvalid,
  output reg         s_axi_arready,
  output reg  [31:0] s_axi_rdata,
  output reg  [1:0]  s_axi_rresp,
  output reg         s_axi_rvalid,
  input  wire        s_axi_rready,
  // AXI-Stream slave: points from AXI DMA MM2S (128-bit stream)
  input  wire [127:0] s_axis_tdata,     // one point per beat: x,y,z binary32 + 32-bit pad
  input  wire        s_axis_tvalid,
  output wire        s_axis_tready,
  (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq INTERRUPT" *)
  (* X_INTERFACE_PARAMETER = "SENSITIVITY LEVEL_HIGH" *)
  output wire        irq
);
  wire rst = !aresetn;
  reg  req, we; reg [23:0] addr; reg [31:0] wd;
  wire ack; wire [31:0] rd;
  vcc_core #(.HASH_AW(HASH_AW), .CELL_AW(CELL_AW), .MAXN_AW(MAXN_AW), .RAM_STYLE(RAM_STYLE)) u_core (
    .clk(aclk), .rst(rst), .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wd),
    .bus_ack(ack), .bus_rdata(rd), .s_valid(s_axis_tvalid), .s_ready(s_axis_tready), .s_data(s_axis_tdata), .irq(irq));

  localparam [2:0] A_IDLE = 0, A_WR = 1, A_WRESP = 2, A_RD = 3, A_RRESP = 4;
  reg [2:0] as;
  reg aw_ok, w_ok; reg [23:0] aw_a; reg [31:0] w_d;
  always @(posedge aclk) begin
    req <= 1'b0;
    if (rst) begin
      as <= A_IDLE; s_axi_awready <= 0; s_axi_wready <= 0; s_axi_bvalid <= 0; s_axi_arready <= 0; s_axi_rvalid <= 0;
      s_axi_bresp <= 0; s_axi_rresp <= 0; aw_ok <= 0; w_ok <= 0;
    end else begin
      s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_arready <= 1'b0;
      case (as)
        A_IDLE: begin
          if (s_axi_awvalid && !aw_ok && !s_axi_awready) begin s_axi_awready <= 1'b1; aw_a <= s_axi_awaddr; aw_ok <= 1'b1; end
          if (s_axi_wvalid && !w_ok && !s_axi_wready) begin s_axi_wready <= 1'b1; w_d <= s_axi_wdata; w_ok <= 1'b1; end
          if (aw_ok && w_ok) begin req <= 1'b1; we <= 1'b1; addr <= aw_a; wd <= w_d; aw_ok <= 0; w_ok <= 0; as <= A_WR; end
          else if (!aw_ok && !w_ok && !s_axi_awvalid && s_axi_arvalid) begin
            s_axi_arready <= 1'b1; req <= 1'b1; we <= 1'b0; addr <= s_axi_araddr; as <= A_RD;
          end
        end
        A_WR: if (ack) begin s_axi_bvalid <= 1'b1; as <= A_WRESP; end
        A_WRESP: if (s_axi_bready) begin s_axi_bvalid <= 1'b0; as <= A_IDLE; end
        A_RD: if (ack) begin s_axi_rdata <= rd; s_axi_rvalid <= 1'b1; as <= A_RRESP; end
        A_RRESP: if (s_axi_rready) begin s_axi_rvalid <= 1'b0; as <= A_IDLE; end
        default: as <= A_IDLE;
      endcase
    end
  end
endmodule
