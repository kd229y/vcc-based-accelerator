// vcc_v0_uart_top through the UART byte protocol (fast baud): build-engine registers + point stream,
// and the PLE AXI-Lite path (reset values, write/read-back). Compile with -DSIM_NO_MMCM.
module tb_v0_uart;
  localparam int CPB = 8;
  logic clk = 0; always #5 clk = ~clk;
  logic rst_btn = 1, rx = 1;
  wire tx; wire [3:0] led;
  vcc_v0_uart_top #(.N_CU(1), .CLKS_PER_BIT(CPB), .IDLE_BITS(12)) dut (.clk, .rst_btn, .uart_rx(rx), .uart_tx(tx), .led);

  task automatic sendb(input logic [7:0] b);
    rx = 0; repeat (CPB) @(posedge clk);
    for (int k = 0; k < 8; k++) begin rx = b[k]; repeat (CPB) @(posedge clk); end
    rx = 1; repeat (CPB) @(posedge clk);
  endtask
  task automatic send32(input logic [31:0] w); for (int k = 0; k < 4; k++) sendb(w[8*k +: 8]); endtask
  task automatic recvb(output logic [7:0] b);
    @(negedge tx); repeat (CPB + CPB / 2) @(posedge clk);
    for (int k = 0; k < 8; k++) begin b[k] = tx; repeat (CPB) @(posedge clk); end
  endtask
  task automatic recv32(output logic [31:0] w); logic [7:0] b; for (int k = 0; k < 4; k++) begin recvb(b); w[8*k +: 8] = b; end endtask
  task automatic wr(input logic [23:0] a, input logic [31:0] d);
    logic [7:0] k; sendb("W"); send32({8'd0, a}); send32(d); recvb(k);
    if (k != "K") begin $display("ERROR no K after W %h", a); errs++; end
  endtask
  task automatic rd(input logic [23:0] a, output logic [31:0] d);
    fork begin sendb("R"); send32({8'd0, a}); end recv32(d); join
  endtask

  int errs = 0;
  logic [31:0] win [0:20000];
  logic [31:0] meta [0:3];
  task automatic check(input string what, input longint got, input longint e);
    if (got != e) begin $display("ERROR %s got %0h expected %0h", what, got, e); errs++; end
    else $display("  ok %s = %0h", what, got);
  endtask

  initial begin
    logic [31:0] v; logic [7:0] k; int n;
    $readmemh("hw/vectors_v0/edge/meta.hex", meta);
    n = meta[0];
    $readmemh("hw/vectors_v0/edge/points_in.hex", win, 0, 3 * n - 1);
    repeat (20) @(posedge clk); rst_btn = 0; repeat (20) @(posedge clk);
    rd(24'h0100FC, v); check("ID", v[31:16], 16'h5CB0);
    // PLE AXI-Lite: reset values and read-back
    rd(24'h000010, v); check("PLE MODE reset", v, 32'hC);
    rd(24'h000014, v); check("PLE range reset", v, 32'd1 << 24);
    rd(24'h000030, v); check("PLE W_MIN x", v, 32'hFFFFB3D7);
    wr(24'h000028, 32'd1234); rd(24'h000028, v); check("PLE max_iter write/read", v, 1234);
    wr(24'h000074, 32'hDEADBEEF); rd(24'h000074, v); check("PLE SMP_SHIFT[5]", v, 32'hDEADBEEF);
    // build engine over UART
    wr(24'h010008, n); wr(24'h01000C, 1); wr(24'h010000, 1);
    sendb("P"); send32(24'h01003C); send32(3 * n);
    for (int i = 0; i < 3 * n; i++) send32(win[i]);
    recvb(k); if (k != "K") begin $display("ERROR no K after P"); errs++; end
    do rd(24'h010004, v); while (v[0]);
    check("build status", v[7:4], 0);
    rd(24'h010010, v); check("kept", v, meta[2]);
    rd(24'h010014, v); check("cells", v, meta[3]);
    // environment really landed in buffer 1 of the CSE memory (header word 1 = n_pts)
    check("env buf1 n_pts", dut.u_pc.u_env.g_buf[1].hdr[1], meta[2]);
    check("env buf1 valid", dut.u_pc.u_env.g_buf[1].hdr[0], 1);
    $display("TB_V0_UART %s (%0d errors)", errs == 0 ? "PASS" : "FAIL", errs);
    $finish;
  end
endmodule
