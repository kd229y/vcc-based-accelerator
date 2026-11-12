// End-to-end test of vcc_uart_top through the serial protocol (fast baud for sim).
// Builds scene s2_edge, checks kept / points / queries / one FK against golden.
`timescale 1ns/1ps
module tb_uart;
  localparam CPB = 8;
  reg clk = 0; always #5 clk = ~clk;
  reg rst_btn = 1, rx = 1;
  wire tx; wire [3:0] led;
  vcc_uart_top #(.CLK_HZ(CPB), .BAUD(1), .IDLE_BITS(12), .HASH_AW(15), .CELL_AW(12), .MAXN_AW(9)) dut (
    .clk(clk), .rst_btn(rst_btn), .uart_rx(rx), .uart_tx(tx), .led(led));

  reg [31:0] meta [0:13]; reg [31:0] win [0:20000]; reg [47:0] exp_p [0:32767];
  reg [63:0] qv [0:4095]; reg [0:0] qe [0:4095];
  reg [31:0] fkin [0:7*200-1]; reg [47:0] fkout [0:59*200-1];

  task sendb(input [7:0] b); integer k;
    begin
      rx = 0; repeat (CPB) @(posedge clk);
      for (k = 0; k < 8; k = k + 1) begin rx = b[k]; repeat (CPB) @(posedge clk); end
      rx = 1; repeat (CPB) @(posedge clk);
    end
  endtask
  task send32(input [31:0] w); begin sendb(w[7:0]); sendb(w[15:8]); sendb(w[23:16]); sendb(w[31:24]); end endtask
  task recvb(output [7:0] b); integer k;
    begin
      @(negedge tx); repeat (CPB + CPB / 2) @(posedge clk);
      for (k = 0; k < 8; k = k + 1) begin b[k] = tx; repeat (CPB) @(posedge clk); end
    end
  endtask
  reg [7:0] b0, b1, b2, b3; reg [31:0] rv;
  task recv32; begin recvb(b0); recvb(b1); recvb(b2); recvb(b3); rv = {b3, b2, b1, b0}; end endtask
  task wr(input [23:0] a, input [31:0] d); begin sendb("W"); send32(a); send32(d); recvb(b0); if (b0 != "K") $display("no K"); end endtask
  task rd(input [23:0] a); begin fork begin sendb("R"); send32(a); end recv32; join end endtask

  integer i, n, errors;
  reg [15:0] qx, qy, qz, qr;
  initial begin
    errors = 0;
    $readmemh("vectors/s2_edge/meta.hex", meta); n = meta[0];
    $readmemh("vectors/s2_edge/points_in.hex", win);
    $readmemh("vectors/s2_edge/exp_points.hex", exp_p);
    $readmemh("vectors/s2_edge/queries.hex", qv);
    $readmemh("vectors/s2_edge/exp_query.hex", qe);
    $readmemh("vectors/fk/fk_in.hex", fkin);
    $readmemh("vectors/fk/fk_out.hex", fkout);
    repeat (10) @(posedge clk); rst_btn = 0; repeat (10) @(posedge clk);
    // simulated cable glitch: half a write command, then silence
    sendb("W"); sendb(8'h08); sendb(8'h00);
    repeat (6000) @(posedge clk);
    rd(24'h0FC); $display("ID %h (after aborted command)", rv);
    if (rv[31:24] != 8'h5C) begin errors = errors + 1; $display("bridge did not resync"); end
    wr(24'h008, n);
    wr(24'h000, 1);
    // point stream with one 'P' command (FIFO absorbs the table clear)
    sendb("P"); send32(24'h03C); send32(3 * n);
    for (i = 0; i < 3 * n; i = i + 1) send32(win[i]);
    recvb(b0); if (b0 != "K") begin $display("no K after P"); errors = errors + 1; end
    rd(24'h004); while (rv[0]) rd(24'h004);
    if (rv[7:4] != meta[1]) begin errors = errors + 1; $display("status %0d", rv[7:4]); end
    rd(24'h00C); if (rv != meta[2]) begin errors = errors + 1; $display("kept %0d vs %0d", rv, meta[2]); end
    // burst read of the first 8 points (16 words)
    fork begin sendb("M"); send32(24'h100000); send32(16); end
      for (i = 0; i < 8; i = i + 1) begin
        recv32; if (rv != exp_p[i][31:0]) begin errors = errors + 1; $display("p%0d xy %h vs %h", i, rv, exp_p[i][31:0]); end
        recv32; if (rv[15:0] != exp_p[i][47:32]) begin errors = errors + 1; $display("p%0d z", i); end
      end
    join
    for (i = 0; i < 40; i = i + 1) begin
      {qr, qz, qy, qx} = qv[i];
      wr(24'h030, {qy, qx}); wr(24'h034, {qr, qz});
      rd(24'h038); while (rv[1]) rd(24'h038);
      if (rv[0] != qe[i]) begin errors = errors + 1; $display("query %0d", i); end
    end
    for (i = 0; i < 7; i = i + 1) wr(24'h040 + 4 * i, fkin[7 * 5 + i]);
    wr(24'h000, 4);
    rd(24'h004); while (rv[12]) rd(24'h004);
    for (i = 0; i < 59; i = i + 7) begin
      rd(24'h500000 + 8 * i); if (rv != fkout[59 * 5 + i][31:0]) begin errors = errors + 1; $display("fk %0d", i); end
    end
    if (errors == 0) $display("PASS uart end-to-end"); else $display("FAIL uart (%0d)", errors);
    $finish;
  end
endmodule
