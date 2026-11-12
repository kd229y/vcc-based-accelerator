// System test through the vcc_core register bus: stream cloud -> build -> plan,
// then compare status, path and statistics with core.cpp Model::plan().
// plusargs: +cloud=<scene dir> +plan=<plan dir>
`timescale 1ns/1ps
module tb_plan;
  reg clk = 0, rst = 1;
  always #5 clk = ~clk;
  reg  [8*256-1:0] cdir, pdir;
  reg  [31:0] meta [0:13];
  reg  [31:0] win [0:200000];
  reg  [31:0] pin [0:13];
  reg  [31:0] pcfg [0:16];
  reg  [31:0] pmeta [0:7];
  reg  [31:0] epath [0:8191];

  reg         req = 0, we = 0;
  reg  [23:0] addr = 0;
  reg  [31:0] wdata = 0;
  wire        ack;
  wire [31:0] rdata;
  wire        irq, s_ready;
  vcc_core #(.HASH_AW(15), .CELL_AW(12), .MAXN_AW(10)) dut (
    .clk(clk), .rst(rst), .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wdata),
    .bus_ack(ack), .bus_rdata(rdata), .s_valid(1'b0), .s_ready(s_ready), .s_data(32'd0), .irq(irq));

  task wr(input [23:0] ad, input [31:0] d);
    begin
      @(negedge clk); req = 1; we = 1; addr = ad; wdata = d;
      @(negedge clk); req = 0; we = 0;
      while (!ack) @(negedge clk);
    end
  endtask
  reg [31:0] rv;
  task rd(input [23:0] ad);
    begin
      @(negedge clk); req = 1; we = 0; addr = ad;
      @(negedge clk); req = 0;
      while (!ack) @(negedge clk);
      rv = rdata;
    end
  endtask

  integer i, j, n, errors, plen;
  initial if ($test$plusargs("trace")) forever begin
    #200000;
    $display("[%0t] rrt st=%0d states=%0d iters=%0d nodes=%0d edges=%0d q=%0d | fk st=%0d q st=%0d build st=%0d",
      $time, dut.u_rrt.st, dut.u_rrt.st_states, dut.u_rrt.st_iters, dut.u_rrt.st_nodes, dut.u_rrt.st_edges,
      dut.u_rrt.st_queries, dut.u_fk.st, dut.u_q.st, dut.u_build.st);
  end
  task expect_eq(input [8*24-1:0] what, input [31:0] got, input [31:0] e);
    begin
      if (got !== e) begin errors = errors + 1; $display("MISMATCH %0s: got %0d expected %0d", what, got, e); end
      else $display("  ok %0s = %0d", what, got);
    end
  endtask

  initial begin
    errors = 0;
    if (!$value$plusargs("cloud=%s", cdir) || !$value$plusargs("plan=%s", pdir)) begin $display("need +cloud +plan"); $finish; end
    $readmemh({cdir, "/meta.hex"}, meta);
    n = meta[0];
    if (n > 0) $readmemh({cdir, "/points_in.hex"}, win);
    $readmemh({pdir, "/plan_in.hex"}, pin);
    $readmemh({pdir, "/plan_cfg.hex"}, pcfg);
    $readmemh({pdir, "/plan_meta.hex"}, pmeta);
    if (pmeta[1] > 0) $readmemh({pdir, "/exp_path.hex"}, epath);
    repeat (4) @(negedge clk); rst = 0;

    rd(24'h0FC); $display("ID = %h", rv);
    wr(24'h008, n);
    wr(24'h000, 32'h1);
    for (i = 0; i < 3 * n; i = i + 1) wr(24'h03C, win[i]);
    rd(24'h004); while (rv[0]) rd(24'h004);
    $display("build status=%0d", rv[7:4]);
    rd(24'h00C); $display("kept=%0d (golden %0d)", rv, meta[2]);

    wr(24'h080, pcfg[4]); wr(24'h084, pcfg[3]); wr(24'h088, pcfg[12]); wr(24'h08C, pcfg[11]);
    wr(24'h090, pcfg[9]); wr(24'h094, pcfg[16]); wr(24'h098, pcfg[6]);
    for (j = 0; j < 7; j = j + 1) begin wr(24'h040 + 4 * j, pin[j]); wr(24'h060 + 4 * j, pin[7 + j]); end
    wr(24'h000, 32'h8);
    rd(24'h004); while (rv[16]) begin repeat (2000) @(negedge clk); rd(24'h004); end
    $display("plan finished: status=%0d", rv[23:20]);
    expect_eq("status", rv[23:20], pmeta[0]);
    rd(24'h0A0); plen = rv; expect_eq("path_len", rv, pmeta[1]);
    rd(24'h0A4); expect_eq("iterations", rv, pmeta[2]);
    rd(24'h0A8); expect_eq("nodes", rv, pmeta[3]);
    rd(24'h0AC); expect_eq("state_calls", rv, pmeta[4]);
    rd(24'h0B0); expect_eq("edge_calls", rv, pmeta[5]);
    rd(24'h0B4); expect_eq("queries", rv, pmeta[6]);
    rd(24'h0B8); $display("  plan cycles = %0d (%0d us @100MHz)", rv, rv / 100);
    for (i = 0; i < plen && i < pmeta[1]; i = i + 1)
      for (j = 0; j < 7; j = j + 1) begin
        rd(24'h600000 + 32 * i + 4 * j);
        if (rv !== epath[7 * i + j]) begin
          errors = errors + 1;
          $display("MISMATCH path[%0d][%0d]: got %h expected %h", i, j, rv, epath[7 * i + j]);
        end
      end
    if (errors == 0) $display("PASS plan %0s", pdir); else $display("FAIL plan %0s (%0d)", pdir, errors);
    $finish;
  end
endmodule
