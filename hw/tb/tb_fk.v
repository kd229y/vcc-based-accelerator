// Bit-exact check of fk_engine against integer_fk() golden vectors.  plusargs: +dir= +n=
`timescale 1ns/1ps
module tb_fk;
  reg clk = 0, rst = 1;
  always #5 clk = ~clk;
  reg [8*256-1:0] dir;
  reg [31:0] qin [0:7*512-1];
  reg [47:0] qout [0:59*512-1];
  reg [223:0] q;
  reg start = 0;
  wire busy, done, owe; wire [5:0] oidx; wire [47:0] oxyz; wire [31:0] cyc;
  fk_engine dut(.clk(clk), .rst(rst), .start(start), .q(q), .busy(busy), .done_pulse(done), .cycles(cyc),
                .out_we(owe), .out_idx(oidx), .out_xyz(oxyz));
  integer c, n, j, errors, outs;
  always @(posedge clk) if (owe) begin
    outs = outs + 1;
    if (oxyz !== qout[c*59 + oidx]) begin
      errors = errors + 1;
      if (errors < 20) $display("MISMATCH case %0d sphere %0d: got %h expected %h", c, oidx, oxyz, qout[c*59+oidx]);
    end
  end
  initial begin
    errors = 0; outs = 0;
    if (!$value$plusargs("dir=%s", dir)) begin $display("need +dir"); $finish; end
    if (!$value$plusargs("n=%d", n)) n = 128;
    $readmemh({dir, "/fk_in.hex"}, qin);
    $readmemh({dir, "/fk_out.hex"}, qout);
    repeat (3) @(negedge clk); rst = 0;
    for (c = 0; c < n; c = c + 1) begin
      for (j = 0; j < 7; j = j + 1) q[32*j +: 32] = qin[c*7 + j];
      @(negedge clk); start = 1; @(negedge clk); start = 0;
      wait (done); @(negedge clk);
    end
    $display("fk: %0d cases, %0d sphere outputs, %0d cycles/FK", n, outs, cyc);
    if (errors == 0 && outs == 59*n) $display("PASS %0s", dir); else $display("FAIL %0s (%0d mismatches)", dir, errors);
    $finish;
  end
endmodule
