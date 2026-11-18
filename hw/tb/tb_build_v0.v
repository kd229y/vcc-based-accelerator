// build_engine_v0 vs PLE-CSE fx_env_build (format v0), field by field.
// plusargs: +dir=<vector dir> [+dir2=<second scene, run right after on the other buffer>] [+words]
`timescale 1ns/1ps
module tb_build_v0;
  reg clk = 0, rst = 1;
  always #5 clk = ~clk;
  reg  [8*256-1:0] dir, dir2;
  reg  [31:0] win [0:700000];
  reg  [31:0] meta [0:3];
  reg  [31:0] ehdr [0:8];
  reg  [15:0] edir [0:7999];
  reg  [127:0] ecell [0:1023];
  reg  [15:0] ex [0:16383], ey [0:16383], ez [0:16383];
  // captured environment (per buffer)
  reg  [31:0] hdr [0:1][0:8];
  reg  [15:0] dirm [0:1][0:7999];
  reg  [127:0] cellm [0:1][0:1023];
  reg  [47:0] ptm [0:1][0:16383];

  reg         start = 0, wbuf = 0, w_valid = 0, pt_valid = 0;
  reg  [31:0] npoints = 0, w_data = 0;
  reg  [95:0] pt_data = 0;
  wire        w_ready, pt_ready, busy, done;
  wire [3:0]  status; wire [15:0] kept; wire [12:0] ncells; wire [31:0] culled, probes, cycles;
  wire        we, wb; wire [1:0] wr; wire [15:0] wa; wire [127:0] wd;
  build_engine_v0 dut (.clk(clk), .rst(rst), .start(start), .npoints(npoints), .wbuf(wbuf),
    .w_valid(w_valid), .w_ready(w_ready), .w_data(w_data), .pt_valid(pt_valid), .pt_ready(pt_ready), .pt_data(pt_data),
    .busy(busy), .done_pulse(done), .status(status), .kept(kept), .ncells(ncells), .culled(culled), .probes(probes),
    .cycles(cycles), .env_wr_en(we), .env_wr_buf(wb), .env_wr_region(wr), .env_wr_addr(wa), .env_wr_data(wd));

  integer errors = 0, wrong_addr = 0;
  always @(posedge clk) if (we) begin
    case (wr)
      2'd0: if (wa < 9) hdr[wb][wa] <= wd[31:0]; else wrong_addr = wrong_addr + 1;
      2'd1: if (wa < 8000) dirm[wb][wa] <= wd[15:0]; else wrong_addr = wrong_addr + 1;
      2'd2: if (wa < 1024) cellm[wb][wa] <= wd; else wrong_addr = wrong_addr + 1;
      default: if (wa < 16384) ptm[wb][wa] <= wd[47:0]; else wrong_addr = wrong_addr + 1;
    endcase
  end

  integer n, widx, words, seed = 3;
  reg hs = 0, phs = 0;
  always @(posedge clk) begin hs <= w_valid && w_ready; phs <= pt_valid && pt_ready; end
  always @(negedge clk) begin
    if (hs) widx = widx + 1;
    if (phs) widx = widx + 3;
    if (words) begin
      pt_valid <= 0;
      if (widx < 3 * n && ($random(seed) & 3) != 0) begin w_valid <= 1; w_data <= win[widx]; end else w_valid <= 0;
    end else begin
      w_valid <= 0;
      if (widx < 3 * n) begin pt_valid <= 1; pt_data <= {win[widx + 2], win[widx + 1], win[widx]}; end
      else pt_valid <= 0;
    end
  end

  task chk(input [8*16-1:0] what, input integer idx, input [127:0] got, input [127:0] expv);
    if (got !== expv) begin
      errors = errors + 1;
      if (errors < 25) $display("MISMATCH %0s[%0d]: got %h expected %h", what, idx, got, expv);
    end
  endtask

  integer i, b;
  // the engine keeps 8-bit AABB offsets: golden lo & ~7, hi | 7 (box grows, answers unchanged)
  function [127:0] round_box; input [127:0] c; integer a;
    begin
      round_box = c;
      for (a = 0; a < 3; a = a + 1) begin
        round_box[26 + 11*a +: 3] = 3'b000;
        round_box[59 + 11*a +: 3] = 3'b111;
      end
    end
  endfunction
  task run(input [8*256-1:0] d, input bsel);
    begin
      $readmemh({d, "/meta.hex"}, meta);
      n = meta[0];
      if (n > 0) $readmemh({d, "/points_in.hex"}, win);
      $readmemh({d, "/env_hdr.hex"}, ehdr);
      $readmemh({d, "/env_dir.hex"}, edir);
      if (meta[3] > 0) $readmemh({d, "/env_cell.hex"}, ecell, 0, meta[3] - 1);
      if (meta[2] > 0) begin
        $readmemh({d, "/env_ptx.hex"}, ex, 0, meta[2] - 1);
        $readmemh({d, "/env_pty.hex"}, ey, 0, meta[2] - 1);
        $readmemh({d, "/env_ptz.hex"}, ez, 0, meta[2] - 1);
      end
      widx = 0;
      @(negedge clk); npoints = n; wbuf = bsel; start = 1; @(negedge clk); start = 0;
      wait (done); @(negedge clk);
      $display("%0s -> buf%0d: status=%0d kept=%0d (fx %0d) cells=%0d (fx %0d) culled=%0d probes=%0d cycles=%0d",
               d, bsel, status, kept, meta[2], ncells, meta[3], culled, probes, cycles);
      for (i = 0; i < 9; i = i + 1) chk("hdr", i, hdr[bsel][i], ehdr[i]);
      for (i = 0; i < 8000; i = i + 1) chk("dir", i, dirm[bsel][i], edir[i]);
      for (i = 0; i < meta[3]; i = i + 1) chk("cell", i, cellm[bsel][i], round_box(ecell[i]));
      for (i = 0; i < meta[2]; i = i + 1) chk("pts", i, ptm[bsel][i], {15'd0, ez[i][10:0], ey[i][10:0], ex[i][10:0]});
    end
  endtask

  initial begin
    words = $test$plusargs("words");
    if (!$value$plusargs("dir=%s", dir)) begin $display("need +dir"); $finish; end
    for (b = 0; b < 2; b = b + 1) for (i = 0; i < 8000; i = i + 1) dirm[b][i] = 16'hDEAD;  // must be overwritten
    repeat (4) @(negedge clk); rst = 0;
    run(dir, 1'b0);
    if ($value$plusargs("dir2=%s", dir2)) run(dir2, 1'b1);
    if (wrong_addr) begin errors = errors + 1; $display("%0d out-of-range writes", wrong_addr); end
    if (errors == 0) $display("PASS build_v0"); else $display("FAIL build_v0 (%0d)", errors);
    $finish;
  end
endmodule
