// Stage A for environment format v0 (PLE-CSE, sw_rrtc/fx/fx_env.c::fx_env_build):
//   q = floor(p * 2^14) - w_min,  w_min = -19497 (floor(-1.19 m * 2^14)); keep 0 <= q < -2*w_min per axis
//   fine cell = q >> 9, query cell = q >> 11 (20 per axis), S=4 selector distance from q[8:5]
// binary32 decode + floor is exact (power-of-two scale). Non-finite input -> abort (fx leaves it undefined).
// 3-stage valid/ready pipeline, 1 point/cycle.
`timescale 1ns/1ps
module stage_a_v0 #(
  parameter signed [17:0] W_MIN = -18'sd19497
) (
  input  wire        clk,
  input  wire        rst,
  input  wire        in_valid,
  output wire        in_ready,
  input  wire [31:0] in_x,
  input  wire [31:0] in_y,
  input  wire [31:0] in_z,
  output reg         out_valid,
  input  wire        out_ready,
  output reg         out_abort,
  output reg         out_cull,
  output reg  [47:0] out_q,        // {z,y,x} 16-bit workspace coordinates (origin w_min)
  output reg  [9:0]  out_d,        // selector distance
  output reg  [15:0] out_hash,     // hash index (any function; only capacity depends on it)
  output reg  [12:0] out_qidx,     // query-cell DIR index (qz*20 + qy)*20 + qx
  output reg  [5:0]  out_low6      // fine cell inside the query cell, Morton {z1,y1,x1,z0,y0,x0}
);
  localparam signed [18:0] QMAX = -2 * W_MIN;     // 38994

  // ---- stage 1: binary32 -> floor(x * 2^14) as signed 17-bit (|x| < 2 m), flags ----
  function [18:0] f2floor;   // {nonfinite, big, signed[16:0]}
    input [31:0] u;
    reg [7:0]  e;
    reg [23:0] mant;
    reg [8:0]  sh;
    reg [23:0] mag;
    reg        frac;
    reg [16:0] v;
    begin
      e    = u[30:23];
      mant = {|e, u[22:0]};
      sh   = 9'd136 - {1'b0, e};
      if (e >= 8'd128) begin mag = 24'd0; frac = 1'b0; end
      else if (sh >= 9'd24) begin mag = 24'd0; frac = (mant != 24'd0); end
      else begin
        mag  = mant >> sh[4:0];
        frac = |(mant & ((24'd1 << sh[4:0]) - 24'd1));
      end
      v = u[31] ? (17'd0 - ({1'b0, mag[15:0]} + frac)) : {1'b0, mag[15:0]};
      f2floor = {&e, (e >= 8'd128), v};
    end
  endfunction

  wire s3_adv = !out_valid || out_ready;
  reg  s2_valid, s1_valid;
  wire s2_adv = !s2_valid || s3_adv;
  wire s1_adv = !s1_valid || s2_adv;
  assign in_ready = s1_adv;

  (* dont_touch = "true" *) reg [18:0] s1_x, s1_y, s1_z;
  always @(posedge clk) begin
    if (rst) s1_valid <= 1'b0;
    else if (s1_adv) begin
      s1_valid <= in_valid;
      s1_x <= f2floor(in_x); s1_y <= f2floor(in_y); s1_z <= f2floor(in_z);
    end
  end

  // ---- stage 2: q = v - w_min, cube cull, cell coordinates, selector distance ----
  wire signed [18:0] qx = $signed({{2{s1_x[16]}}, s1_x[16:0]}) - W_MIN;
  wire signed [18:0] qy = $signed({{2{s1_y[16]}}, s1_y[16:0]}) - W_MIN;
  wire signed [18:0] qz = $signed({{2{s1_z[16]}}, s1_z[16:0]}) - W_MIN;
  function inrange; input signed [18:0] q; inrange = (q >= 0) && (q < QMAX); endfunction
  function [7:0] esq; input [3:0] o; reg signed [5:0] t;
    begin t = $signed({1'b0, o, 1'b1}) - 6'sd16; esq = t * t; end
  endfunction

  reg        s2_abort, s2_cull;
  reg [15:0] s2_qx, s2_qy, s2_qz;
  reg [9:0]  s2_d;
  always @(posedge clk) begin
    if (rst) s2_valid <= 1'b0;
    else if (s2_adv) begin
      s2_valid <= s1_valid;
      s2_abort <= s1_x[18] | s1_y[18] | s1_z[18];
      s2_cull  <= s1_x[17] | s1_y[17] | s1_z[17] | !inrange(qx) | !inrange(qy) | !inrange(qz);
      s2_qx <= qx[15:0]; s2_qy <= qy[15:0]; s2_qz <= qz[15:0];
      s2_d  <= esq(qx[8:5]) + esq(qy[8:5]) + esq(qz[8:5]);
    end
  end

  // ---- stage 3: hash, DIR index, in-cell position ----
  function [20:0] morton7; input [6:0] x, y, z; integer b;
    begin for (b = 0; b < 7; b = b + 1) begin
      morton7[3*b] = x[b]; morton7[3*b+1] = y[b]; morton7[3*b+2] = z[b]; end end
  endfunction
  wire [20:0] fkey = morton7(s2_qx[15:9], s2_qy[15:9], s2_qz[15:9]);
  wire [4:0]  cx = s2_qx[15:11], cy = s2_qy[15:11], cz = s2_qz[15:11];
  always @(posedge clk) begin
    if (rst) out_valid <= 1'b0;
    else if (s3_adv) begin
      out_valid <= s2_valid;
      out_abort <= s2_abort;
      out_cull  <= s2_cull;
      out_q     <= {s2_qz, s2_qy, s2_qx};
      out_d     <= s2_d;
      out_hash  <= fkey[15:0] * 16'h79B1;
      out_qidx  <= ({8'd0, cz} * 13'd400) + ({8'd0, cy} * 13'd20) + {8'd0, cx};
      out_low6  <= {s2_qz[10], s2_qy[10], s2_qx[10], s2_qz[9], s2_qy[9], s2_qx[9]};
    end
  end
endmodule
