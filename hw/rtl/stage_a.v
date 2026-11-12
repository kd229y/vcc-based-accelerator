// Stage A: binary32 -> Q14 (truncate toward zero, spec 6.2.4), radial cull,
// workspace-relative u16 coordinates, fine Morton key (21 b) and S=4 selector
// distance d = sum (2*o+1-16)^2.  Bit-exact with soc_model/core.cpp build().
//
// 2-stage valid/ready pipeline, 1 point/cycle.
`timescale 1ns/1ps
module stage_a #(
  parameter [31:0] RANGE_SQ = 32'd380133009   // ceil(1.19 m * 2^14)^2 = 19497^2
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
  output reg         out_abort,   // non-finite coordinate -> INVALID_INPUT
  output reg         out_cull,    // radial / workspace cull (point silently dropped, as golden)
  output reg  [15:0] out_x,       // workspace-relative (world + 2 m) Q14, = signed ^ 0x8000
  output reg  [15:0] out_y,
  output reg  [15:0] out_z,
  output reg  [20:0] out_key,
  output reg  [9:0]  out_d,
  output reg  [15:0] out_hash     // (key * 2654435761) mod 2^16  (core.cpp hash, low bits)
);
  // ---------------- binary32 -> signed Q14 -----------------
  // Values with |x| >= 2 m (exp >= 128) can never survive the 1.19 m radial cull,
  // so they are flagged "big" instead of being converted (same outcome as golden).
  function [17:0] f2q;  // {nonfinite, big, signed[15:0]}
    input [31:0] u;
    reg [7:0]  e;
    reg [23:0] mant;
    reg [8:0]  sh;
    reg [23:0] mag;
    reg [15:0] v;
    begin
      e    = u[30:23];
      mant = {|e, u[22:0]};
      sh   = 9'd136 - {1'b0, e};
      mag  = (e >= 8'd128 || sh >= 9'd24) ? 24'd0 : (mant >> sh[4:0]);
      v    = u[31] ? (16'd0 - mag[15:0]) : mag[15:0];
      f2q  = {&e, (e >= 8'd128), v};
    end
  endfunction

  wire s1_adv;
  reg        s1_valid;
  // dont_touch: keep these in fabric so the fp32 shifter does not have to meet DSP A/B setup
  (* dont_touch = "true" *) reg [17:0] s1_x, s1_y, s1_z;
  wire s3_adv = !out_valid || out_ready;
  reg  m_valid;
  wire s2_adv = !m_valid || s3_adv;
  assign s1_adv  = !s1_valid || s2_adv;
  assign in_ready = s1_adv;

  always @(posedge clk) begin
    if (rst) s1_valid <= 1'b0;
    else if (s1_adv) begin
      s1_valid <= in_valid;
      s1_x <= f2q(in_x);
      s1_y <= f2q(in_y);
      s1_z <= f2q(in_z);
    end
  end

  // ---------------- cull, key, selector distance -----------------
  wire signed [15:0] vx = s1_x[15:0], vy = s1_y[15:0], vz = s1_z[15:0];
  wire [31:0] sqx = vx * vx;
  wire [31:0] sqy = vy * vy;
  wire [31:0] sqz = vz * vz;
  wire big = s1_x[16] | s1_y[16] | s1_z[16];
  wire nonfinite = s1_x[17] | s1_y[17] | s1_z[17];
  wire [15:0] rx = s1_x[15:0] ^ 16'h8000;
  wire [15:0] ry = s1_y[15:0] ^ 16'h8000;
  wire [15:0] rz = s1_z[15:0] ^ 16'h8000;

  function [20:0] morton7;
    input [6:0] x, y, z;
    integer b;
    begin
      for (b = 0; b < 7; b = b + 1) begin
        morton7[3*b]   = x[b];
        morton7[3*b+1] = y[b];
        morton7[3*b+2] = z[b];
      end
    end
  endfunction

  function [7:0] esq;  // (2*o + 1 - 16)^2, o in 0..15
    input [3:0] o;
    reg signed [5:0] e;
    begin
      e = $signed({1'b0, o, 1'b1}) - 6'sd16;
      esq = e * e;
    end
  endfunction

  // stage 2: squares, key, d (registered)
  reg [31:0] m_sqx, m_sqy, m_sqz;
  reg        m_big, m_nonfinite;
  reg [15:0] m_x, m_y, m_z;
  reg [20:0] m_key;
  reg [9:0]  m_d;
  always @(posedge clk) begin
    if (rst) m_valid <= 1'b0;
    else if (s2_adv) begin
      m_valid <= s1_valid;
      m_sqx <= sqx; m_sqy <= sqy; m_sqz <= sqz;
      m_big <= big; m_nonfinite <= nonfinite;
      m_x <= rx; m_y <= ry; m_z <= rz;
      m_key <= morton7(rx[15:9], ry[15:9], rz[15:9]);
      m_d <= esq(rx[8:5]) + esq(ry[8:5]) + esq(rz[8:5]);
    end
  end
  // stage 3: radial cull compare
  wire [33:0] m_norm = m_sqx + m_sqy + m_sqz;
  always @(posedge clk) begin
    if (rst) out_valid <= 1'b0;
    else if (s3_adv) begin
      out_valid <= m_valid;
      out_abort <= m_nonfinite;
      out_cull  <= m_big || (m_norm >= {2'b00, RANGE_SQ});
      out_x <= m_x; out_y <= m_y; out_z <= m_z;
      out_key <= m_key;
      out_hash <= m_key[15:0] * 16'h79B1;   // low 16 bits of key*0x9E3779B1
      out_d <= m_d;
    end
  end
endmodule
