// Fixed-point forward kinematics (spec 6.5.4), bit-exact with soc_model/core.cpp integer_fk().
//
// v2 (performance):
//   * 7 integer CORDICs run in parallel (~31 cycles for all revolute joints)
//   * 3-lane MAC pipeline: one transform element (3 products + round + translation) issued per cycle
//   * joint constants and the turn matrix feed the multipliers directly (no copies into the register file)
//   ~580 cycles per FK (v1: 3,730).
// Pipeline: S0 operand fetch -> S1 mult -> S2 mult reg -> S3 3-way sum -> S4 round_div(.,2^30) -> S5 write/emit
`timescale 1ns/1ps
module cordic_unit (
  input  wire               clk,
  input  wire               start,
  input  wire signed [31:0] a,          // Q24 angle
  output reg                done,
  output reg  signed [35:0] s,
  output reg  signed [35:0] c
);
  localparam signed [35:0] PI28 = 36'sd843314857, HALF = 36'sd421657428, K0 = 36'sd652032874;
  reg signed [35:0] z, x, y;
  reg               neg, busy = 1'b0;
  reg [4:0]         it = 5'd31;
  reg signed [35:0] at;
  always @* begin
    case (it)
      5'd0: at = 36'sd210828714;  5'd1: at = 36'sd124459457;  5'd2: at = 36'sd65760959;   5'd3: at = 36'sd33381290;
      5'd4: at = 36'sd16755422;   5'd5: at = 36'sd8385879;    5'd6: at = 36'sd4193963;    5'd7: at = 36'sd2097109;
      5'd8: at = 36'sd1048571;    5'd9: at = 36'sd524287;     5'd10: at = 36'sd262144;    5'd11: at = 36'sd131072;
      5'd12: at = 36'sd65536;     5'd13: at = 36'sd32768;     5'd14: at = 36'sd16384;     5'd15: at = 36'sd8192;
      5'd16: at = 36'sd4096;      5'd17: at = 36'sd2048;      5'd18: at = 36'sd1024;      5'd19: at = 36'sd512;
      5'd20: at = 36'sd256;       5'd21: at = 36'sd128;       5'd22: at = 36'sd64;        5'd23: at = 36'sd32;
      5'd24: at = 36'sd16;        5'd25: at = 36'sd8;         5'd26: at = 36'sd4;         5'd27: at = 36'sd2;
      5'd28: at = 36'sd1;         default: at = 36'sd0;
    endcase
  end
  wire signed [35:0] z0 = $signed(a) * 16;
  wire signed [35:0] sx = x >>> it;
  wire signed [35:0] sy = y >>> it;
  always @(posedge clk) begin
    done <= 1'b0;
    if (start) begin
      if (z0 > HALF) begin z <= z0 - PI28; neg <= 1'b1; end
      else if (z0 < -HALF) begin z <= z0 + PI28; neg <= 1'b1; end
      else begin z <= z0; neg <= 1'b0; end
      x <= K0; y <= 36'sd0; it <= 5'd0; busy <= 1'b1;
    end else if (busy) begin
      if (z >= 0) begin x <= x - sy; y <= y + sx; z <= z - at; end
      else begin x <= x + sy; y <= y - sx; z <= z + at; end
      it <= it + 5'd1;
      if (it == 5'd29) busy <= 1'b0;
    end else if (it == 5'd30) begin
      s <= neg ? -y : y; c <= neg ? -x : x; done <= 1'b1; it <= 5'd31;
    end
  end
endmodule

module fk_engine (
  input  wire          clk,
  input  wire          rst,
  input  wire          start,
  input  wire [223:0]  q,            // 7 x signed Q24 joint angles, q[0] in bits [31:0]
  output reg           busy,
  output reg           done_pulse,
  output reg  [31:0]   cycles,
  output reg           out_we,
  output reg  [5:0]    out_idx,
  output reg  [47:0]   out_xyz       // {z,y,x} workspace-relative u16
);
  localparam signed [35:0] RS = 36'sd1073741824;
  localparam [7:0] LOCAL2 = 8'd240;

  // ---------------- parallel CORDICs ----------------
  reg        c_start;
  wire [6:0] c_done;
  wire signed [35:0] sn [0:6];
  wire signed [35:0] cs [0:6];
  genvar g;
  generate for (g = 0; g < 7; g = g + 1) begin : g_cordic
    cordic_unit u (.clk(clk), .start(c_start), .a(q[32*g +: 32]), .done(c_done[g]), .s(sn[g]), .c(cs[g]));
  end endgenerate

  // ---------------- constant ROM views (one per operand lane) ----------------
  reg  [3:0] jj;
  reg  [5:0] sph;
  reg  [3:0] jw0, jw1, jw2, jwt;
  wire [3:0] j_parent, j_child, j_axis, s_link;
  wire signed [35:0] jv0, jv1, jv2, jvt, sp0, sp1, sp2;
  wire [3:0] d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11;
  wire signed [35:0] y0, y1, y2, y3, y4, y5, y6;
  wire [15:0] r0, r1, r2, r3;
  wire [5:0] pa0, pb0, pa1, pb1, pa2, pb2, pa3, pb3;
  wire signed [31:0] lo0, hi0, lo1, hi1, lo2, hi2, lo3, hi3;
  panda_consts u_r0 (.j_idx(jj), .j_parent(d0), .j_child(d1), .j_axis(d2), .j_word(jw0), .j_val(jv0),
    .s_idx(sph), .s_link(d3), .s_axis(2'd0), .s_pos(sp0), .s_radius(r0), .a_idx(5'd0), .a_val(y0),
    .p_idx(10'd0), .p_a(pa0), .p_b(pb0), .l_idx(3'd0), .l_lo(lo0), .l_hi(hi0));
  panda_consts u_r1 (.j_idx(jj), .j_parent(d4), .j_child(d5), .j_axis(d6), .j_word(jw1), .j_val(jv1),
    .s_idx(sph), .s_link(d7), .s_axis(2'd1), .s_pos(sp1), .s_radius(r1), .a_idx(5'd0), .a_val(y1),
    .p_idx(10'd0), .p_a(pa1), .p_b(pb1), .l_idx(3'd0), .l_lo(lo1), .l_hi(hi1));
  panda_consts u_r2 (.j_idx(jj), .j_parent(d8), .j_child(d9), .j_axis(d10), .j_word(jw2), .j_val(jv2),
    .s_idx(sph), .s_link(d11), .s_axis(2'd2), .s_pos(sp2), .s_radius(r2), .a_idx(5'd0), .a_val(y2),
    .p_idx(10'd0), .p_a(pa2), .p_b(pb2), .l_idx(3'd0), .l_lo(lo2), .l_hi(hi2));
  panda_consts u_rt (.j_idx(jj), .j_parent(j_parent), .j_child(j_child), .j_axis(j_axis), .j_word(jwt), .j_val(jvt),
    .s_idx(sph), .s_link(s_link), .s_axis(2'd0), .s_pos(y3), .s_radius(r3), .a_idx(5'd0), .a_val(y4),
    .p_idx(10'd0), .p_a(pa3), .p_b(pb3), .l_idx(3'd0), .l_lo(lo3), .l_hi(hi3));

  // ---------------- register file: 13 link transforms (16 words each) + LOCAL2 ----------------
  reg signed [35:0] rf [0:255];
  reg  [7:0] ra0, ra1, ra2, rat, rb0, rb1, rb2, wa;
  reg        rf_we;
  reg signed [35:0] rf_wd;
  wire signed [35:0] fa0 = rf[ra0], fa1 = rf[ra1], fa2 = rf[ra2], fat = rf[rat];
  wire signed [35:0] fb0 = rf[rb0], fb1 = rf[rb1], fb2 = rf[rb2];
  always @(posedge clk) if (rf_we) rf[wa] <= rf_wd;

  // ---------------- sequencer ----------------
  localparam [3:0] S_IDLE = 4'd0, S_INIT = 4'd1, S_WAITC = 4'd2, S_JOINT = 4'd3, S_ISSUE = 4'd4,
                   S_DRAIN = 4'd5, S_SPH = 4'd6, S_SDRAIN = 4'd7, S_DONE = 4'd8;
  localparam [1:0] M_C1 = 2'd0, M_C2R = 2'd1, M_C2F = 2'd2, M_SPH = 2'd3;
  reg [3:0] st;
  reg [1:0] mode;
  reg [3:0] e, cnt;
  reg [1:0] si;
  wire [1:0] ei = e[3:2];
  wire [1:0] ej = e[1:0];
  reg  [6:0] cdone_seen;
  reg  signed [35:0] sin_l [0:6];
  reg  signed [35:0] cos_l [0:6];

  // pipeline valid / meta
  reg        v0, v1, v2, v3, v4;
  reg [1:0]  k0, k1, k2, k3, k4;          // 0: rf write, 2: sphere
  reg [7:0]  w0, w1, w2, w3, w4;          // rf write address
  reg [5:0]  n0, n1, n2, n3, n4;          // sphere index
  reg [1:0]  a0x, a1x, a2x, a3x, a4x;     // sphere axis
  reg signed [35:0] t0, t1, t2, t3, t4;
  reg signed [35:0] oa0, oa1, oa2, ob0, ob1, ob2;
  reg signed [71:0] p0, p1, p2, m0, m1, m2;
  reg signed [73:0] sum;
  reg signed [35:0] rnd;
  wire pipe_busy = v0 | v1 | v2 | v3 | v4;

  function [7:0] lb; input [3:0] l; lb = {l, 4'b0000}; endfunction
  wire [7:0] abase = (mode == M_SPH) ? lb(s_link) : lb(j_parent);
  wire signed [35:0] cj = cos_l[j_axis[2:0]];
  wire signed [35:0] sj = sin_l[j_axis[2:0]];

  // turn matrix element [k][j] (translation column j=3 is zero)
  function signed [35:0] turn; input [1:0] k, j; input signed [35:0] c, s;
    begin
      case ({k, j})
        4'b0000, 4'b0101: turn = c;
        4'b0001:          turn = -s;
        4'b0100:          turn = s;
        4'b1010:          turn = RS;
        default:          turn = 36'sd0;
      endcase
    end
  endfunction

  // operand addressing (combinational, for the element being issued)
  wire [1:0] row = (mode == M_SPH) ? si : ei;
  always @* begin
    ra0 = abase + row * 3; ra1 = abase + row * 3 + 8'd1; ra2 = abase + row * 3 + 8'd2; rat = abase + 8'd9 + row;
    rb0 = (ej == 2'd3) ? LOCAL2 + 8'd9  : LOCAL2 + ej;
    rb1 = (ej == 2'd3) ? LOCAL2 + 8'd10 : LOCAL2 + 8'd3 + ej;
    rb2 = (ej == 2'd3) ? LOCAL2 + 8'd11 : LOCAL2 + 8'd6 + ej;
    if (mode == M_C1) begin            // A = joint constant row i (and its translation)
      jw0 = ei * 3; jw1 = ei * 3 + 4'd1; jw2 = ei * 3 + 4'd2; jwt = 4'd9 + ei;
    end else begin                     // B = joint constant column j (fixed joints)
      jw0 = (ej == 2'd3) ? 4'd9  : {2'b00, ej};
      jw1 = (ej == 2'd3) ? 4'd10 : 4'd3 + ej;
      jw2 = (ej == 2'd3) ? 4'd11 : 4'd6 + ej;
      jwt = 4'd0;
    end
  end

  // write-back stage (from S4 registers)
  wire signed [35:0] wb = t4 + rnd;
  always @* begin
    rf_we = 1'b0; wa = w4; rf_wd = wb;
    if (st == S_INIT) begin
      rf_we = 1'b1; wa = {4'd0, cnt}; rf_wd = (cnt == 4'd0 || cnt == 4'd4 || cnt == 4'd8) ? RS : 36'sd0;
    end else if (v4 && k4 == 2'd0) rf_we = 1'b1;
  end

  function [15:0] out16; input signed [35:0] v; reg signed [35:0] t;
    begin t = (v + 36'sd512) >>> 10; out16 = t[15:0] ^ 16'h8000; end
  endfunction

  wire issue = (st == S_ISSUE) || (st == S_SPH);
  integer i;
  always @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE; busy <= 1'b0; done_pulse <= 1'b0; out_we <= 1'b0; cycles <= 0; c_start <= 1'b0;
      v0 <= 0; v1 <= 0; v2 <= 0; v3 <= 0; v4 <= 0;
    end else begin
      done_pulse <= 1'b0; out_we <= 1'b0; c_start <= 1'b0;
      if (busy) cycles <= cycles + 32'd1;

      // ---------- S0: operand fetch ----------
      v0 <= issue;
      if (issue) begin
        case (mode)
          M_C1: begin
            oa0 <= jv0; oa1 <= jv1; oa2 <= jv2;
            ob0 <= turn(2'd0, ej, cj, sj); ob1 <= turn(2'd1, ej, cj, sj); ob2 <= turn(2'd2, ej, cj, sj);
            t0  <= (ej == 2'd3) ? jvt : 36'sd0;
            k0 <= 2'd0; w0 <= (ej == 2'd3) ? LOCAL2 + 8'd9 + ei : LOCAL2 + ei * 3 + ej;
          end
          M_C2R, M_C2F: begin
            oa0 <= fa0; oa1 <= fa1; oa2 <= fa2;
            if (mode == M_C2R) begin ob0 <= fb0; ob1 <= fb1; ob2 <= fb2; end
            else begin ob0 <= jv0; ob1 <= jv1; ob2 <= jv2; end
            t0  <= (ej == 2'd3) ? fat : 36'sd0;
            k0 <= 2'd0; w0 <= (ej == 2'd3) ? lb(j_child) + 8'd9 + ei : lb(j_child) + ei * 3 + ej;
          end
          default: begin   // sphere centre, axis si
            oa0 <= fa0; oa1 <= fa1; oa2 <= fa2;
            ob0 <= sp0; ob1 <= sp1; ob2 <= sp2;
            t0  <= fat;
            k0 <= 2'd2; n0 <= sph; a0x <= si;
          end
        endcase
      end
      // ---------- S1..S4 ----------
      v1 <= v0; k1 <= k0; w1 <= w0; n1 <= n0; a1x <= a0x; t1 <= t0;
      p0 <= oa0 * ob0; p1 <= oa1 * ob1; p2 <= oa2 * ob2;
      v2 <= v1; k2 <= k1; w2 <= w1; n2 <= n1; a2x <= a1x; t2 <= t1;
      m0 <= p0; m1 <= p1; m2 <= p2;
      v3 <= v2; k3 <= k2; w3 <= w2; n3 <= n2; a3x <= a2x; t3 <= t2;
      sum <= m0 + m1 + m2;
      v4 <= v3; k4 <= k3; w4 <= w3; n4 <= n3; a4x <= a3x; t4 <= t3;
      rnd <= ((sum + 74'sd536870912) >>> 30);
      // ---------- S5: sphere emit (rf write is combinational above) ----------
      if (v4 && k4 == 2'd2) begin
        case (a4x)
          2'd0: out_xyz[15:0]  <= out16(wb);
          2'd1: out_xyz[31:16] <= out16(wb);
          default: begin out_xyz[47:32] <= out16(wb); out_we <= 1'b1; out_idx <= n4; end
        endcase
      end

      // ---------- control ----------
      case (st)
        S_IDLE: if (start) begin
          busy <= 1'b1; cycles <= 0; c_start <= 1'b1; cnt <= 0; cdone_seen <= 0; st <= S_INIT;
        end
        S_INIT: begin                          // link 0 = identity (12 writes) while the CORDICs run
          cnt <= cnt + 4'd1;
          if (cnt == 4'd11) st <= S_WAITC;
        end
        S_WAITC: begin
          for (i = 0; i < 7; i = i + 1) if (c_done[i]) begin sin_l[i] <= sn[i]; cos_l[i] <= cs[i]; end
          cdone_seen <= cdone_seen | c_done;
          if ((cdone_seen | c_done) == 7'h7F) begin jj <= 0; st <= S_JOINT; end
        end
        S_JOINT: begin
          e <= 0;
          mode <= (j_axis != 4'd15) ? M_C1 : M_C2F;
          st <= S_ISSUE;
        end
        S_ISSUE: begin
          e <= e + 4'd1;
          if (e == 4'd11) st <= S_DRAIN;
        end
        S_DRAIN: if (!pipe_busy) begin
          if (mode == M_C1) begin mode <= M_C2R; e <= 0; st <= S_ISSUE; end
          else if (jj == 4'd11) begin mode <= M_SPH; sph <= 0; si <= 0; st <= S_SPH; end
          else begin jj <= jj + 4'd1; st <= S_JOINT; end
        end
        S_SPH: begin
          if (si == 2'd2) begin
            si <= 0;
            if (sph == 6'd58) st <= S_SDRAIN; else sph <= sph + 6'd1;
          end else si <= si + 2'd1;
        end
        S_SDRAIN: if (!pipe_busy && !out_we) st <= S_DONE;
        S_DONE: begin busy <= 1'b0; done_pulse <= 1'b1; st <= S_IDLE; end
        default: st <= S_IDLE;
      endcase
    end
  end
endmodule
