// RRT-Connect planning engine (spec 6.5): control FSM + integer units, all on chip.
// Bit-exact with soc_model/core.cpp Model::plan()/extend()/edge()/state()/nearest():
//   * xorshift32 sampling, sample = lo + (r*(hi-lo) >> 32), optional goal bias
//   * full-scan squared-Euclidean nearest neighbour (first minimum wins)
//   * integer ceil-sqrt steering, round-half-up division
//   * VAMP rake edge validation (rake 8, resolution 32, same sample order)
//   * state check = joint limits -> FK -> 690 self pairs -> 59 environment queries
//   * fixed alternation of the two trees, connect loop <= 1000, backtrack start->goal
// FK and query engines are external (shared with the host path in vcc_core).
`timescale 1ns/1ps
module rrt_engine #(
  parameter [16:0] RPAD = 17'd43,   // must equal query_engine.RPAD
  parameter MAXN_AW = 10,          // node memory per tree = 2^MAXN_AW (cfg_max_nodes must be <= this)
  parameter PATH_AW = MAXN_AW + 1
) (
  input  wire               clk,
  input  wire               rst,
  input  wire               start,
  input  wire [223:0]       q_start,
  input  wire [223:0]       q_goal,
  input  wire [31:0]        cfg_range,       // floor(rrt_range_rad * 2^24)
  input  wire [15:0]        cfg_max_nodes,   // rrt_max_nodes_per_tree
  input  wire [31:0]        cfg_max_iter,
  input  wire [31:0]        cfg_budget,      // state_evaluation_budget
  input  wire [31:0]        cfg_seed,
  input  wire [31:0]        cfg_goal_bias,   // 0 = off
  input  wire [15:0]        cfg_margin,      // ceil(fk_margin * 2^14)
  output reg                busy,
  output reg                done_pulse,
  output reg  [3:0]         status,
  output reg  [PATH_AW:0]   path_len,
  input  wire [PATH_AW-1:0] path_raddr,
  output wire [223:0]       path_rdata,
  output reg  [31:0]        st_iters, st_nodes, st_states, st_edges, st_queries, cycles,
  // FK engine
  output reg                fk_start,
  output reg  [223:0]       fk_q,
  input  wire               fk_done,
  input  wire               fk_we,
  input  wire [5:0]         fk_idx,
  input  wire [47:0]        fk_xyz,
  // query engine
  output reg                qe_start,
  output reg  [47:0]        qe_c,
  output reg  [15:0]        qe_r,
  input  wire               qe_done,
  input  wire               qe_hit,
  // environment summary for the planner-side global AABB reject (same test as query_engine Q_PREP)
  input  wire               env_ok,          // env_valid && kept != 0
  input  wire [47:0]        env_glo,
  input  wire [47:0]        env_ghi
);
  localparam [3:0] OK = 4'd0, BUDGET = 4'd1, INV_START = 4'd2, INV_GOAL = 4'd3, CAPACITY = 4'd4;
  localparam [15:0] NIL = 16'hFFFF;

  // ---------------- state encoding ----------------
  localparam [5:0]
    P_IDLE=0, P_S1=1, P_S2=2, P_S3=3, P_INIT0=4, P_INIT1=5, P_ITER=6, P_SAMP=7, P_SAMPM=8, P_SAMPW=9,
    P_AEXT=10, P_BEXT=11, P_SWAP=12, P_FIN=13,
    B_CRD=14, B_CNT=15, B_WRD=16, B_WR=17, B_GRD=18, B_G=19,
    S_BEGIN=20, S_LIM=21, S_FK=22, S_FKW=23, S_PAIR=24, S_Q=25, S_QW=26,
    E_BEGIN=27, E_A=28, E_SQ=29, E_SQRT=30, E_BLK=31, E_SMP=32, E_SMPM=33, E_SMPD=34, E_NEXT=35,
    X_BEGIN=36, X_NNRD=37, X_NNDAT=38, X_NNSQ=39, X_NORM=40, X_STEP=41, X_STEPM=42, X_STEPD=43,
    X_CHK=44, X_EDGED=45, P_DIRECT=46, P_PATH1=47, E_SQRTW=48, X_NORMW=49, P_PATH0=50,
    E_CALL=51, E_CHKS=52, X_STEPW=53, P_SAMPX=54;
  reg [5:0] st, s_ret, e_ret, x_ret;

  // ---------------- constants (two ROM views: pair A / pair B) ----------------
  reg  [9:0]  pc;            // pair counter
  reg  [5:0]  qn;            // query sphere counter
  reg  [2:0]  jl;            // joint index for limits
  wire [5:0]  pa, pb, pa2, pb2;
  wire [15:0] rad_a, rad_b;
  wire signed [31:0] llo, lhi, llo2, lhi2;
  wire [3:0] u0, u1, u2, u3, u4, u5, u6, u7;
  wire signed [35:0] w0, w1, w2, w3, w4, w5;
  wire [5:0] sidx_a = (st == S_PAIR) ? gq : qn;
  panda_consts u_ca (.j_idx(4'd0), .j_parent(u0), .j_child(u1), .j_axis(u2), .j_word(4'd0), .j_val(w0),
    .s_idx(sidx_a), .s_link(u3), .s_axis(2'd0), .s_pos(w1), .s_radius(rad_a), .a_idx(5'd0), .a_val(w2),
    .p_idx(pc), .p_a(pa), .p_b(pb), .l_idx(jl), .l_lo(llo), .l_hi(lhi));
  panda_consts u_cb (.j_idx(4'd0), .j_parent(u4), .j_child(u5), .j_axis(u6), .j_word(4'd0), .j_val(w3),
    .s_idx(pb), .s_link(u7), .s_axis(2'd0), .s_pos(w4), .s_radius(rad_b), .a_idx(5'd0), .a_val(w5),
    .p_idx(10'd0), .p_a(pa2), .p_b(pb2), .l_idx(3'd0), .l_lo(llo2), .l_hi(lhi2));

  // ---------------- memories ----------------
  // trees: address {tree, index}; word {parent[15:0], q[223:0]}
  reg                  t_we;
  reg  [MAXN_AW:0]     t_waddr, t_raddr;
  reg  [239:0]         t_wdata;
  wire [239:0]         t_rdata;
  ram_sdp #(.DW(240), .AW(MAXN_AW + 1)) u_tree (.clk(clk), .we(t_we), .waddr(t_waddr), .wdata(t_wdata),
    .raddr(t_raddr), .rdata(t_rdata));
  reg                  pm_we;
  reg  [PATH_AW-1:0]   pm_waddr;
  reg  [223:0]         pm_wdata;
  ram_sdp #(.DW(224), .AW(PATH_AW)) u_path (.clk(clk), .we(pm_we), .waddr(pm_waddr), .wdata(pm_wdata),
    .raddr(path_raddr), .rdata(path_rdata));
  reg [47:0] sph [0:63];
  always @(posedge clk) if (fk_we) sph[fk_idx] <= fk_xyz;

  // ---------------- arithmetic ----------------
  reg  signed [40:0] ma, mb;
  reg  signed [81:0] mres, mp;
  reg                mw;
  // 7 parallel round_div units (one per joint), shared divisor
  reg                dv_start;
  reg  signed [63:0] dn [0:6];
  reg  [31:0]        dv_d;
  wire [6:0]         dv_dn;
  wire signed [63:0] dq [0:6];
  wire               dv_done = dv_dn[0];
  genvar gd;
  generate for (gd = 0; gd < 7; gd = gd + 1) begin : g_div
    div_round u_div (.clk(clk), .rst(rst), .start(dv_start), .n(dn[gd]), .d(dv_d), .done(dv_dn[gd]), .q(dq[gd]));
  end endgenerate
  integer jd;
  reg                sq_start;
  reg  [63:0]        sq_n;
  wire               sq_done;
  wire [32:0]        sq_r;
  isqrt_ceil u_sqrt (.clk(clk), .rst(rst), .start(sq_start), .n(sq_n), .done(sq_done), .r_ceil(sq_r));

  // squared-distance pipeline over 7 joints
  reg  [3:0]         sj;
  reg                s_v0, s_v1, s_v2;
  reg  signed [32:0] s_d, s_dr;
  reg  [65:0]        s_sq;
  reg  [63:0]        s_acc;

  // ---------------- registers ----------------
  reg  [223:0] sq;                 // state() argument
  reg          s_ok;
  reg  [223:0] ea, eb;             // edge() arguments
  reg          e_ok;
  reg  [32:0]  e_dist;
  reg  [15:0]  e_blocks, e_block;
  reg  [3:0]   e_lane;
  reg  [223:0] eq;                 // edge sample
  reg          xt;                 // extend: physical tree
  reg  [223:0] tgt;                // extend target
  reg  [15:0]  x_i, x_best;
  reg  [63:0]  x_bestd;
  reg  [223:0] x_a, x_nq, x_nodeq;
  reg  [32:0]  x_norm;
  reg          x_reached;
  reg  [15:0]  x_res;              // NIL = -1
  reg  [223:0] x_resq;
  reg  [3:0]   err;
  reg  [15:0]  size0, size1;
  reg          cur_a;              // physical tree that is "a"
  reg  [31:0]  it, gb_cnt, conn_k;
  reg  [31:0]  seed;
  reg  [223:0] sample;
  reg  [2:0]   j;
  reg  [15:0]  ai;
  reg  [223:0] conn_tgt;
  reg  [15:0]  b_idx, b_sidx, b_gidx;
  reg  [PATH_AW:0] b_len, b_d;
  // pair pipeline
  reg               pv;
  reg signed [16:0] pdx, pdy, pdz;
  reg [17:0]        prr;

  function signed [31:0] qj; input [223:0] v; input [2:0] k; qj = v[32*k +: 32]; endfunction
  function signed [32:0] qx; input [223:0] v; input [2:0] k; qx = {v[32*k+31], v[32*k +: 32]}; endfunction
  function [31:0] xorshift; input [31:0] s; reg [31:0] t;
    begin t = s ^ (s << 13); t = t ^ (t >> 17); xorshift = t ^ (t << 5); end
  endfunction
  function [15:0] tsize; input t; input [15:0] s0, s1; tsize = t ? s1 : s0; endfunction

  wire [31:0] rnd = xorshift(seed);
  wire [15:0] tsz = xt ? size1 : size0;
  // ---- self-collision check: 4 pair lanes, 3-stage pipeline (fetch/diff -> squares -> compare) ----
  localparam [9:0] NPAIR = 10'd690, NGRP = 10'd173;      // ceil(690 / 4)
  wire        pair_issue = (st == S_PAIR) && (pc != NGRP);
  wire [3:0]  lhit, lv1s, lv2s;
  genvar gl;
  generate for (gl = 0; gl < 4; gl = gl + 1) begin : g_pair
    wire [9:0]  idx = {pc[7:0], 2'b00} + gl;
    wire [5:0]  ia, ib;
    wire [15:0] rsum;
    panda_pairs u_pr (.idx(idx), .a(ia), .b(ib), .rsum(rsum));
    reg               lv1, lv2;
    reg signed [16:0] dx, dy, dz;
    reg [17:0]        rr;
    reg [35:0]        d2r, rr2r;
    always @(posedge clk) begin
      lv1 <= pair_issue && (idx < NPAIR);
      dx <= $signed({1'b0, sph[ia][15:0]})  - $signed({1'b0, sph[ib][15:0]});
      dy <= $signed({1'b0, sph[ia][31:16]}) - $signed({1'b0, sph[ib][31:16]});
      dz <= $signed({1'b0, sph[ia][47:32]}) - $signed({1'b0, sph[ib][47:32]});
      rr <= rsum + {cfg_margin, 1'b0};
      lv2 <= lv1 && (st == S_PAIR);
      d2r <= dx * dx + dy * dy + dz * dz;
      rr2r <= rr * rr;
    end
    assign lhit[gl] = lv2 && (d2r <= rr2r);
    assign lv1s[gl] = lv1;
    assign lv2s[gl] = lv2;
  end endgenerate
  wire phit = |lhit;

  // ---- global AABB reject (identical to query_engine Q_PREP), precomputed for all 59 spheres
  //      during the pair check (2-stage pipeline) -> gvec; S_Q then reads one bit per sphere ----
  reg  [5:0]  gq, g1i;
  reg         g1v;
  reg  [47:0] qc;
  reg  [16:0] qrr;
  reg  [63:0] gvec;
  function gout_axis; input [15:0] c, lo, hi; input [16:0] r;
    reg signed [18:0] cp, cm;
    begin
      cp = $signed({3'b000, c}) + $signed({2'b00, r});
      cm = $signed({3'b000, c}) - $signed({2'b00, r});
      gout_axis = (cp < $signed({3'b000, lo})) || (cm > $signed({3'b000, hi}));
    end
  endfunction
  wire q_gout = !env_ok || gout_axis(qc[15:0], env_glo[15:0], env_ghi[15:0], qrr) ||
                gout_axis(qc[31:16], env_glo[31:16], env_ghi[31:16], qrr) ||
                gout_axis(qc[47:32], env_glo[47:32], env_ghi[47:32], qrr);
  always @(posedge clk) begin
    g1v <= (st == S_PAIR) && (gq != 6'd59);           // stage 1: fetch centre + radius
    g1i <= gq;
    qc  <= sph[gq];
    qrr <= {1'b0, rad_a + cfg_margin} + RPAD;
    if (g1v) gvec[g1i] <= q_gout;                     // stage 2: compare
  end

  // squared-distance diff source
  always @* begin
    case (st)
      E_SQ:    s_d = qx(eb, sj[2:0]) - qx(ea, sj[2:0]);
      default: s_d = qx(tgt, sj[2:0]) - qx(x_nodeq, sj[2:0]);
    endcase
  end
  wire s_issue = ((st == E_SQ) || (st == X_NNSQ)) && (sj < 4'd7);

  // tree / path write ports
  always @* begin
    t_we = 1'b0; t_waddr = 0; t_wdata = 0;
    pm_we = 1'b0; pm_waddr = 0; pm_wdata = 0;
    t_raddr = {xt, x_i[MAXN_AW-1:0]};
    case (st)
      P_INIT0: begin t_we = 1'b1; t_waddr = {1'b0, {MAXN_AW{1'b0}}}; t_wdata = {NIL, q_start}; end
      P_INIT1: begin t_we = 1'b1; t_waddr = {1'b1, {MAXN_AW{1'b0}}}; t_wdata = {NIL, q_goal}; end
      X_EDGED: if (e_ok) begin t_we = 1'b1; t_waddr = {xt, tsz[MAXN_AW-1:0]}; t_wdata = {x_best, x_nq}; end
      P_PATH0: begin pm_we = 1'b1; pm_waddr = 0; pm_wdata = q_start; end
      P_PATH1: begin pm_we = 1'b1; pm_waddr = 1; pm_wdata = q_goal; end
      B_CRD, B_WRD: t_raddr = {1'b0, b_idx[MAXN_AW-1:0]};
      B_GRD:        t_raddr = {1'b1, b_idx[MAXN_AW-1:0]};
      B_WR: begin pm_we = 1'b1; pm_waddr = b_len - 1 - b_d; pm_wdata = t_rdata[223:0]; end
      B_G:  if (b_d != 0) begin pm_we = 1'b1; pm_waddr = b_len + b_d - 1; pm_wdata = t_rdata[223:0]; end
      default: ;
    endcase
  end

  always @(posedge clk) begin
    if (rst) begin
      st <= P_IDLE; busy <= 1'b0; done_pulse <= 1'b0; status <= OK; fk_start <= 1'b0; qe_start <= 1'b0;
      dv_start <= 1'b0; sq_start <= 1'b0; s_v0 <= 1'b0; s_v1 <= 1'b0; mw <= 1'b0; s_v2 <= 1'b0; path_len <= 0; cycles <= 0;
    end else begin
      done_pulse <= 1'b0; fk_start <= 1'b0; qe_start <= 1'b0; dv_start <= 1'b0; sq_start <= 1'b0;
      if (busy) cycles <= cycles + 32'd1;
      // generic multiplier and squared-distance pipeline
      mp <= ma * mb; mres <= mp;           // 2-stage multiplier (DSP MREG/PREG)
      s_v0 <= s_issue; s_dr <= s_d; s_v1 <= s_v0; s_v2 <= s_v1;
      s_sq <= s_dr * s_dr;
      if (s_v1) s_acc <= s_acc + s_sq[63:0];

      case (st)
        // ================= plan() =================
        P_IDLE: if (start) begin
          busy <= 1'b1; cycles <= 0; err <= OK; status <= OK; path_len <= 0;
          st_iters <= 0; st_nodes <= 0; st_states <= 0; st_edges <= 0; st_queries <= 0;
          sq <= q_start; s_ret <= P_S1; st <= S_BEGIN;
        end
        P_S1: if (!s_ok) begin status <= (err != OK) ? err : INV_START; st <= P_FIN; end
              else begin sq <= q_goal; s_ret <= P_S2; st <= S_BEGIN; end
        P_S2: if (!s_ok) begin status <= (err != OK) ? err : INV_GOAL; st <= P_FIN; end
              else if (q_start == q_goal) begin path_len <= 1; status <= OK; st <= P_PATH0; end
              else begin ea <= q_start; eb <= q_goal; e_ret <= P_S3; st <= E_BEGIN; end
        P_S3: if (e_ok) begin path_len <= 2; status <= OK; st <= P_PATH0; end
              else if (err != OK) begin status <= err; st <= P_FIN; end
              else st <= P_INIT0;
        P_PATH0: st <= (path_len == 2) ? P_PATH1 : P_FIN;
        P_PATH1: st <= P_FIN;
        P_INIT0: st <= P_INIT1;
        P_INIT1: begin
          size0 <= 16'd1; size1 <= 16'd1; cur_a <= 1'b0; seed <= cfg_seed | 32'd1; st_nodes <= 32'd2;
          it <= 0; gb_cnt <= 0; st <= P_ITER;
        end
        P_ITER: if (it == cfg_max_iter) begin status <= BUDGET; st <= P_FIN; end
                else begin st_iters <= st_iters + 32'd1; j <= 3'd0; jl <= 3'd0; st <= P_SAMP; end
        P_SAMP: begin   // r = random(); ma*mb = r*(hi-lo)
          seed <= rnd;
          ma <= $signed({9'd0, rnd});
          mb <= $signed({9'd0, lhi - llo});
          st <= P_SAMPM;
        end
        P_SAMPM: begin mw <= ~mw; if (mw) st <= P_SAMPW; end
        P_SAMPW: begin
          sample[32*j +: 32] <= llo + mres[63:32];
          if (j == 3'd6) begin
            st <= P_SAMPX;
          end else begin j <= j + 3'd1; jl <= jl + 3'd1; st <= P_SAMP; end
        end
        P_SAMPX: begin
          tgt <= (cfg_goal_bias != 0 && gb_cnt == 0) ? (cur_a ? q_start : q_goal) : sample;   // b[0].q
          gb_cnt <= (cfg_goal_bias != 0 && gb_cnt + 1 == cfg_goal_bias) ? 32'd0 : gb_cnt + 32'd1;
          xt <= cur_a; x_ret <= P_AEXT; st <= X_BEGIN;   // extend(a, sample)
        end
        P_AEXT: begin
          if (err != OK) begin status <= err; st <= P_FIN; end
          else if (x_res != NIL) begin
            ai <= x_res; conn_tgt <= x_resq; tgt <= x_resq; conn_k <= 0;
            xt <= ~cur_a; x_ret <= P_BEXT; st <= X_BEGIN;
          end else st <= P_SWAP;
        end
        P_BEXT: begin
          if (err != OK) begin status <= err; st <= P_FIN; end
          else if (x_res == NIL) st <= P_SWAP;
          else if (x_reached) begin
            // backtrack: physical tree 0 is always the start tree
            b_sidx <= cur_a ? x_res : ai;
            b_gidx <= cur_a ? ai : x_res;
            b_idx <= cur_a ? x_res : ai; b_len <= 0;
            st <= B_CRD;
          end else if (conn_k + 1 == 1000) st <= P_SWAP;
          else begin conn_k <= conn_k + 1; xt <= ~cur_a; x_ret <= P_BEXT; st <= X_BEGIN; end
        end
        P_SWAP: begin cur_a <= ~cur_a; it <= it + 32'd1; st <= P_ITER; end
        P_FIN: begin busy <= 1'b0; done_pulse <= 1'b1; st <= P_IDLE; end

        // ---- backtrack ----
        B_CRD: st <= B_CNT;
        B_CNT: begin
          b_len <= b_len + 1'b1;
          if (t_rdata[239:224] == NIL) begin b_idx <= b_sidx; b_d <= 0; st <= B_WRD; end
          else begin b_idx <= t_rdata[239:224]; st <= B_CRD; end
        end
        B_WRD: st <= B_WR;
        B_WR: begin   // path[len-1-d] = node
          b_d <= b_d + 1'b1;
          if (t_rdata[239:224] == NIL) begin b_idx <= b_gidx; b_d <= 0; st <= B_GRD; end
          else begin b_idx <= t_rdata[239:224]; st <= B_WRD; end
        end
        B_GRD: st <= B_G;
        B_G: begin    // first read is the junction node itself (skipped)
          if (t_rdata[239:224] == NIL) begin path_len <= b_len + b_d; status <= OK; st <= P_FIN; end
          else begin b_idx <= t_rdata[239:224]; b_d <= b_d + 1'b1; st <= B_GRD; end
        end

        // ================= state(q) =================
        S_BEGIN: begin
          st_states <= st_states + 32'd1;
          if (st_states + 32'd1 > cfg_budget) begin err <= BUDGET; s_ok <= 1'b0; st <= s_ret; end
          else begin jl <= 3'd0; st <= S_LIM; end
        end
        S_LIM: begin
          if ($signed(qj(sq, jl)) < llo || $signed(qj(sq, jl)) > lhi) begin s_ok <= 1'b0; st <= s_ret; end
          else if (jl == 3'd6) st <= S_FK;
          else jl <= jl + 3'd1;
        end
        S_FK: begin fk_q <= sq; fk_start <= 1'b1; st <= S_FKW; end
        S_FKW: if (fk_done) begin pc <= 10'd0; gq <= 6'd0; st <= S_PAIR; end
        S_PAIR: begin
          // 4 pairs per cycle are issued by the g_pair lanes (group pc)
          if (pc != NGRP) pc <= pc + 10'd1;
          if (gq != 6'd59) gq <= gq + 6'd1;      // gvec fill (needs 61 cycles < pair check)
          if (phit) begin s_ok <= 1'b0; st <= s_ret; end
          else if (pc == NGRP && lv1s == 4'd0 && lv2s == 4'd0) begin qn <= 6'd0; st <= S_Q; end
        end
        S_Q: begin
          st_queries <= st_queries + 32'd1;
          if (gvec[qn]) begin                   // global AABB reject: miss, 1 cycle per sphere
            if (qn == 6'd58) begin s_ok <= 1'b1; st <= s_ret; end
            else qn <= qn + 6'd1;
          end else begin
            qe_c <= sph[qn]; qe_r <= rad_a + cfg_margin; qe_start <= 1'b1; st <= S_QW;
          end
        end
        S_QW: if (qe_done) begin
          if (qe_hit) begin s_ok <= 1'b0; st <= s_ret; end
          else if (qn == 6'd58) begin s_ok <= 1'b1; st <= s_ret; end
          else begin qn <= qn + 6'd1; st <= S_Q; end
        end

        // ================= edge(a, b) =================
        E_BEGIN: begin st_edges <= st_edges + 32'd1; sq <= ea; s_ret <= E_A; st <= S_BEGIN; end
        E_A: if (!s_ok) begin e_ok <= 1'b0; st <= e_ret; end
             else begin sj <= 0; s_acc <= 0; st <= E_SQ; end
        E_SQ: begin sj <= sj + 4'd1; if (sj == 4'd9) begin sq_n <= s_acc; sq_start <= 1'b1; st <= E_SQRT; end end
        E_SQRT: if (sq_done) begin e_dist <= sq_r; st <= E_SQRTW; end
        E_SQRTW: begin
          // blocks = max(1, ceil(distance*resolution / (rake * 2^24))), rake*2^24 = 2^27, resolution = 32
          e_blocks <= ((({31'd0, e_dist} << 5) + 64'd134217727) >> 27) == 0 ? 16'd1 : ((({31'd0, e_dist} << 5) + 64'd134217727) >> 27);
          e_block <= 0; e_lane <= 4'd1; st <= E_BLK;
        end
        E_BLK: begin j <= 3'd0; st <= E_SMP; end
        E_SMP: begin   // (b-a)*num
          ma <= qx(eb, j) - qx(ea, j);
          mb <= $signed({25'd0, e_lane * e_blocks - e_block});
          st <= E_SMPM;
        end
        E_SMPM: begin mw <= ~mw; if (mw) st <= E_SMPD; end
        E_SMPD: begin
          dn[j] <= mres[63:0];
          if (j == 3'd6) begin dv_d <= {13'd0, e_blocks, 3'b000}; dv_start <= 1'b1; st <= E_NEXT; end
          else begin j <= j + 3'd1; st <= E_SMP; end
        end
        E_NEXT: if (dv_done) begin
          for (jd = 0; jd < 7; jd = jd + 1) eq[32*jd +: 32] <= qj(ea, jd) + dq[jd][31:0];
          st <= E_CALL;
        end
        E_CALL: begin sq <= eq; s_ret <= E_CHKS; st <= S_BEGIN; end
        E_CHKS: begin
          if (!s_ok) begin e_ok <= 1'b0; st <= e_ret; end
          else if (e_lane == 4'd8) begin
            e_lane <= 4'd1; e_block <= e_block + 16'd1;
            if (e_block + 16'd1 == e_blocks) begin e_ok <= 1'b1; st <= e_ret; end
            else st <= E_BLK;
          end else begin e_lane <= e_lane + 4'd1; st <= E_BLK; end
        end

        // ================= extend(tree, target) =================
        X_BEGIN: begin x_i <= 0; x_best <= 0; x_bestd <= {64{1'b1}}; st <= X_NNRD; end
        X_NNRD: st <= X_NNDAT;                          // t_raddr = {xt, x_i}
        X_NNDAT: begin x_nodeq <= t_rdata[223:0]; sj <= 0; s_acc <= 0; st <= X_NNSQ; end
        X_NNSQ: begin
          sj <= sj + 4'd1;
          if (sj == 4'd9) begin
            if (s_acc < x_bestd) begin x_bestd <= s_acc; x_best <= x_i; x_a <= x_nodeq; end
            if (x_i + 1 == tsize(xt, size0, size1)) st <= X_NORM;
            else begin x_i <= x_i + 16'd1; st <= X_NNRD; end
          end
        end
        X_NORM: begin sq_n <= x_bestd; sq_start <= 1'b1; st <= X_NORMW; end
        X_NORMW: if (sq_done) begin
          x_norm <= sq_r;
          if (sq_r == 0) begin x_reached <= 1'b1; x_res <= x_best; x_resq <= x_a; st <= x_ret; end
          else if (sq_r <= {1'b0, cfg_range}) begin x_reached <= 1'b1; x_nq <= tgt; st <= X_CHK; end
          else begin x_reached <= 1'b0; j <= 3'd0; st <= X_STEP; end
        end
        X_STEP: begin   // (target-a)*range
          ma <= qx(tgt, j) - qx(x_a, j);
          mb <= $signed({9'd0, cfg_range});
          st <= X_STEPM;
        end
        X_STEPM: begin mw <= ~mw; if (mw) st <= X_STEPD; end
        X_STEPD: begin
          dn[j] <= mres[63:0];
          if (j == 3'd6) begin dv_d <= x_norm[31:0]; dv_start <= 1'b1; st <= X_STEPW; end
          else begin j <= j + 3'd1; st <= X_STEP; end
        end
        X_STEPW: if (dv_done) begin
          for (jd = 0; jd < 7; jd = jd + 1) x_nq[32*jd +: 32] <= qj(x_a, jd) + dq[jd][31:0];
          st <= X_CHK;
        end
        X_CHK: begin
          if (x_nq == x_a) begin x_res <= NIL; st <= x_ret; end
          else if (tsize(xt, size0, size1) >= cfg_max_nodes || tsize(xt, size0, size1) >= (1 << MAXN_AW)) begin
            err <= CAPACITY; x_res <= NIL; st <= x_ret;
          end else begin ea <= x_a; eb <= x_nq; e_ret <= X_EDGED; st <= E_BEGIN; end
        end
        X_EDGED: begin   // tree write via comb port when e_ok
          if (!e_ok) begin x_res <= NIL; st <= x_ret; end
          else begin
            x_res <= tsize(xt, size0, size1); x_resq <= x_nq;
            if (xt) size1 <= size1 + 16'd1; else size0 <= size0 + 16'd1;
            st_nodes <= st_nodes + 32'd1;
            st <= x_ret;
          end
        end
        default: st <= P_IDLE;
      endcase
    end
  end
endmodule
