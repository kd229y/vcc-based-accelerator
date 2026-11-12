// Collision query engine (spec 6.4): global AABB cull -> coarse-cell range from the
// actual radius (NOT fixed 3x3x3) -> dense cell-id lookup -> cell AABB (clamp +
// distance) cull -> sphere-vs-representative-point distance, early exit on hit.
// Bit-exact with soc_model/core.cpp Model::query() for cover = 2 (user profile):
//   r = radius + point_radius_q(41) + 2 ;  hit iff  |c - p|^2 <= r^2 for some point.
// Coordinates are workspace-relative u16 (world Q14 + 32768).
`timescale 1ns/1ps
module query_engine #(
  parameter PTS_AW  = 15,
  parameter CELL_AW = 12,
  parameter [16:0] RPAD = 17'd43          // ceil(r_point*2^14)=41 + conversion epsilon 2
) (
  input  wire               clk,
  input  wire               rst,
  input  wire               start,
  input  wire [15:0]        cx, cy, cz,
  input  wire [15:0]        radius,
  input  wire               env_valid,
  input  wire [PTS_AW:0]    kept,
  input  wire [47:0]        glo, ghi,
  output reg                busy,
  output reg                done_pulse,
  output reg                hit,
  output reg  [31:0]        cycles,
  output reg  [31:0]        cells_visited,
  output reg  [31:0]        dist_tests,
  // environment read ports, 1-cycle latency
  output reg  [14:0]        cid_raddr,
  input  wire [15:0]        cid_rdata,
  output reg  [CELL_AW-1:0] cell_raddr,
  input  wire [31:0]        sc_rdata,
  input  wire [95:0]        box_rdata,
  output reg  [PTS_AW-1:0]  p_raddr,
  input  wire [47:0]        p_rdata
);
  localparam [3:0] Q_IDLE = 4'd0, Q_PREP = 4'd1, Q_CELL = 4'd2, Q_CID = 4'd3, Q_BOX = 4'd4,
                   Q_BOXCHK = 4'd5, Q_PTS = 4'd6, Q_DONE = 4'd7, Q_BOXSQ = 4'd8;
  reg [3:0] st;

  reg [15:0] c [0:2];
  reg [16:0] r;
  reg [34:0] rsq;
  reg [4:0]  lo [0:2], hi [0:2];
  reg [4:0]  x, y, z;
  reg        cpv, issued_all;

  // ---- range / global cull (combinational from latched c, r) ----
  wire signed [18:0] cm [0:2];
  wire signed [18:0] cp [0:2];
  genvar g;
  generate for (g = 0; g < 3; g = g + 1) begin : g_rng
    assign cm[g] = $signed({3'b000, c[g]}) - $signed({2'b00, r});
    assign cp[g] = $signed({3'b000, c[g]}) + $signed({2'b00, r});
  end endgenerate
  wire [15:0] glo_a [0:2]; wire [15:0] ghi_a [0:2];
  assign glo_a[0] = glo[15:0]; assign glo_a[1] = glo[31:16]; assign glo_a[2] = glo[47:32];
  assign ghi_a[0] = ghi[15:0]; assign ghi_a[1] = ghi[31:16]; assign ghi_a[2] = ghi[47:32];
  wire global_out =
       (cp[0] < $signed({3'b000, glo_a[0]})) || (cm[0] > $signed({3'b000, ghi_a[0]})) ||
       (cp[1] < $signed({3'b000, glo_a[1]})) || (cm[1] > $signed({3'b000, ghi_a[1]})) ||
       (cp[2] < $signed({3'b000, glo_a[2]})) || (cm[2] > $signed({3'b000, ghi_a[2]}));

  function [4:0] lo_cell; input signed [18:0] v; lo_cell = (v < 0) ? 5'd0 : v[15:11]; endfunction
  function [4:0] hi_cell; input signed [18:0] v; hi_cell = (v[18:11] > 8'd31) ? 5'd31 : v[15:11]; endfunction

  function [14:0] morton5; input [4:0] a, b, d; integer i;
    begin for (i = 0; i < 5; i = i + 1) begin morton5[3*i] = a[i]; morton5[3*i+1] = b[i]; morton5[3*i+2] = d[i]; end end
  endfunction

  // ---- cell AABB clamp distance ----
  wire [15:0] blo [0:2]; wire [15:0] bhi [0:2];
  assign blo[0] = box_rdata[15:0];  assign blo[1] = box_rdata[31:16]; assign blo[2] = box_rdata[47:32];
  assign bhi[0] = box_rdata[63:48]; assign bhi[1] = box_rdata[79:64]; assign bhi[2] = box_rdata[95:80];
  reg  [31:0] bsq [0:2];
  reg  [15:0] bd [0:2];
  reg  [15:0] p_start, p_end, p_cur;

  function [15:0] clampd; input [15:0] cc, l, h;
    clampd = (cc < l) ? (l - cc) : (cc > h) ? (cc - h) : 16'd0;
  endfunction

  // ---- point distance pipeline: issue -> (rdata) square -> sum/compare ----
  reg        v1, v2, v3;
  reg signed [16:0] pe [0:2];
  reg [31:0] psq [0:2];
  wire signed [16:0] e0 = $signed({1'b0, c[0]}) - $signed({1'b0, p_rdata[15:0]});
  wire signed [16:0] e1 = $signed({1'b0, c[1]}) - $signed({1'b0, p_rdata[31:16]});
  wire signed [16:0] e2 = $signed({1'b0, c[2]}) - $signed({1'b0, p_rdata[47:32]});
  wire [33:0] pdist = psq[0] + psq[1] + psq[2];
  wire        phit  = v3 && ({1'b0, pdist} <= rsq);

  // advance x -> y -> z ; returns done flag
  reg adv_done;
  reg [4:0] nx, ny, nz;
  always @* begin
    nx = x; ny = y; nz = z; adv_done = 1'b0;
    if (x != hi[0]) nx = x + 5'd1;
    else begin
      nx = lo[0];
      if (y != hi[1]) ny = y + 5'd1;
      else begin
        ny = lo[1];
        if (z != hi[2]) nz = z + 5'd1;
        else adv_done = 1'b1;
      end
    end
  end

  always @* begin
    cid_raddr  = morton5(x, y, z);
    cell_raddr = cid_rdata[CELL_AW-1:0];
    p_raddr    = p_cur[PTS_AW-1:0];
  end

  integer a;
  always @(posedge clk) begin
    if (rst) begin
      st <= Q_IDLE; busy <= 1'b0; done_pulse <= 1'b0; hit <= 1'b0; cycles <= 0;
      cells_visited <= 0; dist_tests <= 0; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
    end else begin
      done_pulse <= 1'b0;
      if (busy) cycles <= cycles + 32'd1;
      case (st)
        Q_IDLE: if (start) begin
          c[0] <= cx; c[1] <= cy; c[2] <= cz;
          r <= {1'b0, radius} + RPAD;
          busy <= 1'b1; hit <= 1'b0; cycles <= 0; cells_visited <= 0; dist_tests <= 0;
          st <= Q_PREP;
        end
        Q_PREP: begin
          rsq <= r * r;
          for (a = 0; a < 3; a = a + 1) begin
            lo[a] <= lo_cell(cm[a]);
            hi[a] <= hi_cell(cp[a]);
          end
          x <= lo_cell(cm[0]); y <= lo_cell(cm[1]); z <= lo_cell(cm[2]);
          cpv <= 1'b0; issued_all <= 1'b0;
          if (!env_valid || kept == 0 || global_out) st <= Q_DONE;
          else st <= Q_CELL;
        end
        // cell scan: one cell-id lookup issued per cycle; the previous lookup is examined in the same cycle
        Q_CELL: begin
          if (cpv && cid_rdata != 16'hFFFF) begin       // previous cell non-empty: cell_raddr = its id
            cpv <= 1'b0; st <= Q_BOX;                     // (x,y,z) already points at the next cell
          end else if (issued_all) st <= Q_DONE;
          else begin                                      // cid_raddr = morton(x,y,z) issued now
            cpv <= 1'b1; cells_visited <= cells_visited + 32'd1;
            x <= nx; y <= ny; z <= nz;
            if (adv_done) issued_all <= 1'b1;
          end
        end
        Q_BOX: begin               // sc/box data valid
          p_start <= sc_rdata[15:0];
          p_end   <= sc_rdata[15:0] + sc_rdata[31:16];
          for (a = 0; a < 3; a = a + 1) bd[a] <= clampd(c[a], blo[a], bhi[a]);
          st <= Q_BOXSQ;
        end
        Q_BOXSQ: begin
          for (a = 0; a < 3; a = a + 1) bsq[a] <= bd[a] * bd[a];
          st <= Q_BOXCHK;
        end
        Q_BOXCHK: begin
          if ({1'b0, bsq[0]} + bsq[1] + bsq[2] > rsq) begin
            st <= Q_CELL;
          end else begin
            p_cur <= p_start; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
            st <= Q_PTS;
          end
        end
        Q_PTS: begin
          // stage 0: p_raddr = p_cur this cycle
          v1 <= (p_cur != p_end);
          if (p_cur != p_end) begin p_cur <= p_cur + 16'd1; dist_tests <= dist_tests + 32'd1; end
          // stage 1: squares
          v2 <= v1;
          pe[0] <= e0; pe[1] <= e1; pe[2] <= e2;
          v3 <= v2;
          psq[0] <= pe[0] * pe[0]; psq[1] <= pe[1] * pe[1]; psq[2] <= pe[2] * pe[2];
          // stage 2: compare
          if (phit) begin
            hit <= 1'b1; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; st <= Q_DONE;
          end else if (p_cur == p_end && !v1 && !v2 && !v3) begin
            st <= Q_CELL;
          end
        end
        Q_DONE: begin
          busy <= 1'b0; done_pulse <= 1'b1; st <= Q_IDLE;
        end
        default: st <= Q_IDLE;
      endcase
    end
  end
endmodule
