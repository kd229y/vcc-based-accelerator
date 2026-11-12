// Sequential integer helpers for the planner, bit-exact with core.cpp.
`timescale 1ns/1ps

// round_div(n, d) = floor((n + d/2) / d), d > 0   (core.cpp round_div / floor_div)
// 64-iteration restoring divider.
module div_round (
  input  wire               clk,
  input  wire               rst,
  input  wire               start,
  input  wire signed [63:0] n,
  input  wire [31:0]        d,
  output reg                done,
  output reg  signed [63:0] q
);
  reg        busy, neg;
  reg [6:0]  i;
  reg [63:0] num, quo;
  reg [64:0] rem;
  reg [31:0] dd;
  wire signed [64:0] nn = {n[63], n} + {33'd0, d[31:1]};
  wire [64:0] trial = {rem[63:0], num[63]} - {33'd0, dd};
  always @(posedge clk) begin
    if (rst) begin busy <= 1'b0; done <= 1'b0; end
    else begin
      done <= 1'b0;
      if (start) begin
        busy <= 1'b1; i <= 7'd0; dd <= d; rem <= 65'd0; quo <= 64'd0;
        if (!nn[64]) begin num <= nn[63:0]; neg <= 1'b0; end
        else begin num <= (-nn) + {33'd0, d} - 65'd1; neg <= 1'b1; end
      end else if (busy) begin
        if (!trial[64]) begin rem <= trial; quo <= {quo[62:0], 1'b1}; end
        else begin rem <= {rem[63:0], num[63]}; quo <= {quo[62:0], 1'b0}; end
        num <= {num[62:0], 1'b0};
        i <= i + 7'd1;
        if (i == 7'd63) begin
          busy <= 1'b0; done <= 1'b1;
          q <= neg ? -$signed({quo[62:0], ~trial[64]}) : $signed({quo[62:0], ~trial[64]});
        end
      end
    end
  end
endmodule

// ceil_sqrt(n) for n < 2^64 (core.cpp isqrt + ceil)
module isqrt_ceil (
  input  wire        clk,
  input  wire        rst,
  input  wire        start,
  input  wire [63:0] n,
  output reg         done,
  output reg  [32:0] r_ceil
);
  reg        busy, fin, fin2;
  reg [5:0]  i;
  reg [63:0] x, r, b, n0, rr_r;
  wire [64:0] rb = {1'b0, r} + {1'b0, b};
  wire [63:0] rr = r[31:0] * r[31:0];
  always @(posedge clk) begin
    if (rst) begin busy <= 1'b0; done <= 1'b0; fin <= 1'b0; fin2 <= 1'b0; end
    else begin
      done <= 1'b0;
      if (start) begin
        busy <= 1'b1; fin <= 1'b0; fin2 <= 1'b0; i <= 6'd0; x <= n; n0 <= n; r <= 64'd0; b <= 64'h4000000000000000;
      end else if (busy && !fin) begin
        if ({1'b0, x} >= rb) begin x <= x - rb[63:0]; r <= (r >> 1) + b; end
        else r <= r >> 1;
        b <= b >> 2;
        i <= i + 6'd1;
        if (i == 6'd31) fin <= 1'b1;
      end else if (busy && fin && !fin2) begin
        rr_r <= rr; fin2 <= 1'b1;
      end else if (busy && fin2) begin
        busy <= 1'b0; fin <= 1'b0; fin2 <= 1'b0; done <= 1'b1;
        r_ceil <= (rr_r == n0) ? {1'b0, r[31:0]} : {1'b0, r[31:0]} + 33'd1;
      end
    end
  end
endmodule
