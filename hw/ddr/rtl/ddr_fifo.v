// ddr_fifo: synchronous FIFO, LUTRAM storage, first-word fall-through (dout valid while !empty).
`timescale 1ns/1ps
module ddr_fifo #(
  parameter W     = 128,
  parameter DEPTH = 32,                    // any value >= 2
  parameter CW    = $clog2(DEPTH + 1)      // count width
) (
  input  wire          clk,
  input  wire          rst,
  input  wire          push,
  input  wire [W-1:0]  din,
  input  wire          pop,
  output wire [W-1:0]  dout,
  output wire          empty,
  output wire          full,
  output reg  [CW-1:0] count
);
  localparam AW = (DEPTH > 1) ? $clog2(DEPTH) : 1;
  localparam integer LAST_I = DEPTH - 1;
  localparam integer FULL_I = DEPTH;
  localparam [AW-1:0] LAST = LAST_I[AW-1:0];
  localparam [CW-1:0] FULLC = FULL_I[CW-1:0];
  (* ram_style = "distributed" *) reg [W-1:0] mem [0:DEPTH-1];
  reg [AW-1:0] wp, rp;
  wire do_push = push && !full;
  wire do_pop  = pop && !empty;

  assign empty = (count == 0);
  assign full  = (count == FULLC);
  assign dout  = mem[rp];

  always @(posedge clk) begin
    if (do_push) mem[wp] <= din;
  end

  always @(posedge clk) begin
    if (rst) begin
      wp <= 0; rp <= 0; count <= 0;
    end else begin
      if (do_push) wp <= (wp == LAST) ? {AW{1'b0}} : wp + 1'b1;
      if (do_pop)  rp <= (rp == LAST) ? {AW{1'b0}} : rp + 1'b1;
      if (do_push && !do_pop) count <= count + 1'b1;
      else if (!do_push && do_pop) count <= count - 1'b1;
    end
  end

`ifndef SYNTHESIS
  // A push into a full FIFO loses data; the users must never do it.
  always @(posedge clk) if (!rst && push && full) $display("ERROR ddr_fifo %m: push while full");
`endif
endmodule
