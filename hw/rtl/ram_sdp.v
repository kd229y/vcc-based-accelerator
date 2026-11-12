// Simple dual-port RAM: one synchronous write port, one synchronous read port
// (1-cycle read latency, read-old-data on same-address collision).
// Infers BRAM (or URAM on UltraScale+ when RAM_STYLE="ultra") in Vivado.
`timescale 1ns/1ps
module ram_sdp #(
  parameter DW = 32,
  parameter AW = 10,
  parameter DEPTH = (1 << AW),
  parameter RAM_STYLE = "block"
) (
  input  wire          clk,
  input  wire          we,
  input  wire [AW-1:0] waddr,
  input  wire [DW-1:0] wdata,
  input  wire [AW-1:0] raddr,
  output reg  [DW-1:0] rdata
);
  (* ram_style = RAM_STYLE *) reg [DW-1:0] mem [0:DEPTH-1];
  always @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
  end
endmodule
