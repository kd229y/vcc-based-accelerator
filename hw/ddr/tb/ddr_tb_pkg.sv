// Shared helpers of the DDR testbenches: initial memory pattern and a small PRNG.
`timescale 1ns/1ps
package ddr_tb_pkg;
  // Non-zero initial content of beat index i (byte address >> 4).
  function automatic logic [127:0] init_beat(input logic [31:0] i);
    logic [127:0] v;
    for (int k = 0; k < 4; k++) begin
      logic [31:0] x = (i * 4 + k) ^ 32'hA5A5_5A5A;
      x ^= x << 13; x ^= x >> 17; x ^= x << 5;
      v[32*k +: 32] = x | 32'h0100_0001;   // never all-zero
    end
    return v;
  endfunction

  // xorshift32 step (state must stay non-zero).
  function automatic logic [31:0] xs32(input logic [31:0] s);
    logic [31:0] x = s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return x;
  endfunction
endpackage
