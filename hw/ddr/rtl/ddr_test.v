// ddr_test: DDR self-test engine on the uart_bridge register bus (docs/DDR_PORT.md section 5); user of ddr_port.
// Data pattern = hash(word address, seed), so read-back is checked without stored copies.
// Registers (byte offsets, 32 bit):
//   0x000 CTRL     W: bit0 start (ignored while busy or before calibration)
//   0x004 STATUS   R: [0] calib_done [1] busy [2] pass [3] fail [4] axi_err (ddr_port err, sticky until reset)
//   0x008 BASE     start byte address (16 B aligned)      0x00C LEN   total beats (16 B each)
//   0x010 BURST    beats per command (0 = 1); MODE 4: number of reads
//   0x014 MODE     0 write+check, 1 write, 2 check, 3 strobe, 4 random single-beat reads (latency)
//   0x018 SEED     0x020 ERRS   0x024 ERR_ADDR   0x028 ERR_GOT   0x02C ERR_EXP (low 32 bits of the first bad beat)
//   0x030 WR_CYC   0x034 RD_CYC   0x038 LAT_MIN   0x03C LAT_MAX   0x040 LAT_SUM (MODE 4, cycles from command to data)
//   0x100 WIN_ADDR byte address of a single word      0x104 WIN_DATA W: write the word (4-byte strobe), R: read it
// MODE 3: write pattern A (SEED), write pattern B (~SEED) with strobes from a third hash, check every byte.
// MODE 4: read i at BASE + 16 * floor(h * LEN / 2^16), h = hw({i[8:0], i}, SEED) >> 16; data checked against SEED.
`timescale 1ns/1ps
module ddr_test (
  input  wire         clk,
  input  wire         rst,
  // register bus (uart_bridge)
  input  wire         bus_req,
  input  wire         bus_we,
  input  wire [23:0]  bus_addr,
  input  wire [31:0]  bus_wdata,
  output reg          bus_ack,
  output reg  [31:0]  bus_rdata,
  // ddr_port user side
  input  wire         calib_done,
  input  wire         port_err,
  output wire         wr_cmd_valid,
  input  wire         wr_cmd_ready,
  output wire [26:0]  wr_cmd_addr,
  output wire [15:0]  wr_cmd_len,
  output wire         wr_valid,
  input  wire         wr_ready,
  output wire [127:0] wr_data,
  output wire [15:0]  wr_strb,
  input  wire         wr_done,
  output wire         rd_cmd_valid,
  input  wire         rd_cmd_ready,
  output wire [26:0]  rd_cmd_addr,
  output wire [15:0]  rd_cmd_len,
  input  wire         rd_valid,
  output wire         rd_ready,
  input  wire [127:0] rd_data,
  /* verilator lint_off UNUSEDSIGNAL */
  input  wire         rd_last,
  /* verilator lint_on UNUSEDSIGNAL */
  // status for LEDs
  output wire         busy,
  output reg          pass,
  output reg          fail
);
  // ---- pattern ----
  // One 32-bit word of the pattern: xorshift of (word address ^ seed), plus a seed constant, then a fold.
  function [31:0] hw(input [24:0] widx, input [31:0] s, input [31:0] sk);
    reg [31:0] x;
    begin
      x = {7'd0, widx} ^ s;
      x = x ^ (x << 13); x = x ^ (x >> 17); x = x ^ (x << 5);
      x = x + sk;
      hw = x ^ (x >> 15);
    end
  endfunction
  function [127:0] pat(input [22:0] a, input [31:0] s, input [31:0] sk);   // a = byte address [26:4]
    pat = {hw({a, 2'd3}, s, sk), hw({a, 2'd2}, s, sk), hw({a, 2'd1}, s, sk), hw({a, 2'd0}, s, sk)};
  endfunction
  function [31:0] skf(input [31:0] s);
    skf = {s[15:0], s[31:16]} + 32'h9E37_79B9;
  endfunction

  // ---- registers ----
  reg [26:0] r_base;
  reg [23:0] r_len;
  reg [15:0] r_burst;
  reg [2:0]  r_mode;
  reg [31:0] r_seed;
  reg [31:0] errs, err_addr, err_got, err_exp, wr_cyc, rd_cyc, lat_min, lat_max, lat_sum;
  reg [26:0] win_addr;
  reg        done;

  // seeds of pattern A, pattern B, and the strobe hash, with their constants (fixed during a run)
  reg [31:0] sa, ska, sb, skb, ss, sks;

  // ---- state ----
  localparam [3:0] S_IDLE = 0, S_WR = 1, S_RD = 2, S_LH = 3, S_LM = 4, S_LC = 5, S_LW = 6,
                   S_WW = 7, S_WWD = 8, S_WR1 = 9, S_WRW = 10, S_DRAIN = 11, S_WD = 12, S_LA = 13;
  reg [3:0]  st;
  reg        ph_b;            // write phase with pattern B and strobes (MODE 3)
  assign busy = (st == S_WR) || (st == S_RD) || (st == S_DRAIN) || (st == S_LH) || (st == S_LM) ||
                (st == S_LA) || (st == S_LC) || (st == S_LW);

  wire [15:0] bl = (r_burst == 16'd0) ? 16'd1 : r_burst;

  // write engine
  reg [23:0] wc_left, wd_left;
  reg [26:0] wc_addr, wd_addr;
  reg [23:0] w_iss, w_dn;
  wire [15:0] wc_len = ({8'd0, bl} < wc_left) ? bl : wc_left[15:0];
  wire [127:0] pa_w = pat(wd_addr[26:4], ph_b ? sb : sa, ph_b ? skb : ska);
  /* verilator lint_off UNUSEDSIGNAL */
  wire [31:0]  st_w = hw({wd_addr[26:4], 2'd0}, ss, sks);
  /* verilator lint_on UNUSEDSIGNAL */

  // read engine and checker
  reg [23:0] rc_left, rk_left;
  reg [26:0] rc_addr, rk_addr;
  wire [15:0] rc_len = ({8'd0, bl} < rc_left) ? bl : rc_left[15:0];
  wire [127:0] pa_ra = pat(rk_addr[26:4], sa, ska);
  wire [127:0] pa_rb = pat(rk_addr[26:4], sb, skb);
  wire [31:0]  st_r  = hw({rk_addr[26:4], 2'd0}, ss, sks);
  reg  [127:0] exp_r;
  integer k;
  always @(*) begin
    for (k = 0; k < 16; k = k + 1)
      exp_r[8*k +: 8] = (r_mode == 3'd3 && st_r[k]) ? pa_rb[8*k +: 8] : pa_ra[8*k +: 8];
  end

  // single-word window
  reg [31:0] win_wd;

  // MODE 4
  reg [15:0] l_idx, l_h;
  /* verilator lint_off UNUSEDSIGNAL */
  reg [39:0] l_prod;
  wire [31:0] l_hash = hw({l_idx[8:0], l_idx}, sa, ska);   // index repeated: spreads the high bits for small i
  /* verilator lint_on UNUSEDSIGNAL */
  reg [31:0] l_cnt;

  // ---- ddr_port user side ----
  assign wr_cmd_valid = (st == S_WR && wc_left != 24'd0) || st == S_WW;
  assign wr_cmd_addr  = (st == S_WW) ? {win_addr[26:4], 4'd0} : wc_addr;
  assign wr_cmd_len   = (st == S_WW) ? 16'd1 : wc_len;
  assign wr_valid     = (st == S_WR && wd_left != 24'd0) || st == S_WWD;
  assign wr_data      = (st == S_WWD) ? {4{win_wd}} : pa_w;
  assign wr_strb      = (st == S_WWD) ? (16'h000F << {win_addr[3:2], 2'd0}) : (ph_b ? st_w[15:0] : 16'hFFFF);
  assign rd_cmd_valid = (st == S_RD && rc_left != 24'd0) || st == S_LC || st == S_WR1;
  assign rd_cmd_addr  = (st == S_WR1) ? {win_addr[26:4], 4'd0} : (st == S_LC) ? rk_addr : rc_addr;
  assign rd_cmd_len   = (st == S_RD) ? rc_len : 16'd1;
  assign rd_ready     = (st == S_RD) || (st == S_LW) || (st == S_WRW);
  wire wc_hs = wr_cmd_valid && wr_cmd_ready;
  wire wd_hs = wr_valid && wr_ready;
  wire rc_hs = rd_cmd_valid && rd_cmd_ready;
  wire rk_hs = rd_valid && rd_ready;

  // check pipeline: stage 1 holds the beat and its expected value, stage 2 compares
  reg          c_v;
  reg  [127:0] c_got, c_exp;
  reg  [26:0]  c_addr;
  wire         c_bad = c_v && (c_got != c_exp);

  // ---- bus decode ----
  /* verilator lint_off UNUSEDSIGNAL */
  wire [23:0] ba = bus_addr;
  /* verilator lint_on UNUSEDSIGNAL */
  wire [11:0] ra = ba[11:0];
  wire win_data = (ra == 12'h104);
  wire start_req = bus_req && bus_we && ra == 12'h000 && bus_wdata[0] && st == S_IDLE && calib_done;

  reg [31:0] bus_rdata_c;
  always @(*) begin
    case (ra)
      12'h004: bus_rdata_c = {27'd0, port_err, fail, pass, busy, calib_done};
      12'h008: bus_rdata_c = {5'd0, r_base};
      12'h00C: bus_rdata_c = {8'd0, r_len};
      12'h010: bus_rdata_c = {16'd0, r_burst};
      12'h014: bus_rdata_c = {29'd0, r_mode};
      12'h018: bus_rdata_c = r_seed;
      12'h020: bus_rdata_c = errs;
      12'h024: bus_rdata_c = err_addr;
      12'h028: bus_rdata_c = err_got;
      12'h02C: bus_rdata_c = err_exp;
      12'h030: bus_rdata_c = wr_cyc;
      12'h034: bus_rdata_c = rd_cyc;
      12'h038: bus_rdata_c = lat_min;
      12'h03C: bus_rdata_c = lat_max;
      12'h040: bus_rdata_c = lat_sum;
      12'h100: bus_rdata_c = {5'd0, win_addr};
      default: bus_rdata_c = 32'd0;
    endcase
  end

  always @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE; bus_ack <= 1'b0; done <= 1'b0; pass <= 1'b0; fail <= 1'b0; c_v <= 1'b0;
      r_base <= 27'd0; r_len <= 24'd0; r_burst <= 16'd16; r_mode <= 3'd0; r_seed <= 32'd1; win_addr <= 27'd0;
      errs <= 32'd0; err_addr <= 32'd0; err_got <= 32'd0; err_exp <= 32'd0;
      wr_cyc <= 32'd0; rd_cyc <= 32'd0; lat_min <= 32'hFFFF_FFFF; lat_max <= 32'd0; lat_sum <= 32'd0;
      wc_left <= 24'd0; wd_left <= 24'd0; rc_left <= 24'd0; rk_left <= 24'd0; ph_b <= 1'b0;
    end else begin
      bus_ack <= 1'b0;

      // register bus: plain registers answer next cycle; WIN_DATA answers when the DDR access is done
      if (bus_req) begin
        if (win_data && st == S_IDLE && calib_done) begin
          if (bus_we) begin win_wd <= bus_wdata; st <= S_WW; end
          else st <= S_WR1;
        end else begin
          bus_ack <= 1'b1;
          bus_rdata <= bus_rdata_c;
          if (bus_we && !busy) begin
            case (ra)
              12'h008: r_base   <= {bus_wdata[26:4], 4'd0};
              12'h00C: r_len    <= bus_wdata[23:0];
              12'h010: r_burst  <= bus_wdata[15:0];
              12'h014: r_mode   <= bus_wdata[2:0];
              12'h018: r_seed   <= bus_wdata;
              12'h100: win_addr <= {bus_wdata[26:2], 2'd0};
              default: ;
            endcase
          end
        end
      end

      // stage 2: compare
      if (c_bad) begin
        errs <= errs + 32'd1;
        if (errs == 32'd0) begin err_addr <= {5'd0, c_addr}; err_got <= c_got[31:0]; err_exp <= c_exp[31:0]; end
      end
      c_v <= 1'b0;

      case (st)
        S_IDLE: if (start_req) begin
          done <= 1'b0; pass <= 1'b0; fail <= 1'b0;
          errs <= 32'd0; err_addr <= 32'd0; err_got <= 32'd0; err_exp <= 32'd0;
          wr_cyc <= 32'd0; rd_cyc <= 32'd0; lat_min <= 32'hFFFF_FFFF; lat_max <= 32'd0; lat_sum <= 32'd0;
          sa <= r_seed; ska <= skf(r_seed); sb <= ~r_seed; skb <= skf(~r_seed);
          ss <= r_seed ^ 32'h5A5A_A5A5; sks <= skf(r_seed ^ 32'h5A5A_A5A5);
          wc_left <= r_len; wd_left <= r_len; wc_addr <= r_base; wd_addr <= r_base; w_iss <= 24'd0; w_dn <= 24'd0;
          rc_left <= r_len; rk_left <= r_len; rc_addr <= r_base; rk_addr <= r_base;
          ph_b <= 1'b0; l_idx <= 16'd0;
          case (r_mode)
            3'd2:    st <= S_RD;
            3'd4:    st <= S_LH;
            default: st <= S_WR;
          endcase
        end

        S_WR: begin
          wr_cyc <= wr_cyc + 32'd1;
          if (wc_hs) begin wc_left <= wc_left - {8'd0, wc_len}; wc_addr <= wc_addr + {7'd0, wc_len, 4'd0}; end
          if (wd_hs) begin wd_left <= wd_left - 24'd1; wd_addr <= wd_addr + 27'd16; end
          w_iss <= w_iss + {23'd0, wc_hs};
          w_dn  <= w_dn + {23'd0, wr_done};
          if (wc_left == 24'd0 && wd_left == 24'd0 && w_iss == w_dn) begin
            if (r_mode == 3'd3 && !ph_b) begin
              ph_b <= 1'b1;
              wc_left <= r_len; wd_left <= r_len; wc_addr <= r_base; wd_addr <= r_base; w_iss <= 24'd0; w_dn <= 24'd0;
            end else if (r_mode == 3'd1) st <= S_DRAIN;
            else st <= S_RD;
          end
        end

        S_RD: begin
          rd_cyc <= rd_cyc + 32'd1;
          if (rc_hs) begin rc_left <= rc_left - {8'd0, rc_len}; rc_addr <= rc_addr + {7'd0, rc_len, 4'd0}; end
          if (rk_hs) begin
            c_v <= 1'b1; c_got <= rd_data; c_exp <= exp_r; c_addr <= rk_addr;
            rk_left <= rk_left - 24'd1; rk_addr <= rk_addr + 27'd16;
          end
          if (rc_left == 24'd0 && rk_left == 24'd0) st <= S_DRAIN;
        end

        // MODE 4: hash -> multiply -> command -> wait
        S_LH: begin
          l_h <= l_hash[31:16];
          st <= S_LM;
        end
        S_LM: begin
          l_prod <= {24'd0, l_h} * {16'd0, r_len};
          st <= S_LA;
        end
        S_LA: begin
          rk_addr <= r_base + {l_prod[38:16], 4'd0};
          st <= S_LC;
        end
        S_LC: if (rc_hs) begin l_cnt <= 32'd1; st <= S_LW; end
        S_LW: begin
          l_cnt <= l_cnt + 32'd1;
          if (rk_hs) begin
            c_v <= 1'b1; c_got <= rd_data; c_exp <= exp_r; c_addr <= rk_addr;
            if (l_cnt < lat_min) lat_min <= l_cnt;
            if (l_cnt > lat_max) lat_max <= l_cnt;
            lat_sum <= lat_sum + l_cnt;
            l_idx <= l_idx + 16'd1;
            st <= (l_idx + 16'd1 >= bl) ? S_DRAIN : S_LH;
          end
        end

        // wait until the last compare has updated errs
        S_DRAIN: if (!c_v) begin
          st <= S_IDLE; done <= 1'b1;
          pass <= errs == 32'd0 && !port_err;
          fail <= errs != 32'd0 || port_err;
        end

        // single-word window: write
        S_WW:  if (wc_hs) st <= S_WWD;
        S_WWD: if (wd_hs) st <= S_WD;
        S_WD:  if (wr_done) begin bus_ack <= 1'b1; bus_rdata <= 32'd0; st <= S_IDLE; end
        // single-word window: read
        S_WR1: if (rc_hs) st <= S_WRW;
        S_WRW: if (rk_hs) begin
          bus_ack <= 1'b1; bus_rdata <= rd_data[32*win_addr[3:2] +: 32]; st <= S_IDLE;
        end
        default: st <= S_IDLE;
      endcase
    end
  end

  /* verilator lint_off UNUSEDSIGNAL */
  wire unused_ok = &{1'b0, done};
  /* verilator lint_on UNUSEDSIGNAL */
endmodule
