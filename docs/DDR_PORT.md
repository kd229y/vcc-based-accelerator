# DDR 移植設計

狀態：L1–L4 全部通過（第 12 節），2026-11-25。等使用者同意後 merge 回 `feature/ple-cse-v2`。branch `feature/ddr-port`（從 `feature/ple-cse-v2` 開出）。

## 1. 目標和範圍

- 把 SoC（`CUSTOM_SoC_Platform`）裡已在板上用過的 DDR2 控制器移植到本 repo 的獨立 Nexys A7-100T 設計。
- 實作一個加速器之後要用的 DDR 存取介面 `ddr_port`，**讀和寫都要有**。
- 用一個測試引擎 `ddr_test`（經過現有的 `uart_bridge`）證明 DDR 能跑而且資料正確。**這一步不接加速器本體**（PLE-CSE、建表引擎）。
- 研究要不要替 DDR 加 cache（第 8 節）。
- 不做：改 SoC 的 BD、決定 SoC 的 address map、DMA 進 `cse_env_mem`、cache 一致性的軟體流程。

## 2. 移植的來源：SoC 的 MIG 設定

來源：`CUSTOM_SoC_Platform/Platform/hw/srcs/sources_1/bd/MMIO/ip/MMIO_mig_7series_0_0/mig_a.prj`，SoC 已在板上使用這份設定。

| 項目 | 值 |
|---|---|
| IP | MIG 7 Series 4.2（Vivado 2024.1 內有 `mig_7series_v4_2`） |
| 記憶體 | DDR2 `MT47H64M16HR-25E`，16-bit，128 MB（row 13、col 10、bank 3） |
| 記憶體時脈 | tCK 3333 ps（300 MHz），PHY 4:1，所以 **ui_clk = 75 MHz** |
| 輸入時脈 | sys_clk 300 MHz、ref_clk 200 MHz，都是 No Buffer（由 FPGA 內部的 MMCM 給） |
| 使用者介面 | AXI4：資料 128 bit、位址 27 bit、ID 4 bit、不支援 narrow burst、讀寫仲裁 `RD_PRI_REG` |
| reset | `sys_rst` active low |
| 其他 | XADC 溫度補償開啟；位址排列 BANK_ROW_COLUMN；Strict ordering |

DDR2 每次 BL8 傳 8 × 16 bit = 128 bit，剛好是一個 AXI beat。

## 3. 架構

```
               Nexys A7-100T（獨立設計，non-project flow）
 clk 100 MHz ──▶ MMCME2_BASE ──┬─ 300 MHz ─▶ sys_clk_i ┐
 (E3)                           └─ 200 MHz ─▶ clk_ref_i ┤
                                                        │   MIG（mig_ddr2）
                                                        ├──▶ ddr2_* 腳位 ──▶ DDR2 晶片
              ui_clk 75 MHz（整個設計的系統時脈） ◀──────┤
                                                        │ S_AXI（AXI4，128 bit）
 uart_rx/tx ─▶ uart_bridge ─▶ ddr_test ─▶ ddr_port ───┘
               （現有）        （新增）      （新增）
```

- **系統時脈直接用 MIG 的 ui_clk（75 MHz）**，和 SoC 相同。PLE-CSE 本來就收斂在 75 MHz，之後接上加速器不用跨時脈。
- `ddr_port` 是**之後加速器要用的介面**；`ddr_test` 只是測試用的使用者，之後會換成加速器的 DMA。
- 放進 SoC 時，`ddr_port` 的 AXI4 master 改接到 `smartconnect_0` 新的 SI（NUM_SI 3 → 4），用參數 `AXI_BASE` 加上 DDR 在 SoC 的位址 `0x6000_0000`。介面不用改。

## 4. 存取介面 `ddr_port`

### 4.1 使用者端（加速器看到的介面，全部在 ui_clk）

讀和寫是**兩組獨立的命令通道**，可以同時進行（例：一邊串流讀點雲，一邊寫回結果）。

| 訊號 | 方向 | 說明 |
|---|---|---|
| `wr_cmd_valid/ready`、`wr_cmd_addr[26:0]`、`wr_cmd_len[15:0]` | in/out | 寫命令：起始 byte 位址（16 B 對齊）、beat 數（1–65535） |
| `wr_valid/ready`、`wr_data[127:0]`、`wr_strb[15:0]` | in/out | 寫資料，每個 beat 16 B，`wr_strb` 是 byte enable |
| `wr_done` | out | 一個寫命令的所有 B 回應都收到時，送一拍 pulse |
| `rd_cmd_valid/ready`、`rd_cmd_addr[26:0]`、`rd_cmd_len[15:0]` | in/out | 讀命令 |
| `rd_valid/ready`、`rd_data[127:0]`、`rd_last` | out/in | 讀資料，依命令順序回來；`rd_last` 標記一個命令的最後一個 beat |
| `err` | out | sticky：收到 SLVERR / DECERR，或收到沒對齊的命令 |
| `calib_done` | in | MIG 的 `init_calib_complete`；拉高前 `*_cmd_ready` 一律為 0 |

規則：
- 位址的單位是 byte，必須 16 B 對齊；長度的單位是 beat。不支援 narrow 傳輸，部分寫入要用 `wr_strb`。
- 回應依序：每個方向只用一個 AXI ID（0），所以不需要 reorder buffer。
- 讀和寫之間**不保證順序**：同一個位址先寫後讀，要等 `wr_done` 之後再送讀命令。
- 每個方向一次只處理一個命令：寫命令在 `wr_done` 之後、讀命令在最後一個 beat 被取走之後，`*_cmd_ready` 才再拉高。
- 錯誤的命令（位址沒有 16 B 對齊，或 len = 0）：設定 `err`。位址會向下對齊後照常執行；len = 0 的寫命令立刻送出 `wr_done`，len = 0 的讀命令沒有任何 beat。

### 4.2 AXI4 端（接 MIG，或之後接 smartconnect）

- 一個長命令會切成多個 INCR burst：每個 burst 最多 `BURST_MAX` 個 beat（參數，預設 16，上限 256），而且**不跨 4 KB 邊界**（AXI 規定）。
- `AxSIZE` = 4（16 B），`AxBURST` = INCR，`AxCACHE`、`AxPROT`、`AxQOS` 給固定值。
- 讀：最多 `RD_OUTSTANDING` 個 burst 在途中（參數，預設 4），用來蓋掉 DDR 延遲。只有讀資料 FIFO 的空位足夠放下所有在途的 beat 時，才送出新的 AR。這樣 **R channel 不會被這個 master 卡住**；放進 SoC 後，也不會擋到 CPU 的讀取。
- 寫：W FIFO 裡已經有一整個 burst 的資料時，才送出 AW，避免 burst 送到一半停住、佔住 DDR。
- FIFO 預設用 LUTRAM（讀 32 beat、寫 16 beat），不用 BRAM。

## 5. 測試引擎 `ddr_test`（經 UART 存取）

資料樣式由位址和 seed 算出來（雜湊），所以檢查讀回資料時不需要另外存期望值，可以掃完整的 128 MB。

| 位址 | 名稱 | 說明 |
|---|---|---|
| 0x000 | CTRL | W：bit0 開始 BIST |
| 0x004 | STATUS | R：[0] calib_done [1] busy [2] pass [3] fail [4] axi_err（`ddr_port` 的 `err`，reset 前一直保持） |
| 0x008 | BASE | 起始 byte 位址 |
| 0x00C | LEN | 總 beat 數 |
| 0x010 | BURST | 每個命令的 beat 數（用來掃 1、4、16、64、256；0 當作 1）；MODE 4 時是讀取的次數 |
| 0x014 | MODE | 0 寫再讀回比對；1 只寫；2 只讀並比對；3 byte-strobe 測試；4 隨機單一 beat 讀（量延遲） |
| 0x018 | SEED | 樣式的 seed |
| 0x020 | ERRS | 錯誤的 beat 數 |
| 0x024 | ERR_ADDR | 第一個錯誤的位址 |
| 0x028/0x02C | ERR_GOT / ERR_EXP | 第一個錯誤 beat 的低 32 bit |
| 0x030/0x034 | WR_CYC / RD_CYC | 寫、讀階段的 cycle 數（算頻寬） |
| 0x038/0x03C/0x040 | LAT_MIN / LAT_MAX / LAT_SUM | MODE 4 的讀取延遲（cycle） |
| 0x100 | WIN_ADDR | 單一 word 存取的位址 |
| 0x104 | WIN_DATA | 寫：寫入一個 32-bit word（strobe 只開 4 byte）；讀：讀回那個 word |

MODE 3（strobe）：先整段寫入樣式 A，再用隨機 strobe 寫入樣式 B，最後讀回，檢查每個 byte 是 A 還是 B 都正確。

實作細節（`hw/ddr/rtl/ddr_test.v` 檔頭；`hw/ddr/host/ddr_host.py` 的 `hw()` 是同一個公式）：
- 樣式：位址 a（byte，4 B 對齊）的 32-bit word = `hw(a >> 2, seed)`。`hw` 是 xorshift(13, 17, 5)，加上 `rot16(seed) + 0x9E3779B9`，最後 `x ^ (x >> 15)`。樣式 A 用 SEED，樣式 B 用 ~SEED，strobe = `hw(a >> 2, SEED ^ 0x5A5AA5A5)` 的低 16 bit（a 是該 beat 的起始位址）。
- MODE 4：第 i 次讀取的位址 = BASE + 16 × floor(h × LEN / 2^16)，h = `hw({i[8:0], i}, SEED)` 的高 16 bit（index 重複一次，i 小的時候位址才會分散；只用 `hw(i)` 時 200 次讀取只落在 10 個位址）。資料用 SEED 的樣式 A 比對，所以要先用同一個 SEED 寫過那一段。延遲 = 讀命令被接受到資料出現的 cycle 數（包含 `ddr_port` 本身約 4 cycle）。
- UART 用 115200 baud（`ddr_uart_top` 參數 `BAUD`）。

## 6. 要改和新增的東西

| 類別 | 內容 |
|---|---|
| **BD** | **不改**。本 repo 用 non-project 的 tcl flow，沒有 BD；SoC 的 BD 也不動 |
| **IP** | 新增 1 個：MIG 7 Series 4.2，build 時由 `create_ip` 加上 `mig_ddr2.prj` 產生（`ModuleName` 改成 `mig_ddr2`，其他設定和 SoC 相同）。時脈用 `MMCME2_BASE` primitive，不用 clk_wiz IP |
| **位置** | 全部放在我們自己的新目錄 `hw/ddr/`，不放進組員的 `hw/rtl`、`hw/tb`、`hw/vivado`、`hw/host`，也不改組員的 Makefile target 和 scripts。`uart_bridge.v` 只是被 instantiate，不修改 |
| **新增 RTL** | `hw/ddr/rtl/ddr_port.v`（介面 + AXI4 master）、`hw/ddr/rtl/ddr_fifo.v`、`hw/ddr/rtl/ddr_test.v`、`hw/ddr/rtl/ddr_uart_top.v`（板子頂層） |
| **新增 build** | `hw/ddr/vivado/mig_ddr2.prj`、`hw/ddr/vivado/build_ddr.tcl`、`hw/ddr/vivado/ddr_top.xdc`（只有時脈、按鈕、UART、LED；DDR2 腳位由 MIG 的 xdc 負責）、`hw/ddr/vivado/program.tcl`（照 SoC 的 `Platform/hw/program.tcl`，WSL2 + localhost:3121）。輸出到 `build/ddr/` |
| **新增 tb** | `hw/ddr/tb/axi_mem_model.sv`、`hw/ddr/tb/tb_ddr_port.sv`（Verilator）、`hw/ddr/tb/tb_ddr_mig.sv`（xsim，真的 MIG 加上 Micron DDR2 模型） |
| **新增 host** | `hw/ddr/host/ddr_host.py`：上板測試，最後印出 PASS / FAIL，並把結果寫進 log 檔。由使用者在自己的 WSL 終端機執行（agent 的非互動 shell 讀不到 UART） |
| **Makefile** | 新的 `hw/ddr/Makefile`：`test`、`sim-mig`、`build`、`program`。`test` 加進我們的 `hw/ple-cse/scripts/sim_regress.sh`（`make regress`） |

## 7. 驗證計畫

| 層級 | 內容 | PASS 條件 |
|---|---|---|
| L1 Verilator | `ddr_port` + `ddr_test` 接 AXI4 記憶體模型。模型遵守 byte strobe；記憶體初值填非零；AWREADY / WREADY / ARREADY / RVALID 加上隨機間隔（固定 seed），延遲掃最小、預設、最大。協定檢查：valid 在 ready 前不能變、WLAST 位置、len ≤ 256、不跨 4 KB、size = 16 B。每個讀回的 beat 都和 scoreboard 比對 | 每個 MODE 和每種 burst 長度的錯誤都是 0；協定檢查沒有違規 |
| L1 mutant | 植入的錯誤：WLAST 早一拍、跨 4 KB、忽略 strobe、讀資料 FIFO 少一格 | 每個 mutant 都要 FAIL |
| L2 xsim | `ddr_test` + `ddr_port` + 真的 MIG RTL + Micron `ddr2_model.v`（Vivado 內附），`SIM_BYPASS_INIT_CAL = FAST` | calib 完成；小範圍 BIST 的 MODE 0 和 3 錯誤為 0 |
| L3 Vivado | `build_ddr.tcl` 跑完 impl 和 bitstream | ui_clk 和 MIG 內部的時脈 WNS ≥ 0、WHS ≥ 0；有 MIG 的 utilization 數字 |
| L4 上板（使用者執行） | WSL2：`make -C hw/ddr program` 燒錄 → 使用者在自己的終端機跑 `ddr_host.py`（`/dev/ttyUSB1`）：calib → 128 MB 全範圍 MODE 0 → MODE 3 → burst 長度掃描 → MODE 4 延遲；agent 讀它寫出的 log | 每一項錯誤都是 0；記下頻寬和延遲 |

模擬盲點：L1 的記憶體模型是理想的；L2 用 FAST 校準，不模擬板上的走線延遲；refresh 和溫度漂移只有板上才看得到。所以 L4 要跑全範圍，並且重複多輪。

## 8. 要不要替 DDR 加 cache（研究）

**結論（L4 確認）：不加。** 改用 burst 加 FIFO（多個讀 burst 在途中、寫入緩衝）。L4 量到：單一 beat 隨機讀平均延遲 L = 26 cycle（347 ns），長命令的串流頻寬寫 899 MB/s、讀 870 MB/s（峰值 1.2 GB/s 的 75% / 72%）。目前規劃的使用者都是串流，所以瓶頸在命令長度，不在命中率（見第 12 節的 burst 掃描）。

| 加速器會做的 DDR 存取 | 存取樣式 | 會重複使用嗎 | cache 有幫助嗎 |
|---|---|---|---|
| 點雲輸入（建表引擎） | 循序、讀一次 | 不會 | 沒有；要的是預取，`RD_OUTSTANDING` 已經提供 |
| 結果、路徑寫回 | 循序、寫一次 | 不會 | 沒有；寫入緩衝就夠 |
| 把預先建好的環境表複製進 `cse_env_mem` | 循序 | 不會 | 沒有 |
| 樹或環境表直接放在 DDR | 隨機、重複使用 | 會 | 有，但前面的分析已決定這兩者留在 BRAM |

不加的理由：
1. **沒有會重複使用的存取**：目前規劃的使用者都是讀一次或寫一次的串流，cache 命中率接近 0。
2. **LUT 是瓶頸**：cache 要 tag、資料 BRAM 和控制邏輯，估計會多 1k–2k LUT（估計值）。
3. **一致性**：SoC 的 dcache 是 write-back，而且沒有硬體一致性。加速器再加一個 cache，就變成兩個互不知道的 cache，CPU 和加速器兩邊都要 flush 或 invalidate，軟體流程更容易出錯。不加 cache 的話，只需要 CPU 在啟動前執行 `fence.i`（SoC 已實作）。

什麼時候要重新考慮：出現「隨機存取而且會重複使用」的使用者時，例如樹超過 1024 個節點、要把舊節點放到 DDR。判斷方式：用 L4 MODE 4 量到的單一 beat 平均延遲 L（= 26 cycle），如果那個使用者每 L 個 cycle 需要不只一次隨機存取，而且重複使用率高，再評估加一個小的唯讀 cache。

## 9. 風險

- MIG 的 xdc 依照 prj 裡的腳位名稱（`ddr2_*`）去 `get_ports`，所以頂層的 port 名稱必須和 prj 一致。
- MIG 開了 XADC 溫度補償，所以設計裡不能再有別的 XADC。
- L2 的 MIG 模擬很慢（實測整個 `tb_ddr_mig` 約 6 分鐘），不放進 `make regress`，改成另外執行（`make -C hw/ddr sim-mig`）。
- 板上測試只能由使用者執行（H3 交接），agent 看不到板子的輸出。

## 10. 已決定（2026-11-25，使用者確認）

1. 系統時脈直接用 MIG 的 ui_clk（75 MHz），不另外做跨時脈。
2. 讀和寫分成兩組獨立的命令通道（第 4.1 節）。
3. **等 L4 上板 PASS 之後**，才 merge 回 `feature/ple-cse-v2`。在那之前，所有 commit 都只留在 `feature/ddr-port`。

## 11. 實作順序（每一步都要通過才往下）

1. `ddr_fifo.v`、`ddr_port.v`，加上 `axi_mem_model.sv`、`tb_ddr_port.sv` → L1：每種 burst 長度、延遲最小 / 預設 / 最大，錯誤都是 0；協定檢查沒有違規。
2. L1 mutant：4 個植入的錯誤都要 FAIL。
3. `ddr_test.v` → 用 L1 的模型跑 MODE 0–4。
4. `mig_ddr2.prj`、`ddr_uart_top.v`、`build_ddr.tcl` → L3：合成、佈線、bitstream，WNS 和 WHS ≥ 0。
5. `tb_ddr_mig.sv` → L2：真的 MIG 加上 Micron 模型。
6. `ddr_host.py`，再寫 H3 交接 → 使用者上板（L4）。
7. L4 PASS → 依 L4 的數字更新第 8 節（cache 結論）→ 使用者同意後 merge 回 `feature/ple-cse-v2`。

L3 排在 L2 之前：先確認 MIG 產生和 build 都沒問題，再花時間跑較慢的 MIG 模擬。

## 12. 實作結果（2026-11-25）

| 層級 | 指令 | 結果 |
|---|---|---|
| lint | `make -C hw/ddr lint` | rc=0；`ddr_port` 用 3 組參數（預設、BURST_MAX=256、BURST_MAX=1） |
| L1 | `make -C hw/ddr test-l1`（`tb/run_l1.sh`） | 5 組參數 × 4 組延遲（min、def、max、slow = 記憶體快而使用者慢）的 20 個 `TB_DDR_PORT` 都是 `errors=0 proto_errors=0`；`TB_DDR_TEST`（MODE 0–4、window、植入錯誤）3 組延遲都是 `errors=0` |
| L1 mutant | `make -C hw/ddr test-mutants` | 12/12 killed：文件的 4 個（WLAST 早一拍、跨 4 KB、忽略 strobe、讀 FIFO 少一格），加上讀 credit 多一格、wr_done 不等 B、AW 不扣已分配的 beat，和 `ddr_test` 的 5 個 |
| L2 | `make -C hw/ddr sim-mig`（xsim，整個 `ddr_uart_top` 經 UART） | `TB_DDR_MIG errors=0`；FAST 校準在 108.9 µs 完成；MODE 4 延遲 25–30 cycle |
| L3 | `make -C hw/ddr build` | WNS +1.405 ns、WHS +0.031 ns（都在 ui_clk），0 個 CRITICAL WARNING。第一版的最差路徑是 `ddr_port` 的 `ar_rem` → burst 長度計算 → `ar_addr`（18 級，+1.080 ns），重新佈線後最差路徑換到 MIG 內部。資源 6,937 LUT（MIG 3,944、`ddr_test` 1,932、`uart_bridge` 569、`ddr_port` 492）、0 BRAM、1 DSP |
| L4 | `make -C hw/ddr program` + `ddr_host.py --rounds 3`（Nexys A7-100T，JTAG 序號 210292…） | `DDR_HOST PASS`：3 輪全部 PASS（見下表），log `build/ddr/board/ddr_host_20261125_135804.log` |

注意：
- 「讀 FIFO 少一格」只有在「記憶體快、使用者慢」時才會出現，所以 L1 加了 `slow` 這組延遲；只用 min / def / max 時這個 mutant 會存活。
- `make regress` 包含 `make -C hw/ddr test`（lint + L1，約 45 秒），不含 mutant 和 L2。

### L4 板上數字（3 輪結果一致，下表取第 0 輪；峰值 = 16 B × 75 MHz = 1.2 GB/s）

| 測試 | 寫 MB/s | 讀 MB/s | cycle/beat（寫 / 讀） |
|---|---|---|---|
| MODE 0，128 MB，命令 256 beat | 899（75%） | 870（72%） | 1.33 / 1.38 |
| MODE 0，1 MB，命令 1 beat | 169 | 44 | 7.1 / 27.3 |
| MODE 0，1 MB，命令 4 beat | 362 | 158 | 3.3 / 7.6 |
| MODE 0，1 MB，命令 16 beat | 498 | 447 | 2.4 / 2.7 |
| MODE 0，1 MB，命令 64 beat | 790 | 738 | 1.5 / 1.6 |
| MODE 0，1 MB，命令 256 beat | 909 | 846 | 1.3 / 1.4 |
| MODE 3 strobe，8 MB | 784（log 顯示 392，見下） | 761 | |
| MODE 4 隨機讀 10,000 次（全 128 MB） | | | 延遲最小 26、最大 30 / 28 / 36、平均 26.0 cycle |

其他：每輪的 window word 8/8 正確；用 window 讀回的 word 等於 host 的樣式公式（RTL 和 host 的公式一致）；`errs=0`，沒有 AXI 錯誤。

判讀：
- 和模擬一致：L2 的 MODE 4 延遲 25–30 cycle（平均 26），板上 26–36（平均 26.0）。
- **短命令的讀取被延遲卡住**：命令 1 beat 時每 beat 27.3 cycle ≈ 延遲 26 + 1；命令 4 beat 時 (26 + 4) / 4 ≈ 7.5，和量到的 7.6 一致。原因是 `ddr_port` 每個方向一次只處理一個命令（第 4.1 節），下一個命令要等上一個做完，延遲沒辦法在命令之間重疊。**加速器應該用 ≥ 64 beat（1 KB）的命令**；如果之後的使用者只能發短命令，要改成讓下一個命令的 AR 提早送出（命令佇列），不是加 cache。
- MODE 3 的寫頻寬：WR_CYC 包含樣式 A 和 B 兩次寫入，host 原本只用 8 MB 算，所以 log 顯示 392 MB/s，實際約 784 MB/s。`ddr_host.py` 已修正（只影響顯示，不影響 PASS/FAIL）。
- 第一次上板測試時接錯成 Arty A7-100T（同一顆 xc7a100t，所以燒錄也會成功），結果 UART 0 bytes、LED 全暗。燒錄後要看 `program.log` 的 JTAG 序號：Nexys A7 是 `210292…`，Arty 是 `210319…`。

