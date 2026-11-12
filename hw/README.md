# VCC Morton 加速器 — RTL 實作

依 `spec.md`（第 3、6 章）與 `SoC_軟體驗證資料包`（`soc_model/core.cpp` 整數功能模型、`spec/design_v1.json` 原方案設定）寫成的 Verilog 硬體。
**所有引擎都以 core.cpp 為 golden，逐 bit 比對通過**（見「驗證結果」）。

## 1. 內容一覽

```
hw/
├─ rtl/                     可合成 Verilog-2005（Vivado / iverilog 皆可）
│  ├─ stage_a.v             Stage A：fp32→Q14（向零截斷，6.2.4）、1.19 m 徑向剔除、Morton key、S=4 選點距離 d
│  ├─ build_engine.v        建構引擎（方案 A）：hash 邊讀邊挑 → prefix sum → counting sort → 格內排序 + AABB
│  ├─ query_engine.v        碰撞查詢：global AABB → 依實際半徑列舉粗格（非寫死 27 格）→ cell AABB → 球對點
│  ├─ fk_engine.v           定點 FK：30 輪整數 CORDIC、Q30 旋轉 / Q24 平移、59 顆球 → Q14
│  ├─ rrt_engine.v          RRT-Connect FSM：xorshift 取樣、全掃描 NN、整數 sqrt steering、VAMP rake 邊驗證、回溯
│  ├─ arith_units.v         round_div 除法器、ceil-sqrt
│  ├─ panda_consts.v        由 build/panda_generated.hh 自動產生（URDF/SRDF 常數、690 自碰撞對、關節限制）
│  ├─ vcc_core.v            以上全部 + 32-bit 暫存器匯流排 + AXI-Stream 點雲輸入
│  ├─ uart_bridge.v         UART ↔ 匯流排（任何有 USB-UART 的板子）
│  └─ vcc_top.v             vcc_uart_top（Arty/Nexys…）、vcc_axil_top（Zynq：AXI4-Lite + AXI-Stream + IRQ）
├─ tb/                      testbench（build/query、FK、整顆 core 規劃、UART 端到端）
├─ golden/                  gen_golden.cpp（直接 #include 原封不動的 core.cpp）、make_vectors.py、gen_consts.py
├─ vectors/                 已產生的測試向量（7 個建構場景、328 組 FK、4 組規劃）
├─ sim/run_all.sh           一鍵回歸
├─ vivado/                  build_uart.tcl、build_zynq.tcl、program.tcl、xdc/
└─ host/vcc_host.py         上板後的 driver + 自我測試（UART 或 Zynq MMIO）
```

## 2. 架構與 spec 對照

| spec | 硬體 | 備註 |
|---|---|---|
| 3.3–3.5 剔除、量化、邊讀邊挑 | `stage_a` + `build_engine` FILTER | 2 級 pipeline；嚴格小於才取代、平手保留先到；hash = key·2654435761 mod C、linear probing |
| 3.5 「滿了必須回報」 | `status=4 CAPACITY_EXCEEDED` | 規則與 golden 相同：kept·4 ≥ 3C；另有 `cell_overflow` 旗標 |
| 3.6–3.7 排序、建表 | PREFIX → SCATTER → CELLFIX | 粗格 counting sort（32768 桶 = offset 表）＋格內依 key 低 6 bit 排（64-slot scratch），結果等同 golden 的 3×7-bit radix sort |
| 3.8 / 6.4 查詢 | `query_engine` | dense cell-id 表、clamp+距離 cell 剔除（精確版）、r = radius + 41 + 2 |
| 6.2.2 外層包圍球 | 範圍依 ⌈2R/L⌉+1 計算 | 不寫死 3×3×3，半徑最大 65535 LSB 都正確 |
| 6.5.2–6.5.4 RRT + 定點 FK | `rrt_engine` + `fk_engine` | 主迴圈為 FSM（spec 建議），控制完全在晶片上 |
| 6.1.1 主機介面 | `vcc_axil_top`（AXI-Lite + AXI-Stream + IRQ）/ `vcc_uart_top` | 點雲可走 AXI-Stream（接 AXI DMA） |

FK 與查詢引擎由 host 與規劃器共用；host 也可單獨下 FK、單一查詢。

## 3. 驗證結果（iverilog，對 core.cpp 逐 bit 比對）

| 測試 | 內容 | 結果 | 硬體 cycles |
|---|---|---|---|
| s0_pkgvec | 資料包 rtl_vectors 原始點雲（10,000 點）+ 資料包自己的 512 筆查詢 | PASS | build 171K |
| s1_shelf | MotionBenchMaker bookshelf_thin 表面取樣 52,500 點 → 7,067 代表點 / 344 格 | PASS | build 377K |
| s2_edge | 數值邊界：±0、非正規數、±1e30、|x|=1.19/2.0、同格平手、重複點 | PASS | build 150K |
| s3_cap1024 | hash 容量溢位 → CAPACITY_EXCEEDED | PASS | |
| s4_inf | 中途出現 Inf → INVALID_INPUT | PASS | |
| s5_empty | 空點雲 | PASS | |
| s6_dense | 12,000 點實心球 → 11,671 代表點 / 3,457 格 | PASS | build 257K |
| fk_pkg / fk_rand | 資料包 128 組 + 200 組（含 0 與上下限） | PASS | 3,325 / FK |
| p1_free | 無障礙直線可達 | PASS | 52K |
| p0_detour | 資料包 synthetic_detour（13 iter、22 nodes、388 state、22,831 query） | PASS | 1.87M（18.7 ms） |
| p2_shelf_s1 / s3 | bookshelf_thin 第 1 題，seed 1 / 3（路徑 24 / 32 點） | PASS | 4.9M / 7.6M |
| uart | 經 UART 協定端到端：建構、burst 讀回、查詢、FK | PASS | |

比對項目：建構狀態、kept / cell 數、culled 數、global AABB、**每一個**代表點、全部 32769 個 offset、32768 個 cell id、每個 cell 的 start/count/AABB、每一筆查詢、每一顆 FK 球心；規劃則比對狀態、整條路徑（每個 waypoint 7 個 Q24 碼）以及 iterations / nodes / state 呼叫 / edge 呼叫 / query 次數，全部須完全一致。

自行重跑：

```bash
cd hw
python3 golden/make_vectors.py      # 重新產生向量（需要 g++、numpy）
sim/run_all.sh                       # 全部 testbench
```

## 3.1 實機結果（Nexys A7-100T，2026-11-10）

設定 `nexys`（HASH_AW=14、CELL_AW=12、MAXN_AW=9），100 MHz，timing 通過（WNS +0.183 ns），資源 BRAM 112/135、LUT 16%、DSP 33/240。
`python vcc_host.py --port COM4 selftest` → **SELFTEST PASS**，與 core.cpp 逐 bit 一致：

| 項目 | 結果 |
|---|---|
| 建表 s0 / s1_shelf / s2_edge / s5_empty / s6_dense | 全部代表點、offset、cell id、cell 一致（s6：11,671 點 / 3,457 格） |
| s4_inf | INVALID_INPUT，一致 |
| 查詢 | 850 筆全部一致 |
| FK | 20 組，0 個球心不一致（3,730 cycles / FK） |
| 規劃 p1_free / p0_detour / p2_shelf_s1 | 路徑與統計完全一致；硬體時間 0.56 / 20.3 / 53.0 ms |

註：build 的 `hw_cycles` 包含等待 UART 傳點雲的時間，不代表引擎本身速度（模擬中 s1_shelf 的引擎時間約 3.7 ms）。

## 4. 上板（Windows + Vivado）

### 4.1 選設定（BRAM 是主要限制）

| profile | HASH_AW | 最多代表點 | CELL_AW | 每棵樹節點 | BRAM36 估計 | 適用 |
|---|---|---|---|---|---|---|
| golden 完全相同 | 15 | 24,576 | 12 | 1,024 | ~190 | KV260（hash/點用 URAM）、Kintex、ZCU104 |
| **預設（FPGA）** | 14 | 12,288 | 12 | 512 | ~120 | **PYNQ-Z2 / Zybo Z7-20 / Arty A7-100T / Nexys A7-100T** |

容量只影響「何時回報 CAPACITY_EXCEEDED」，不影響結果。bookshelf 類場景約 7–9K 個代表點、樹數十個節點，在預設 profile 內與 golden 完全相同。

### 4.2 UART 板（Arty A7-100T / Nexys A7-100T）

```bat
cd hw\vivado
vivado -mode batch -source build_uart.tcl -tclargs arty      :: 或 nexys
vivado -mode batch -source program.tcl -tclargs out_arty\vcc_uart_top.bit
pip install pyserial numpy
cd ..\host
python vcc_host.py --port COM5 selftest
```

其他 7 系列板：`-tclargs custom <part> <你的.xdc>`，xdc 需要 `clk`（100 MHz）、`rst_btn`、`uart_rx`、`uart_tx`、`led[3:0]`。鮑率 921600（`vcc_uart_top` 參數 `BAUD`）。

### 4.3 Zynq 板（PYNQ-Z2 / Zybo Z7-20 / KV260）

```bat
vivado -mode batch -source build_zynq.tcl -tclargs pynq-z2     :: 或 zybo-z7-20、kv260
```

產生 `out_<board>/vcc_zynq.xsa / .bit / .hwh`。加速器位址：Zynq-7000 為 `0x40000000`，UltraScale+ 為 `0xA0000000`（16 MiB）。
PYNQ 上：把 `.bit/.hwh` 載入 Overlay 後執行 `sudo python3 vcc_host.py --mmio 0x40000000 selftest`（需同時拷貝 `hw/vectors`）。
Vitis 裸機：用 `Xil_Out32/Xil_In32` 依 `vcc_core.v` 開頭的暫存器表操作即可。

### 4.4 暫存器使用流程

1. `NPOINTS` ← 點數；`CTRL` ← 1（build）；把 x,y,z binary32 依序寫入 `PT_WORD`（或走 AXI-Stream）；輪詢 `STATUS[0]`。
2. 查詢：寫 `Q_XY`、`Q_ZR`（寫入即開始），讀 `Q_RESULT`。座標為 workspace 相對 u16 Q14（= 世界 Q14 + 32768）。
3. FK：寫 `Q0..Q6`（Q24），`CTRL` ← 4，從 `0x500000` 讀 59 顆球。
4. 規劃：`Q0..Q6` 起點、`G0..G6` 目標、`RANGE..MARGIN` 參數（reset 值即 design_v1.json），`CTRL` ← 8，輪詢 `STATUS[16]`，讀 `PATH_LEN` 與 `0x600000` 起的路徑。

## 5. 已知限制與尚未完成（請在決定下一步時參考）

- UART 流程已在 Nexys A7-100T 實際建置與驗證；Zynq 流程（build_zynq.tcl）尚未實際跑過。
- 接觸不良對策：UART bridge 有 ~84 ms 閒置逾時（半個命令會被丟棄）；`vcc_host.py --bit <bitfile>` 斷線會自動重連、斷電會自動重燒並重跑該項測試；`program.tcl` 燒錄失敗會重試 8 次。
- **吞吐量是「先正確」版**：Stage B 每點約 3–5 cycle（spec 目標每 cycle 1 點）；查詢引擎 P = 1 lane、N = 1 個；FK 約 3.3K cycle；規劃 detour 題約 17 ms @100 MHz。效能優化（多 lane、多查詢引擎、FK 平行化）屬 spec 7.4 的 M4/M6。
- 環境只有一份（未做雙緩衝）；輸入是 fp32 點雲（未做深度影像反投影 6.2.4(b)）。
- 建構採 **方案 A（hash）**，因為軟體 golden 就是方案 A；spec 6.3.7 建議的方案 B 兩層區塊尚未實作。
- 規劃器的 goal bias 支援任意間隔；speculative sphere batch（golden 預設關閉）未實作。
- 經 AXI-Lite / UART 送點雲很慢（spec 6.1.1 也提醒），正式量測請接 AXI DMA → `s_axis`。
