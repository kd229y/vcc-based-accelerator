# VCC-based Morton accelerator

以 Morton code 統一點雲濾波與碰撞結構建構，並把點雲前處理、碰撞查詢、FK 與 RRT-Connect 規劃放上晶片的整數加速器。
設計說明見 [`spec.md`](spec.md)；進度報告見 [`hw/REPORT_meeting.md`](hw/REPORT_meeting.md)。

## 分工與相關 repo

| 部分 | 負責 | 位置 |
|---|---|---|
| 點雲前處理：fp32→定點、剔除、Morton 濾波（邊讀邊挑）、建表 | 本 repo | `hw/rtl/stage_a_v0.v`、`hw/rtl/build_engine_v0.v`（整合版，接 PLE-CSE）；`stage_a.v`、`build_engine.v`（獨立版） |
| RRT-Connect 規劃（PLE）、碰撞查詢（CSE）、CPU | 協作者 | [SyncMeow/PLE-CSE](https://github.com/SyncMeow/PLE-CSE) |
| 軟體功能模型（golden）與驗證系統 | 本 repo | `SoC_軟體驗證資料包/` |

> **注意**：`hw/rtl/rrt_engine.v`、`hw/rtl/query_engine.v`、`hw/rtl/fk_engine.v` 是為了在 FPGA 上做端到端驗證而寫的**參考實作**。
> 正式的規劃與碰撞查詢以 PLE-CSE 為準。

### 與 PLE-CSE 的整合（已完成，實機驗證）

`hw/rtl/build_engine_v0.v` 直接輸出 PLE-CSE 的 environment format v0，寫進 `ple_cse_top` 的 `env_wr_*` 埠。
整合頂層 `hw/rtl/vcc_v0_top.sv`：UART → 我們的建表引擎 + PLE-CSE（N_CU = 1），Nexys A7-100T，75 MHz。

| 驗證 | 結果 |
|---|---|
| 建表 vs PLE-CSE `fx_env_build` | 9 題 + 邊界測試，HDR / DIR / CELL / PTS 逐 bit 一致（AABB 為往外取整後的值） |
| 整合模擬（Verilator）vs `fx_ref_solve` | **全部 700 題**一致（688 題成功、12 題 BUDGET，皆與 golden 相同） |
| Nexys A7 實機 | 9 題 SYSTEST PASS，規劃 cycles 與模擬完全相同 |
| 資源 / timing | BRAM 117.5/135、LUT 47%、DSP 58%；75 MHz WNS +0.775 ns |

建表引擎的設計：hash 濾波（只存 q，d 由座標重算）→ 每個查詢格 64-bit 佔用遮罩 → 依查詢格 Morton 順序配 slot、寫全部 DIR →
以遮罩 popcount 算格內名次，點直接寫到 PTS 最終位置（不另存點陣列）→ CELL → HDR。AABB 偏移存 8 bits、往外取整寫出。

**對 PLE-CSE 的修改**（`hw/patches/ple-cse-pts-offset.patch`，用 `make ple-patch` 套用到 `hw/ple-cse`）：
PTS 改存「相對查詢格原點的 11-bit 偏移」（48 → 33 bits，無損），CSE S3 讀點後補回格子原點；環境記憶體因此從 62 降到 47 BRAM。
PLE-CSE 原有測試全部通過（`tb_cse_worst` 的格外點在新格式不合法，改為預期 HIT，合法最壞情況由 `tb_cse_array_worst` 涵蓋）。
這份 patch 是針對 PLE-CSE v1（`7733701`）寫的；PLE-CSE 已併入本 repo 的 `hw/ple-cse`（v2），patch 套不上 v2，相關目標尚未對 v2 驗證。這份 patch 也是預計給協作者的 PR 內容。

```bash
make ple-patch       # 套用 CSE 修改（整合版必要）
make vectors-v0      # 產生 format-v0 golden 向量（需要 gcc）
make sim-v0          # 建表 vs fx_env_build（iverilog）
make sim-sys         # 建表 + PLE-CSE 規劃 vs fx_ref_solve（Verilator 5）
make sim-sys-all     # 全部 700 題
```

FPGA：`hw/vivado/build_v0.tcl`（Vivado 2025.1），上板：`hw/host/vcc_sys_host.py --port COM4 run`。
若 Windows 把 3121 所在的埠範圍保留給 Hyper-V（`netsh int ipv4 show excludedportrange protocol=tcp`），
用 `hw_server -s TCP::3400` 啟動 JTAG 伺服器，`program.tcl` 與 host 腳本以 `localhost:3400` / `--hw-url localhost:3400` 連線。

## 目錄

```
spec.md                      設計文件（方法、硬體架構、實驗計畫）
hw/
  rtl/                       Verilog-2005（Vivado / iverilog）
  tb/                        testbench（建表+查詢、FK、整顆 core 規劃、UART 端到端）
  golden/                    golden 向量產生器（直接 #include SoC_軟體驗證資料包/soc_model/core.cpp）
  vectors/                   已產生的測試向量
  sim/                       run_all.sh 等模擬腳本
  vivado/                    建置 / 燒錄腳本與 xdc（Nexys A7、Arty、Zynq）
  host/                      板上 driver 與自我測試（UART 或 Zynq MMIO）
  ple-cse/                   PLE-CSE（RRT-Connect 規劃 PLE + 碰撞查詢 CSE）的 RTL、tb、OOC 腳本、C driver
sw_rrtc/                     PLE-CSE 的定點 golden model（fx）、RTL 產生器、host 工具
docs/                        PLE-CSE 規格與 v2 計畫
toolchain.mk                 sw_rrtc 用的 RISC-V GCC 偵測
  REPORT_meeting.md          進度報告
SoC_軟體驗證資料包/           整數功能模型、驗證 runner、獨立參考、資料
  soc_model/core.cpp         golden model（C ABI）
  upstream/vamp-vcc          git submodule：nycu-caslab/vamp-vcc @ 96a0d55（固定版本）
```

## 快速開始

```bash
git clone --recurse-submodules git@github.com:kd229y/vcc-based-accelerator-.git
make help
make sim          # 全部 testbench（需要 iverilog >= 11）
make vectors      # 重新產生 golden 向量（需要 g++、numpy）
```

FPGA（Windows + Vivado 2025.1，Nexys A7-100T）：

```bat
cd hw\vivado
vivado -mode batch -source build_uart.tcl -tclargs nexys
vivado -mode batch -source program.tcl -tclargs out_nexys\vcc_uart_top.bit
cd ..\host
python vcc_host.py --port COM4 --bit ..\vivado\out_nexys\vcc_uart_top.bit selftest
```

## 驗證狀態

- 模擬 18 項、Nexys A7-100T 實機 selftest 全部與 `core.cpp` 逐 bit 一致：環境 SRAM 內容、每筆查詢、每顆 FK 球心、整條規劃路徑與統計。
- 100 MHz timing 通過；BRAM 82.6%、LUT 28.5%、DSP 22%。
- 效能數字、限制與下一步見 `hw/REPORT_meeting.md`。

