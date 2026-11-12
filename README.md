# VCC-based Morton accelerator

以 Morton code 統一點雲濾波與碰撞結構建構，並把點雲前處理、碰撞查詢、FK 與 RRT-Connect 規劃放上晶片的整數加速器。
設計說明見 [`spec.md`](spec.md)；進度報告見 [`hw/REPORT_meeting.md`](hw/REPORT_meeting.md)。

## 分工與相關 repo

| 部分 | 負責 | 位置 |
|---|---|---|
| 點雲前處理：fp32→定點、徑向剔除、Morton 濾波（邊讀邊挑）、建表 | 本 repo | `hw/rtl/stage_a.v`、`hw/rtl/build_engine.v` |
| RRT-Connect 規劃（PLE）、碰撞查詢（CSE）、CPU | 協作者 | [SyncMeow/PLE-CSE](https://github.com/SyncMeow/PLE-CSE) |
| 軟體功能模型（golden）與驗證系統 | 本 repo | `SoC_軟體驗證資料包/` |

> **注意**：`hw/rtl/rrt_engine.v`、`hw/rtl/query_engine.v`、`hw/rtl/fk_engine.v` 是為了在 FPGA 上做端到端驗證而寫的**參考實作**。
> 正式的規劃與碰撞查詢以 PLE-CSE 為準。

### 與 PLE-CSE 的接點（待辦）

PLE-CSE 的 `cse_env_mem`（environment format v0）留有 build-engine write port。本 repo 的建表引擎接上去之前，輸出格式要先對齊：

| | PLE-CSE format v0 | 本 repo 目前 |
|---|---|---|
| 原點 | w_min = −1.19 m | −2 m（workspace-relative u16） |
| 查詢格目錄 | 20³，索引 (z·G+y)·G+x | 32³，索引為粗格 Morton key |
| cell AABB | 相對格子原點的偏移（11 bits） | 完整 6 × 16 bits |
| 緩衝 | 雙緩衝 | 單一 |

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

## 沒有放進 repo 的東西

- `SoC_軟體驗證資料包/` 的 `build/`（DLL 等編譯產物）、`runs/`、`experiments/`、`examples/`、`.venv/`、`manifest.json`：都是產生出來的檔案或大檔。
- `third_party/`（Eigen 3.4.0、nigh、pdqsort）：只有重建 Windows 軟體模型時才需要，固定版本見 `SoC_軟體驗證資料包/docs/來源與授權.md`。
- Vivado 輸出與 bitstream：依上面的指令重新產生。

上游程式碼、資料與第三方套件依各自原始授權使用，見 `SoC_軟體驗證資料包/docs/來源與授權.md`。
