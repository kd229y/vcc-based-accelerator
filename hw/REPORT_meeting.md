# Morton 點雲前處理與碰撞查詢加速器：RTL 實作與 FPGA 驗證

> 2026-11-10 進度報告
> 對應 spec 7.4：**M5（RTL 實作、對照 golden model）完成；M6（合成、FPGA 原型）部分完成；第一輪效能優化完成**

---

## 1. 摘要

- 依 spec 第 3、6 章完成整顆加速器的 Verilog RTL：**點雲濾波 → Morton 建表 → 碰撞查詢 → 定點 FK → RRT-Connect 規劃**，全部為整數運算，主 CPU 只負責送題目、收結果。
- 以軟體功能模型 `core.cpp` 為 golden，**模擬 18 項、實機（Nexys A7-100T）全部項目逐 bit 一致**。比對範圍包括環境 SRAM 內容、每筆查詢、每顆 FK 球心，以及整條規劃路徑與統計數。
- 實機條件：100 MHz、timing 通過；資源用量 BRAM 83%、LUT 29%、DSP 22%。
- **第一輪效能優化完成**（第 4.5 節）：FK 6.5×、規劃 4.7–5.6×、查詢 1.9×、建表約 1.8×；優化後實機重新驗證，仍然逐 bit 一致。
- 在 100 MHz 的 FPGA 上，**規劃仍比 x86 CPU 慢約 5–17 倍**。依 spec 假設的 500 MHz ASIC 換算，差距約 1–3.5 倍，所以加速還需要下一輪架構改動（第 7 節）。

---

## 2. 硬體架構

```
   UART / AXI-Lite                      AXI-Stream（預留給 DMA）
        │ 暫存器匯流排                       │ fp32 點雲
        ▼                                   ▼
┌───────────────────────── 建構引擎 build_engine ─────────────────────────┐
│ Stage A：fp32→Q14、1.19 m 徑向剔除、Morton key、S=4 選點距離（3 級 pipeline）│
│ FILTER：hash（linear probing）邊讀邊挑，嚴格小於才取代、平手保留先到         │
│ PREFIX：32768 個粗格計數 prefix sum → offset 表 + cell id 配置             │
│ SCATTER：依粗格 counting sort          CELLFIX：格內依 key 低 6 bit 排序 + AABB │
└───────────────────────────────┬──────────────────────────────────────┘
                                ▼ 環境 SRAM：代表點、offset、cell id、cell AABB、global AABB
┌──────── 規劃引擎 rrt_engine ────────┐      ┌──── 碰撞查詢 query_engine ────┐
│ xorshift 取樣、全掃描 NN             │─────▶│ global AABB → 依實際半徑列舉粗格 │
│ 整數 ceil-sqrt steering、round 除法  │◀─────│ → cell AABB → 球對代表點距離      │
│ VAMP rake 邊驗證（rake 8 / res 32）  │      └──────────────────────────────┘
│ state 檢查：關節限制 → FK → 690 自碰撞對 → 59 球查詢 │
│ 雙樹交替、connect ≤ 1000、回溯       │─────▶ fk_engine：30 輪整數 CORDIC、Q30/Q24、59 顆球 → Q14
└─────────────────────────────────┘
```

| 模組 | 對應 spec | 重點 |
|---|---|---|
| `stage_a` | 3.3–3.4、6.2.4(a)、6.3.2 | fp32→定點只用移位器（無 FPU），向零截斷 |
| `build_engine` | 3.5–3.7、6.3（方案 A） | 容量滿時回報 CAPACITY_EXCEEDED，不會默默丟點（修正原 CenterVox 問題） |
| `query_engine` | 3.8、6.4 | 依 ⌈2R/L⌉+1 列舉粗格，**不寫死 27 格**，所以外層包圍球也正確 |
| `fk_engine` | 6.5.4 | 單一 36×36 乘法器 + 暫存器檔，循序執行 |
| `rrt_engine` | 6.5.2–6.5.3 | 控制完全以 FSM 實作 |

---

## 3. 驗證方法

1. **Golden**：原封不動 `#include` 軟體包的 `core.cpp`，另寫產生器輸出測試向量，所以參考端沒有重新實作。
2. **測試向量**

   | 類別 | 內容 |
   |---|---|
   | 建表 7 個場景 | 資料包 rtl_vectors 原始點雲、bookshelf_thin（52,500 點）、數值邊界（±0、非正規數、±1e30、\|x\|=1.19 / 2.0、同格平手）、容量溢位、Inf 輸入、空點雲、密集球（3,457 格） |
   | 查詢 | 約 4,500 筆（含資料包附的 512 筆，以及剛好接觸 / 差 1 LSB 的邊界案例） |
   | FK | 328 組（含零角度與關節上下限） |
   | 規劃 | 4 題：無障礙、synthetic detour、bookshelf（seed 1 / 3） |

3. **比對層級**
   - (a) 單元模擬：環境 SRAM 逐 entry 比對。
   - (b) 系統模擬：只透過暫存器匯流排操作整顆 core。
   - (c) UART 協定端到端模擬。
   - (d) **實機**：Python 經 UART 重播同一批向量，逐 bit 比對。
4. 規劃的比對標準：狀態、整條路徑（每個 waypoint 7 個 Q24 碼），以及 iterations / nodes / state 呼叫 / edge 呼叫 / query 次數，**全部須完全相同**。

---

## 4. 結果

### 4.1 模擬（iverilog）：18 / 18 通過

| 測試 | 結果 | 備註 |
|---|---|---|
| 建表 7 個場景（hash 容量 2^15，與 golden 相同） | PASS | 包含 CAPACITY_EXCEEDED、INVALID_INPUT 兩種錯誤狀態 |
| 建表 4 個場景（2^14，FPGA 實際設定） | PASS | |
| FK 128 + 200 組 | PASS | |
| 規劃 4 題 | PASS | detour：13 iter、22 nodes、388 state、22,831 query，完全一致 |
| UART 端到端（含命令傳到一半斷線後恢復） | PASS | |

### 4.2 實機（Nexys A7-100T，XC7A100T）：SELFTEST PASS

| 項目 | 結果 |
|---|---|
| 建表 5 個場景 | 全部代表點、offset、cell id、cell AABB 一致（最大：11,671 點 / 3,457 格） |
| Inf 輸入 | 正確回報 INVALID_INPUT |
| 查詢 850 筆 | 全部一致 |
| FK 20 組 | 0 個球心不一致 |
| 規劃 3 題 | 路徑與統計完全一致 |

### 4.3 資源與 timing（XC7A100T，100 MHz）

| 資源 | 優化前 | 優化後 |
|---|---|---|
| LUT | 10,333（16.3%） | 18,078 / 63,400（28.5%） |
| FF | 7,333（5.8%） | 11,217 / 126,800（8.9%） |
| BRAM36 | 112（83.0%） | **111.5 / 135（82.6%）** |
| DSP48 | 33（13.8%） | 53 / 240（22.1%） |
| Timing（100 MHz） | WNS +0.183 ns | WNS +0.065 ns，WHS +0.024 ns，通過 |

- BRAM 是主要限制。其中 32768-entry 的粗格計數表與 cell id 表就占 32 塊，佔用量和點數無關。
- FPGA 版容量：最多 12,288 個代表點（golden 為 24,576）、每棵樹 512 節點（golden 為 4,096）。bookshelf 場景約 7–9K 點、樹數十個節點，不受影響。

### 4.4 優化前的瓶頸分析

第一版（只求正確）在 detour 題中，每次 state 檢查平均約 5.2K cycles，其中 FK 佔約 70%（3,730 cycles），690 個自碰撞對約 13%。建表時每點約 5–7 cycles，另外每次建表都有約 100K cycles 的固定開銷（清表、prefix、scatter）。

### 4.5 效能優化（第一輪）

所有改動都保持與 `core.cpp` 逐 bit 一致：模擬回歸 18 項與實機自我測試全部重新通過，實機量到的 cycles 與模擬完全相同。

| 單元 | 改動 |
|---|---|
| FK | 7 個 CORDIC 同時算；3 條 MAC lane 全 pipeline，每 cycle 算出一個矩陣元素；關節常數與 turn 矩陣直接當運算元，不再複製進暫存器檔 |
| 自碰撞檢查 | 690 對從每 cycle 1 對改成 4 對 |
| 環境查詢（規劃器端） | 全域 AABB 剔除在自碰撞檢查期間預先算好，每顆球只需 1 cycle；只有通過的球才送進查詢引擎 |
| 邊插值與 steering | 7 個除法器同時算（每個取樣點約 490 → 約 95 cycles） |
| 查詢引擎 | cell-id 查表 pipeline 化，空格從 2 cycles 變 1 cycle |
| 建表 | 每點 2 cycles（原 3–5）；粗格計數表加 1-bit epoch 標記、hash 表在 scatter 時順便清除，所以連續建表免清表（省 32K cycles）；新增一 beat 一點的 128-bit 輸入埠（給 DMA） |

| 工作（100 MHz） | 優化前 | 優化後 | 加速 | 參考：x86（spec M0） |
|---|---|---|---|---|
| 單次 FK | 3,730 cycles | **572** | 6.5× | — |
| 規劃 detour | 2.03M（20.3 ms） | **361K（3.6 ms）** | 5.6× | Panda 平均約 0.64 ms |
| 規劃 bookshelf（seed 1 / 3） | 5.30M / 7.63M | **1.11M / 1.83M（11.1 / 18.3 ms）** | 4.8× / 4.2× | 同上 |
| 單次查詢（bookshelf 平均） | 548 | **285** | 1.9× | — |
| 建表 bookshelf 52,500 點（全速輸入） | 372K | **約 206K（2.1 ms）** | 約 1.8× | 117K 點約 1.65 ms |

- 「建表優化前」的 372K 是在每 cycle 75% 機率送 1 個 32-bit word 的條件下量的；優化後的數字用新的 128-bit 點輸入埠全速送入，並且不含首次清表。兩者輸入條件不同，所以只能看出量級。
- 優化後 detour 每次 state 檢查約 930 cycles，其中 FK 572（約 60%）、自碰撞約 180、查詢與邊插值約 180。
- 依 spec 假設的 500 MHz ASIC 時脈換算：detour 約 0.72 ms、bookshelf 約 2.2 ms，與 x86 的 0.64 ms 差距縮小到 1–3.5 倍（換算僅供參考，測試題目也不同）。

---

## 5. 過程中發現並修正的問題

| 問題 | 發現方式 | 意義 |
|---|---|---|
| 規劃器樹記憶體位址截斷，goal 樹節點寫進 start 樹 | 系統模擬卡死 | 單元測試抓不到，需要系統層比對 |
| hash 容量為 2^14 時，清表迴圈只清了一半粗格計數表 | **實機才出現**（模擬只測過 2^15） | 回歸測試已加入 FPGA 實際參數 |
| 100 MHz timing 不過（WNS 最差 −1.9 ns） | Vivado 報告 | FK 寫回、自碰撞平方、查詢距離、Stage A 加 pipeline 後通過 |
| 優化後 timing 又不過（規劃器全域剔除路徑，−1.2 ns） | Vivado 報告 | 改成在自碰撞檢查期間預先計算 → 通過（WNS +0.065 ns） |
| 板子 USB 接頭接觸不良 | — | UART bridge 加閒置逾時；host 會自動重連 / 重燒 / 重試 |

---

## 6. 限制與與 spec 的差異

1. **建表採方案 A（hash）**，因為軟體 golden 就是方案 A。spec 6.3.7 建議的方案 B（兩層區塊、無收尾）尚未實作。
2. 單一環境，沒有雙緩衝（6.1）。輸入是 fp32 點雲，沒有深度影像反投影（6.2.4(b)）。
3. 查詢只有 1 個引擎、1 條距離 lane（N = 1、P = 1）；規劃器一次只做一個 state 檢查。
4. 經 UART 送點雲（約 92 KB/s）只適合驗證，不適合量端到端延遲。
5. Zynq（AXI-Lite + DMA）流程的腳本已寫好但尚未實際跑過。
6. 只測了 1 個真實場景和 4 題規劃，**沒有在硬體上跑完整的 benchmark 成功率**。
7. 沒有量功耗與 ASIC 面積。

---

## 7. 下一步（建議優先順序）

1. **擴大驗證**：在板上跑更多 MotionBenchMaker 題目（數十到上百題），統計一致率。
2. **規劃器的 rake 平行化**：VAMP 的 rake 本來就是 8 個取樣點同時檢查；目前硬體一次做一個 state。同時做 2–4 個 state 需要多份 FK 與查詢引擎，FK 仍佔每次 state 約 60%。
3. **FK 再加速**：球心階段改成 3 列同時算（177 → 59 cycles），相鄰 compose 重疊以減少 drain。
4. **建表**：Stage B 加 forwarding，做到每 cycle 1 點；用佔用 bitmap 跳過空粗格，省下固定 32K cycles 的 prefix。
5. **查詢**：多條距離 lane（P）、多個查詢引擎（N）、edge kill。
5. **方案 B 兩層區塊**：減少 BRAM 與寫入次數（6.3.6、6.8）。
6. Zynq + AXI DMA 平台：量含資料搬移的端到端延遲。

---

## 8. 想討論的問題

1. **下一階段的重點**：先做效能（目標是追上或超過 CPU），還是先擴大驗證、做方案 B 比較？
2. **平台**：繼續用 Nexys A7（BRAM 已用 83%），還是換有 ARM 和 DDR 的 Zynq 板（例如 PYNQ-Z2、KV260）做端到端量測？
3. **評估指標**：論文要比的是 cycle 數（換算 ASIC 時脈），還是 FPGA 實測時間？是否需要走 Yosys/OpenROAD 估面積與功耗（spec 7.3 ④）？
4. **方案 A / B**：要兩者都實作做比較，還是直接改成方案 B？

---

## 附錄：檔案與重現方式

```
hw/
├─ rtl/        Verilog（stage_a、build_engine、query_engine、fk_engine、rrt_engine、vcc_core、uart_bridge…）
├─ tb/         testbench        ├─ golden/  golden 向量產生器（include core.cpp）
├─ vectors/    測試向量          ├─ sim/run_all.sh   一鍵回歸
├─ vivado/     建置 / 燒錄腳本、xdc      └─ host/vcc_host.py  板上 driver 與自我測試
```

```bash
python3 golden/make_vectors.py && sim/run_all.sh                         # 模擬
vivado -mode batch -source vivado/build_uart.tcl -tclargs nexys          # 建置
python host/vcc_host.py --port COM4 --bit <bitfile> selftest             # 實機
```
