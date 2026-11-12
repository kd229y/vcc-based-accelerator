# SoC 軟體模型執行與結果說明

> 歷史文件：以下描述先前助理修改版。使用者已要求撤回額外策略，目前入口與有效結果請看 [使用者原方案](README_原方案.md)；以下 6/700、零漏判及效能數字不得當成原方案結果。

目前優先處理正確性，最新結果見 [正確性優先驗證更新](reports/正確性優先驗證報告.md)：27 項測試通過，修正浮點 query 球心量化的漏判，並新增 8,066 筆原始點雲比對。先前 700 組與速度比較保留為歷史執行結果；成功率不是正確率。

正式 runner 使用 `runner/data.py` 從固定場景重建點雲，seed=0、每 primitive 10,000 點，所有比較分支共用同一個陣列。向量化取樣器不保證與初始 examples 裡 XYZ 的另一個取樣器逐點相同。系統案例保存 input_points_f32.npy；700 組另保存生成參數及逐例 SHA-256，可用固定套件版本重建後核對。

本包已包含可執行的 C++ 整數功能模型、獨立參考計算、模組與整合測試、點雲到路徑的總驗證、原始 VCC 基準、時間與記憶體模型，以及後續 RTL 可使用的軟體測試向量。

先看 [實作與驗證結果報告](reports/實作與驗證結果報告.md)。它將「程式能正確執行」與「目前方案的規劃品質是否足夠」分開判斷。完整 SoC 軟體流程已建立，不表示現在的保守濾波方案已適合進入 RTL 或已證明省電。

## 1 在這台電腦直接執行

在本資料夾開啟 PowerShell，執行：

```powershell
# 模組 整合與五組端到端案例 並比較原始 VCC
.\.venv\Scripts\python.exe runner\run_validation.py --suite smoke --baseline

# 模組 整合 端到端及全部 700 組 Panda 問題
.\.venv\Scripts\python.exe runner\run_validation.py --suite all --baseline
```

也可以使用 `run_validation.ps1 -Suite smoke -Baseline`。若電腦限制 PowerShell 腳本執行，使用上面的 Python 命令即可，不需要變更系統安全設定。

`--suite modules` 只跑模組；`integration` 跑版本、緩衝與事件測試；`system` 跑兩筆合成及三筆 MBM 範例；`regression` 跑完整 MBM。輸出預設放在 `runs/` 的新時間戳資料夾，可以加 `--output` 指定位置。`--limit` 是偵錯限縮案例，正式全量驗證不要設定。

本次完整驗收輸出在 `runs/release_v1/`。`runs/` 中其他資料夾保留開發過程、已被取代的參數試驗及刻意測試的不安全版本，不能將其混成目前的驗收結果。

## 2 一次執行會產生什麼

| 檔案或資料夾 | 用途 |
|---|---|
| `summary.json` | 模組 整合 系統與 700 組回歸的分開統計 |
| `summary.csv` | 每個案例的狀態 CPU 時間 SRAM 與運算量 |
| `report.md` | 可讀的執行摘要 |
| `system/<case>/path.json` | 規劃狀態與七維關節角度路徑 |
| `system/<case>/path_validation.json` | 獨立路徑驗證結果與保證範圍 |
| `system/<case>/environment_snapshot/` | 格目錄 點資料 AABB 及二進位布局 |
| `system/<case>/trace_i64.npy` | 原生模型的詳細事件資料 有儲存容量上限 |
| `system/<case>/rrt_control_i64.npy` | 最新 runner 另存完整 NN 與插入控制串流 不受詳細 trace 容量限制 |
| `system/<case>/planner_events.jsonl` | 取樣 最近鄰 插入與 Connect 等控制事件 |
| `system/<case>/trace_replay.json` | 任意精度最近鄰 延伸與 parent 規則比對 |
| `system/<case>/timing_estimate.json` | 模型實際執行時收到事件的即時時間註記 |
| `system/<case>/energy_estimate.json` | 操作量與能量估算狀態 |
| `regression/<case>/` | 全量測試的輸入摘要 路徑與個案結果 |

SUCCESS 表示模型找到路徑且通過本包獨立檢查。BUDGET_EXHAUSTED 是固定預算內未找到；INVALID_START 或 INVALID_GOAL 可能是保守模型額外阻擋，不等同原始幾何一定碰撞。PATH_VALIDATION_FAILED 表示模型曾回報成功，但被獨立驗證器抓出問題，屬於必須處理的測試失敗。

詳細 trace 有儲存容量上限，不保證每次全部存下；dropped 會明列，不能當作完整 memory trace。正式 release 的 box 長案例已於 `runs/complete_control_replay_v1/` 補驗完整控制事件；新 runner 會直接分開保存這份控制串流。

PASS 的總檢查狀態只表示必要的模型正確性測試與成功路徑檢查通過；成功率、面積及能耗仍需另外達標。原始 VCC 在更強的獨立幾何檢查下遭拒絕時，保留它自己的原始成功狀態與拒絕原因，不能只保留有利案例。

## 3 模型如何對應原本方案

建構核心仍是細 voxel 中心點選擇、一次 Morton radix 排序、粗格分段與 AABB；查詢處理任意合法半徑，不寫死為 27 格。規劃包含整數最近鄰、定點 FK、環境及自碰撞、雙向 RRT Connect、樹更新與路徑回溯。

數值版本 v1 使用 signed 16 bit Q13 點座標、Q24 關節角與內部位置、Q30 旋轉、30 次 CORDIC、64 bit 中間運算，以及固定順序的平手規則。詳細參數在 `spec/design_v1.json`，[介面與數值規格](spec/介面與數值規格.md) 說明輸入輸出、狀態及 trace；誤差分析與溢位界在 `reports/numeric_bounds.json`。

驗證時發現，只有代表點而沒有原始點覆蓋保護，可以產生穿過原始障礙的路徑。因此提供兩個明確語意：

- `raw_point_coverage=false`：代表點模型，加量化誤差；用於原方案消融與反例，不能直接宣稱涵蓋整份原始點雲。
- `raw_point_coverage=true`：使用同一細格原始點範圍，計算代表點覆蓋球；預設採這個保守版本，但它明顯增加額外阻擋。

預設保守版本的距離剔除改為 1.6 m。這是依球模型鏈長、球半徑及檢查放大量保留的候選界限；原始 VCC 基準仍使用其 1.19 m 設定。4 MiB 是功能驗證的容量上限，不是最終晶片 SRAM 規格；實際完整常駐需求另由計數器與容量掃描報告。

運動邊以整數區間細分與運動界做保守檢查。相對原始 VCC 的離散 rake 檢查，這是不同的安全與效能條件，不能把全流程時間差全部歸因於 Morton。

## 4 獨立驗證的範圍

模組參考不使用待測 Morton 索引：用 map 分格、標準排序與暴力距離檢查。FK 另外從固定 URDF 建立 NumPy 齊次變換，並與上游 sphere FK 方程比較。

最終路徑以球形機器人、SRDF 自碰撞配對及原始解析障礙物檢查。對每段運動使用幾何 clearance 與關節鏈運動界，必要時細分；只靠畫圖或少量離散姿態不算通過。這個保證限於列出的模型，不包含實體機器人誤差、任意 mesh 的覆蓋證明、感測器完整性或馬達控制。

深度影像入口也有整數反投影、座標轉換及到路徑的合成案例測試。真實相機序列尚未提供，不能把現有表面點雲當成真實深度感測器測試。

## 5 原始基準與消融

`upstream/vamp-vcc/` 保持原始 commit，不在其中修改程式。Windows 配置相容副本位於 `build/vcc_windows_include/`，使用 Zig 隨附 LLVM 編譯原始 RRTC FKCC CenterVox 與 MVT。Eigen 自動向量化關閉，VAMP 顯式 AVX2 保留；相容性修改與命令保存在 `build/build_report.json`。

基準使用保存的 xorshift 取樣規則與每十次一次目標取樣，關閉 dynamic domain 並交替雙樹。原始 Connect 增長方式與定點模型仍有差異，數值精度與邊檢查也不同。全系統結果用於應用評估，單看其時間差不能當成資料結構的純收益。

CPU 核心消融可執行：

```powershell
.\.venv\Scripts\python.exe runner\compare_variants.py
```

它使用同一批點雲與查詢，比較原始 VCC、相同網格 VCC、浮點 Morton、整數代表點與整數保守覆蓋。查詢時間在原生批次內量測，暖機後取五次中位數，排除 Python 逐筆呼叫成本；SIMD、精度與覆蓋差異仍會影響比較。

## 6 硬體評估如何使用

```powershell
.\.venv\Scripts\python.exe runner\design_space.py
```

結果在 `runs/design_space_v1/`，包含 hash 容量、完整常駐 SRAM 容量、距離 lane、常駐 SRAM 與外部記憶體快取方案的掃描。

重新彙整已保存的全量結果與說明報告：

```powershell
.\.venv\Scripts\python.exe tools\diagnose_endpoints.py
.\.venv\Scripts\python.exe tools\finalize_report.py
```

這兩支工具讀取 `runs/release_v1/`；更換正式 run 時須同步調整來源，不能混用不同配置。`finalize_report.py` 也會產生 K=1/4/16/64/256 的環境建構攤提表。

`simulation/timing_assumptions.json` 的 500 MHz、延遲與頻寬都是可調假設。系統模式會在 C++ 實際產生操作時呼叫時間模型，再繼續後續規劃；目前為單一路徑相依的循序排程，不宣稱多引擎推測執行或 cycle accurate RTL。系統 smoke 的 CPU 時間包含 Python 模擬 callback；CPU 效能請看沒有 callback 的回歸及原生批次。

完整常駐 SRAM 計數與外部快取假設是不同儲存配置，要分開解讀。所有未知的能量係數保持 null，現階段不輸出虛構瓦數或省電百分比。填入有來源的 `simulation/energy_coefficients.json` 後才能得到係數式能量估算；面積、可達頻率與實際功率仍需 RTL 合成或實測。

## 7 重新建置與環境

目前 `.venv` 已有所需套件；修改 C++ 後執行：

```powershell
.\.venv\Scripts\python.exe runner\build.py
```

在其他電腦重建時，先建立 Python 虛擬環境並安裝 `requirements_runtime.txt`。本次在 Windows x64 驗證，整數核心使用本機 MinGW GCC，VCC 使用 `ziglang==0.14.1` 的 LLVM；VCC 基準需 AVX2 FMA BMI2。其他主機的建置尚未實測。

完整原始相依來源保留於 `third_party/`：Eigen 3.4.0、指定 commit 的 nigh 及 pdqsort，授權隨來源保存。`.venv`、Git metadata 與 Zig 編譯快取不納入資料清單校驗。

## 8 後續 RTL 銜接

```powershell
.\.venv\Scripts\python.exe tools\export_rtl_vectors.py
```

`rtl_vectors/v1/` 包含固定格式的點雲、環境記憶體、512 筆查詢及其預期答案、128 組 FK 輸入與 59 顆球的預期球心。位元寬度、端序及 packing 在 manifest 中記錄。這些是軟體 golden vectors，本包沒有實作 RTL、合成或 FPGA。
