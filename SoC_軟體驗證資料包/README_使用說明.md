# SoC 軟體驗證資料包

目前預設已還原使用者的代表點方案，請先看 [原方案入口與還原紀錄](README_原方案.md)。先前助理保守修改版的結果均屬歷史實驗。

整理日期：2026-11-03（Asia/Taipei）

本資料包為「以 Morton Code 統一點雲濾波與碰撞結構建構」的整體 SoC 軟體驗證準備輸入資料、模型及參考原始碼。範圍包含點雲前處理、環境 SRAM、碰撞查詢、FK、最近鄰與 RRT-Connect。

**後續已新增可執行的 SoC 軟體模型與總驗證系統。** 最新命令、完成範圍及實際結果請先讀 [使用者原方案入口](README_原方案.md)。本文件保留資料整理及格式說明；資料完整性通過不代表規劃品質或晶片效益已達標。

## 1. 先看哪裡

從規格、參考計算、SoC 模組到完整總驗證與硬體評估的建置順序，記錄於 [建置規畫書](SoC軟體功能模型與完整驗證系統建置規畫書.md)。目前 runner 與軟體模型已實作並執行；最新結果與操作請看 [執行與結果說明](README_執行與結果.md)。

| 路徑 | 內容 |
|---|---|
| `upstream/vamp-vcc/` | 完整 VCC 原始碼；固定於與 HackMD 相同的 commit |
| `upstream/vamp-vcc/resources/panda/` | Panda URDF、SRDF、碰撞球模型、meshes、原始 MBM 壓縮檔 |
| `data/panda_model_summary.json` | 從球模型 URDF 匯出的關節與碰撞球摘要；不是 FK 實作 |
| `data/panda_mbm/raw_yaml/` | 700 組問題的原始 scene/request，共 1,400 個 YAML |
| `data/panda_mbm/problems/` | 每組問題的標準化輸入及原始場景／請求 JSON |
| `data/panda_mbm/index.json` | 所有 700 組輸入的索引 |
| `examples/` | box、bookshelf_thin、cage 各一組 XYZ／PLY 點雲與起終點 |
| `tools/prepare_panda_data.py` | 不依賴 VAMP 的本資料包資料整理與點雲產生工具 |
| `tools/verify_package.py` | 檔案 SHA-256、模型依賴、問題與點雲數量完整性檢查 |
| `tools/check_upstream_transforms.py` | 選用：與上游 float64 變換程式獨立比對，需 NumPy |
| `docs/驗證計畫.md` | 整個 SoC 的正確性、整合、效能與能量驗證方式 |
| `docs/來源與授權.md` | 上游連結、版本、授權與已知比較限制 |
| `templates/` | 待填的設計設定及量測欄位；沒有虛構結果 |
| `reports/` | 本次資料整理與完整性檢查紀錄 |

UR5、Fetch、Baxter 的上游模型與問題壓縮檔也保留在完整 repository 中；本次實際解包、轉換及產生點雲的是 Panda。

## 2. 一組測試到底包含什麼

以 `examples/panda_bookshelf_thin_0001/` 為例：

- `problem.json`：Panda 七個關節的順序、起始角度、目標角度、障礙物尺寸及姿態。
- `pointcloud.xyz`：未濾波 XYZ；每列三個數值，單位公尺，沒有標題列。
- `pointcloud.ply`：同一份點雲的 ASCII PLY，可由支援 PLY 的工具開啟。
- `point_sources.csv`：每個點對應的障礙物，`primitive_index` 指向 `problem.json` 的 `primitives` 陣列。
- `metadata.json`：點數、seed、邊界、產生方式及資料檢查結果。

原始檔案由 `scene0001.yaml`、`request0001.yaml` 開始。此處 `0001` 保留來源編號；以陣列載入時的第 0 題通常對應來源編號 1，不要把兩種編號混用。

每個 primitive 預設取樣 10,000 點，seed 為 0。這些點雲是已知障礙物表面的合成取樣，沒有相機遮蔽、感測器雜訊或深度影像。深度反投影另有整數入口與合成測試；仍需真實相機資料驗證感測條件。

取樣工具使用 Python 標準函式庫的隨機數與面積加權取樣。它不是上游 NumPy 取樣器的逐點複製；公平比較時，讓 VCC 與新模型共用此處保存的**同一份**點雲，或統一改用上游取樣器重新產生並保存。

## 3. 不用安裝 VCC 就能先讀資料

在本資料夾開啟 PowerShell，可以使用已建立的資料整理環境：

```powershell
.\.venv\Scripts\python.exe -c "import json; from pathlib import Path; p=json.loads(Path('examples/panda_bookshelf_thin_0001/problem.json').read_text(encoding='utf-8')); print(p['joint_names']); print(p['start']); print(p['goals'])"
```

Python 載入方式：

```python
import json
from pathlib import Path

case = Path('examples/panda_bookshelf_thin_0001')
problem = json.loads((case / 'problem.json').read_text(encoding='utf-8'))
with (case / 'pointcloud.xyz').open(encoding='ascii') as stream:
    points = [tuple(map(float, line.split())) for line in stream]

joint_names = problem['joint_names']
start = problem['start']       # 七個關節，rad
goals = problem['goals']       # 一個或多個七維目標
# 可將 points、start、goals 送入 runner/model.py 的 Model 功能模型。
```

座標與角度單位固定為 m 與 rad。`world_from_local` 是 4×4 齊次轉換矩陣，採欄向量 `p_world = T @ [x, y, z, 1]`。障礙物 box 尺寸是完整邊長；cylinder 尺寸順序為 `[height, radius]`。不要再對已在世界座標中的 XYZ 重複套用障礙物變換。

`source_scene.json` 保留 allowed collision matrix、fixed frame transforms 等原始資訊；`source_request.json` 保留手指關節及完整請求。七維規劃向量只抽取 Panda arm，不表示手指或自碰撞設定可以忽略。

## 4. 重新產生與完整性檢查

`.venv` 已安裝資料整理與功能驗證所需的 PyYAML、NumPy 與 Zig 工具鏈。VCC 基準以本包 C++ 相容介面執行，沒有安裝原始 VAMP Python extension。此虛擬環境與本機 Python 綁定，移動到其他電腦後請重建。

```powershell
# 本機已建立環境，可直接重新產生相同預設資料。
.\.venv\Scripts\python.exe tools\prepare_panda_data.py --samples-per-primitive 10000 --seed 0

# 只檢查已保存的檔案、模型及點雲，不需要 PyYAML。
python tools\verify_package.py
```

完整性檢查需要 Git，以確認固定 commit 與原始碼未被修改。本次另外使用本機既有 NumPy，比對全部問題的 8,200 個障礙物姿態轉換，結果見 `reports/transform_reference_report.json`；這是輸入座標轉換檢查，並非機器人 FK 驗證。需要重跑時，使用有安裝 NumPy 的 Python 執行 `python tools/check_upstream_transforms.py`。

若在別台電腦準備資料整理環境：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools\requirements.txt
```

重新產生工具會覆寫本資料包的衍生 JSON／範例點雲；不修改上游 repository。若改變 seed 或點數，原本的 SHA-256 清單將不再相符，請把新輸出當作另一個資料版本保存。

## 5. 要跑原始 VCC baseline 時

完整原始碼位於 `upstream/vamp-vcc/`。目前已在 Windows 編譯並執行其 C++ 核心，相容配置修改另存副本；請使用 [最新執行說明](README_執行與結果.md) 中的 runner。以下另保留原始 Python 介面的 Linux／WSL2 準備方式。

以下是待執行的上游流程，**不是本次已通過的測試**。指令需在 Linux／WSL2 的 `upstream/vamp-vcc/` 根目錄執行，並準備 Python、C++17 編譯器與建置工具；建置過程可能下載上游依賴。

```bash
python3 -m venv .venv-baseline
source .venv-baseline/bin/activate
python -m pip install '.[examples]'
python resources/problem_tar_to_pkl_json.py --robot panda
python scripts/evaluate_mbm.py --robot panda --problem box --pointcloud --pointcloud_repr mvt --pointcloud_filter centervox
```

上游轉換器會建立它自己的 `resources/panda/problems.json`、`problems.pkl`，並以 VAMP 檢查起終點是否有效。本資料包的 `data/panda_mbm/problems/.../problem.json` 是便於新模型使用的獨立 schema，**不能直接冒充上游 `problems.pkl`**；其中 `validation_status` 明確標為尚未檢查。

上游 evaluate 腳本會自行重抽點雲。本包 runner 已另接入同一份輸入陣列比較雙方；完整整數模型位於 `soc_model/core.cpp`，不是由這條上游 benchmark 指令自動取得。

## 6. 建置順序紀錄

1. 讀入一筆已保存的點雲、Panda 模型與起終點，保留原始障礙物幾何供獨立驗證。
2. 建立簡單參考計算，以及整數 SoC 的量化、濾波、建表與碰撞查詢；同規格逐筆比對。
3. 加入高精度 FK 參考與定點 FK、暴力最近鄰參考、RRT 狀態機。
4. 串成完整功能模型，驗證環境版本、容量、查詢 ID、parent 與完成條件。
5. 獨立驗證成功路徑，再加入記憶體事件、cycles 與能量模型。

詳細驗收條件見 `docs/驗證計畫.md`；實際完成狀態見 `README_執行與結果.md` 與 `runs/release_v1/`。完整性報告只代表資料整理；模組、路徑與全量回歸的結果另行保存，不能互相替代。
