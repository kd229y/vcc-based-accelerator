# 使用者原方案：目前預設入口

目前預設已恢復使用者的 Morton 代表點設計，設定在 `spec/design_v1.json`。先前助理加入的被刪點覆蓋球、1.6 m 剔除範圍、自適應區間邊檢查及每十次目標取樣已撤回。保留原筆記本來就有的數值量化 ε 與定點 FK 誤差處理。

詳細變更、來源依據與尚未完成的對齊項目見 [原方案還原紀錄](reports/原方案還原說明.md)。本次來源筆記快照與 SHA-256 在 `docs/source/`。

## 執行

在本資料夾開啟 PowerShell：

```powershell
# 模組 整合與五個完整案例
.\.venv\Scripts\python.exe runner\run_validation.py --suite smoke --baseline

# 完整 700 個 Panda 場景
.\.venv\Scripts\python.exe runner\run_validation.py --suite regression --baseline

# 修改 C++ 後重新建置
.\.venv\Scripts\python.exe runner\build.py
```

也可執行 `run_validation.ps1 -Suite smoke -Baseline`。預設新執行輸出至 runs 的時間戳目錄，不覆蓋正式結果。

## 本次正式結果

- `runs/user_restored_v1/`：29 項模組與整合測試、五個端到端案例。
- `runs/user_restored_regression_v1/`：還原設定下的 700 組與 VCC 對照。
- `rtl_vectors/user_original_v1/`：本次 Q14 二進位測試向量。

規劃器是否找到路徑，看 `planner_status`；是否通過獨立檢查，看 `path_validation`。PATH_VALIDATION_FAILED 會保留原生 SUCCESS 與路徑，再列出獨立拒絕原因，不能把它當成規劃器沒找到路。

成功率與數值是否符合規格分開呈現。驗證發現差異時記錄反例，不自行更改設計語意來提高通過率。

## 數值及資料

座標 Q14，1 LSB=2^-14 m。環境點依原筆記存為 unsigned16 相對 workspace 最小角；檔案轉回世界座標為 `(stored-32768)/16384`。細格 3.125 cm，粗格 12.5 cm，挑點 S=4。點雲輸入依固定 MBM 場景取樣，所有比較分支共用同一個輸入陣列，個案 manifest 保存 SHA-256。

詳細格式見 [原方案數值與介面](spec/原方案數值與介面.md)。原始 VCC、資料集、URDF、SRDF 與 mesh 維持原固定版本。

## 歷史實驗

`archive/assistant_variant_before_restore/` 保存助理修改版的程式與設定。舊 `runs/release_v1/`、`runs/correctness_priority_v2/`、`rtl_vectors/v1/` 與原先的速度／功耗評估不屬於現在的使用者原方案結果。原先 6/700 的結論不得沿用。
