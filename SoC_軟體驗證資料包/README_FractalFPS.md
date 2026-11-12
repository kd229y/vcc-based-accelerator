# 獨立 Fractal 分塊＋FPS 軟體模型

新增模型與原方案分開放置於 `experiments/fractal_fps/`。兩個前端接同一套 Morton 建表、碰撞查詢、FK 與 RRT；FPS 結果不會再次進入細格中心挑點。

- [模型說明與執行命令](experiments/fractal_fps/README.md)
- [五個完整案例與 CPU 計時](experiments/fractal_fps/runs/smoke_v1/report.md)
- [完整 700 組結果與原模型一致性](experiments/fractal_fps/runs/結果報告.md)
- [原中心代表點模型／Fractal 模型介面](experiments/fractal_fps/model.py)
- [兩前端比較入口](experiments/fractal_fps/run_compare.py)

原本的 `spec/design_v1.json`、原始模型程式與建置紀錄保持原樣。本實驗有獨立設定、程式、DLL、結果目錄及來源版本紀錄。
