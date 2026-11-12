# 只加分區版入口

第三版已獨立放在 `experiments/fractal_partition/`：Fractal 分區＋你原本的 voxel 中心挑點，沒有 FPS，跨區同 voxel 合併，使用同一 Morton／FK／RRT 後端。原方案與 Fractal＋FPS 版本保留。

- [模型說明與執行方式](experiments/fractal_partition/README.md)
- [完整驗證結果與 CPU 計時](experiments/fractal_partition/runs/結果報告.md)
- [機器可讀驗收](experiments/fractal_partition/runs/acceptance.json)
- [五組完整 Morton 表與輸出](experiments/fractal_partition/runs/smoke_v1/)
- [700 組回歸結果](experiments/fractal_partition/runs/regression_700_v1/summary.json)

這版只改資料如何分區處理，保持原本代表點與後端結果。路徑原生成功率、獨立幾何認證與執行時間分開呈現。
