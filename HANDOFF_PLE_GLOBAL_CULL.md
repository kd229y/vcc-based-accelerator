# Handoff：把全域 AABB 剔除搬到 PLE（v3）

日期：2026-11-25。對象：接手這項工作的組員。
本文件自成一份，不需要先讀 v2 的文件。檔案位置都相對於 repo 根目錄。

## 0. 一句話

讓 PLE 在送出環境查詢之前，先用環境的全域 AABB 擋掉一定不會撞到的球，直接當成 MISS，不送進 CSE。
**CSE 的 RTL 不改**；改的是 PLE（`ple_qd`）和頂層接線。
**先量、再決定要不要做**：步驟 1、2 只是量測，量完把數字交給負責人決定，才進入步驟 3。

## 1. 背景

### 1.1 現在誰在剔除

CSE 每個 CU 的 S0 會做全域剔除（`hw/ple-cse/rtl/cse/cse_cu.sv:136-138`）：

```systemverilog
global_cull = (sx_hi < hglo_x) || (sx_lo > hghi_x) ||
              (sy_hi < hglo_y) || (sy_lo > hghi_y) ||
              (sz_hi < hglo_z) || (sz_lo > hghi_z);
```

- `sx_lo = cx - r`、`sx_hi = cx + r`，19 bit 有號；`hglo_*`、`hghi_*` 是環境 header 的全域 AABB（u16，補 0 成 19 bit 有號）。
- 被剔除的查詢設 `skip_search`（`cse_cu.sv:165-166`），回 MISS。
- golden 的同一條規則：`sw_rrtc/fx/fx_env.c:133-135`（`c[k] + r < glo[k] || c[k] - r > ghi[k]` → MISS，並 `fx_cs.gculled++`）。

### 1.2 為什麼要搬

被剔除的查詢在 CSE 裡仍然要走完整條路：Q 握手 → query FIFO → dispatcher → 佔用一個 CU 的 S0 → response FIFO → R。
在 PLE 先擋掉，可以省下 Q/R 頻寬、FIFO 與 dispatcher 的位置，以及 CU 的少量時間。

**預期效益不大，所以要先量。** 被剔除的查詢在 CU 裡只要 1～2 個 cycle；沒被剔除的查詢平均要走約 19 格（見 2.2 的試量）。
另外 N_CU = 2 時整體瓶頸在 CSE（相對於理想 CSE 約 2.5 倍），減少 CSE 的工作量才有用，所以這項的效益只能用**真正的 CSE**來量，理想 CSE（`cse_bfm` mode 1）量不出差別。

### 1.3 CSE 為什麼不用改

- CSE 的剔除**保留**：CSE 對任何查詢都要答對，PLE 只是少送一些。
- 被 PLE 擋掉的球，結果本來就是 MISS，所以規劃結果和 golden 逐 bit 相同，只有「送進 CSE 的查詢數」和 cycle 數改變。
- CSE 的 Q/R/K 介面不變。介面規則「每筆被接受的查詢恰好一筆回應」（`docs/INTERFACES.md` I1）不受影響，因為被擋掉的查詢根本沒有送出。

### 1.4 相關模組的階層

```
ple_cse_top (hw/ple-cse/rtl/ple_cse_top.sv)
├── ple_top   (rtl/ple/ple_top.sv)      AXI-Lite、計數器暫存器
│   └── ple_core  (rtl/ple/ple_core.sv)
│       └── ple_evu   (rtl/ple/ple_evu.sv)
│           └── ple_qd    (rtl/ple/ple_qd.sv)   ← 發查詢的地方，改這裡
├── cse_top   (rtl/cse/cse_top.sv)      不改
└── cse_env_mem (rtl/cse/cse_env_mem.sv) 已經輸出 hdr_valid/err/n_pts/glo/ghi（暫存器，無延遲）
```

`ple_cse_top.sv` 裡已經有 `hdr_valid`、`hdr_err`、`hdr_n_pts`、`hdr_glo`、`hdr_ghi` 這些線（第 51-54 行），目前只接到 `cse_top`。

## 2. 步驟 1：量剔除比例（golden，約 10 分鐘）

### 2.1 指令

```bash
make -C sw_rrtc TARGET_PREFIX=none fx
build/sw_rrtc_host/rrtc_fx -p 0:699 -n 10 -d 0.2 | grep -E '^FXCSE'       # 階層模式（預設）
build/sw_rrtc_host/rrtc_fx -p 0:699 -n 10 -d 0.2 -f | grep -E '^FXCSE'    # flat 模式（只查細球）
```

- 輸出 `FXCSE,queries=…,gculled=…,cells=…,…`。剔除比例 = `gculled / queries`。
- 沒被剔除的查詢平均走的格數 = `cells / (queries − gculled)`。
- 先用 `-p 0:19 -n 1` 試跑（幾秒），確認輸出格式後再跑全部。全部的時間沒量過，可以用 `-p` 分段平行跑再加總。

### 2.2 試量結果（2026-11-25，20 題 × 1 seed，只是試跑）

| 模式 | queries | gculled | 剔除比例 | 未剔除查詢平均走的格數 |
|---|---|---|---|---|
| 階層 | 76,541 | 29,636 | 38.7% | 19.2 |
| flat | 323,543 | 197,976 | 61.2% | 4.9 |

v1 的舊數字（`records/20261108-fx-golden.md`，699 × 10，舊座標）是 38.2%，**不能直接沿用**：v3 的座標原點、每軸格數和建表規則都變了。正式數字以 699 × 10 為準。

## 3. 步驟 2：量效益上限（只改 testbench，不動 RTL）

目的：在動 RTL 之前，用真正的 CSE 量「如果 PLE 完美地擋掉所有可剔除的查詢，cycle 會少多少」。

### 3.1 做法：在 `cse_rtl.sv` 加一層只給測試用的過濾

`hw/ple-cse/tb/cse_rtl.sv` 是 PLE 測試用來包真正 CSE（`cse_top` + `cse_env_mem`）的外殼，已經有 `hdr_glo/ghi` 等線。在它的 Q 入口加一個由 plusarg 控制的過濾（例如 `+pcull=1`）：

1. 對每筆 `q_valid` 的查詢，用 1.1 的式子計算是否可剔除，條件是 `hdr_valid[0] && !hdr_err[0] && hdr_n_pts[0] != 0`。
2. 可剔除的查詢：不送進 `u_top`，對 PLE 直接 `q_ready = 1`，並把 `{tag, user}` 放進一個本地佇列。
3. 本地佇列在 `u_top` 這個 cycle 沒有 `r_valid` 時，送出一筆 `r_status = ST_MISS` 的回應（`u_top` 優先，因為 PLE 的 `r_ready` 永遠是 1）。
4. 另外記一個 `npcull` 計數，和現有的 `nq` 分開，結束時印出來。

注意：

- 這只是量測工具，**不進合成**。`cse_rtl.sv` 是我們的測試檔，可以改；`rtl/cse/` 底下不能改。
- 介面檢查器 `ple_cse_if_checker.sv` 是掛在 PLE 這一側的 Q/R/K 上，會檢查「每筆查詢恰好一筆回應」，所以這層過濾寫錯會被抓到。
- kill 之後才送出的本地 MISS 是合法的（INTERFACES.md I1：KILLED 只代表「沒算」，回真正的結果也可以）。

### 3.2 要跑的組合

用 `tb_core` 的真正 CSE 版本（`make -C hw/ple-cse test-ple-cse NCU=n` 的建置，`+perf=1` 取 cycle 數），每組各跑 `+pcull=0` 和 `+pcull=1`：

| 組合 | 為什麼 |
|---|---|
| N_CU = 2，階層 | 目前的預設 |
| N_CU = 8，階層 | 大板子的方向；CSE 不再是主要瓶頸時效益會變小 |
| N_CU = 2，flat | flat 的剔除比例高很多，作為上限參考 |

- 每組都要確認 `errors=0`（結果仍和 golden 逐 bit 相同）。
- 記錄：總 cycle、`nq`、`npcull`、相對於 `+pcull=0` 的變化百分比。
- 題目與 seed 數照 `tb_core` 的預設（14 題 × 2 seed），和 `records/20261110-ple-cse-integration.md` 的 N_CU 掃描相同，方便對照。

### 3.3 決策點（停在這裡）

把 2.1 和 3.2 的數字整理成一張表，交給負責人決定要不要做步驟 3。
參考：上一次 PLE 的優化（SCU 列過濾）在真正的 CSE、N_CU = 2 時是 −1.5%，N_CU = 8 時是 −3.9%。

## 4. 步驟 3：RTL 實作（決定要做才進行）

### 4.1 剔除的規則（必須和 CSE、golden 完全一致）

```
cull = hdr_valid && !hdr_err && (hdr_n_pts != 0) &&
       ( (cx + r < glo_x) || (cx - r > ghi_x) ||
         (cy + r < glo_y) || (cy - r > ghi_y) ||
         (cz + r < glo_z) || (cz - r > ghi_z) )
```

- `cx/cy/cz`：PSM 讀出的球心（18 bit 有號，`COORD_W`），`r`：`RQ[a_sph]`（常數表）。計算用 19 bit 有號，和 `cse_cu.sv` 相同。
- 比較是**嚴格的** `<`、`>`：剛好碰到邊界（`cx + r == glo_x`）**不能**剔除。
- **只有 header 有效、沒有錯誤、而且有點時才剔除。** header 無效或有錯時一定要照常送出，讓 CSE 回 ERROR，PLE 才會回報 ENV_ERR；如果在 PLE 擋掉，錯誤會被吞掉。
- buffer：用 `cfg_env_buf` 選 `hdr_*[cfg_env_buf]`（目前 N_BUF = 1，固定是 0）。

### 4.2 建議做法 A：在 QD 的 stage A → Q 暫存器之間判斷（先做這個）

QD 發查詢分兩級（`ple_qd.sv`）：stage A 選出 slot 和球（`issue`），下一級把 PSM 讀出的球心放進 Q 暫存器（`a_adv`，第 204-212 行）。

在 `a_adv` 那一刻，`psm_rd` 和 `RQ[a_sph]` 都已經有了，可以直接算 `cull`：

- `cull = 1`：**不要**拉 `q_valid`；把這筆當成「立刻收到 MISS」處理：這個 slot 的 `n_outst` 減 1、`n_tag[cur_tag]` 減 1（`issue` 時已經各加了 1）。MISS 在現有的回應處理裡就是只做這兩件事（第 191-201 行）。
- `cull = 0`：照舊。
- 階層模式：外包球被剔除＝外包球 MISS，它的細球本來就不會排進 `pend`，不需要額外處理。

要小心的地方：

- **計數一致**：同一個 cycle 可能同時有 `kill_now`、真正的 R、`issue`、本地 MISS。`n_tag` 和 `n_outst` 都是組合式累加（`n_tag = …`），本地 MISS 也要用同樣的方式累加，不要用非阻塞賦值另外寫，否則會互相覆蓋。
- `kill_now` 時 stage A 被丟掉的那筆（第 235 行 `if (issue || (a_v && !a_adv)) n_tag… - 1`）：確認被剔除的那筆不會被減兩次。
- pose 完成的條件是 `outst[s] == 0`（第 243 行），本地 MISS 讓 `outst` 歸零後，pose 會正常完成。
- **時序**：PSM 是 LUTRAM，讀出來接 6 個 19 bit 比較器再進 Q 暫存器，是一條新的路徑。若 75 MHz 不收斂，改成多一級暫存器（QD 每 cycle 發 1 筆的吞吐量不能變）。
- 可選的優化：被剔除時 Q 暫存器不需要空出來，`a_adv` 的條件可以不等 `q_ready`。先不做，等正確性過了再說。

### 4.3 備案做法 B：在 PSM 寫入時建剔除遮罩

pose 寫進 PSM 時每 cycle 寫 4 顆球（`wr_busy`），可以同時算這 4 顆的 `cull`，存成每個 slot 70 bit 的遮罩，選球時直接跳過被剔除的球。
好處是連 QD 的發送 cycle 都省下；代價是 4 組比較器 + `NSLOT × 70` bit 的遮罩，而且選球邏輯（`ec`、`eo`、`s_kid`、`s_out`）要改。只有做法 A 量出來 QD 的發送速率成為瓶頸時才考慮。

### 4.4 接線

1. `ple_qd` 新增輸入：`env_hdr_valid`、`env_hdr_err`、`env_hdr_nz`（`n_pts != 0`）、`env_glo[3]`、`env_ghi[3]`（u16）。
2. 一路往上接：`ple_evu` → `ple_core` → `ple_top` → `ple_cse_top`。
3. `ple_cse_top` 把現有的 `hdr_*`（取 `[0]` 或依 `mode[0]` 選）接進 `u_ple`。
4. 建議在 `ple_top` 加一個計數器「被 PLE 剔除的查詢數」，放在目前計數器之後的空位址（現有計數器在 0x0C0–0x0DC，`ple_top.sv:12-16`）。位址要和負責人確認，並更新 `hw/ple-cse/sw/ple_drv.c` 的定義與 `ple_top.sv` 開頭的註解。
5. **不改** `hw/ple-cse/rtl/cse/` 底下任何檔案。

## 5. 步驟 4：測試

### 5.1 會受影響的測試

| 測試 | 檔案 | 要做的事 |
|---|---|---|
| QD 單元 | `tb/tb_qd.sv` | 接上新的埠；glo/ghi 從 golden 取：`fx_ref_env_word(0, 3..8, 0)`（3–5 = glo x/y/z，6–8 = ghi x/y/z），valid/err 取 `fx_ref_env_word(0, 0, 0)` 的 bit 0 / bit 1 |
| EVU | `tb/tb_evu.sv` | 同上 |
| core | `tb/tb_core.sv`（`cse_bfm` 與 `+define+RTL_CSE` 兩種） | 同上；`cse_bfm` 版要自己提供 header 值，`cse_rtl` 版從 `cse_env_mem` 的 header 接出 |
| top | `tb/tb_top.sv` | 同上；`0x0CC`（env queries）只檢查非 0，仍會通過；新計數器要加檢查 |
| 驅動 | `tb/tb_drv.sv` + `tb/drv_harness.c` | 若新增計數器暫存器，要加讀取 |

### 5.2 一定要加的測試

1. **邊界**：在 `tb_qd` 加 directed 測試，讓球剛好 `cx + r == glo_x`（不能剔除，要送出）、`cx + r == glo_x − 1`（要剔除），六個面各一組。golden 規劃產生的查詢幾乎不會剛好落在邊界，只靠回歸抓不到 `<` 寫成 `<=`。
2. **header 無效或錯誤時不剔除**：`tb_core` 已有注入 ERROR 的 run（`expect_err`、`err_at`），確認加了剔除之後 ENV_ERR 仍然會被回報；再加一個「所有球都在 AABB 外，且 header 有錯」的情況，必須回報 ENV_ERR。
3. **計數**：`+pcull` 量測時的 `npcull` 和 RTL 版本的新計數器，在同一組題目上應該相同。

### 5.3 mutant

在 `hw/ple-cse/tb/mutants.py` 的 `qd_m*` 後面加至少這幾個，全部都要被測試抓到：

| 名稱 | 改法 |
|---|---|
| `qd_cull_le` | 一個面的 `<` 改成 `<=` |
| `qd_cull_axis` | 一個軸用錯的 glo/ghi（例如 y 用 x 的） |
| `qd_cull_valid` | 拿掉 `hdr_valid && !hdr_err` 的條件 |
| `qd_cull_cnt` | 本地 MISS 不減 `n_tag`（或不減 `n_outst`） |

## 6. 步驟 5：回歸、合成與交付

全部要附上實際輸出的最後幾行：

```bash
make lint
make test
make regress
make -C hw/ple-cse test-ple-mutants
make -C hw/ple-cse test-ple-cse NCU=1
make -C hw/ple-cse test-ple-cse NCU=4
```

- `make regress` 的最後一行應該是 `12/12 passed`（目前的數量，含 DDR）。
- OOC：`hw/ple-cse/scripts/ooc_impl_plecse.tcl`，`ple_cse_top`，13.333 ns（75 MHz），N_CU = 2，佈線後 WNS ≥ 0。記錄 `ple_qd` 的 LUT / FF 前後對照（目前整體 28,905 LUT、WNS +0.737 ns）。
- 效能：用 RTL 版重跑 3.2 的組合，和 `+pcull=1` 的上限比較，cycle 應該接近（RTL 版不會比上限好）。

## 7. 交付內容

1. 步驟 1、2 的數字表（決策點時先交一次）。
2. RTL 與測試的 commit（訊息要短；不要加 `Co-Authored-By`）。
3. 一份紀錄 `records/<日期>-ple-global-cull.md`：量測數字、做法、時序、資源、回歸結果。
4. `docs/INTERFACES.md` I3 補一句：PLE 也讀環境 header 的 `valid`、`err`、`n_pts`、`glo`、`ghi`（暫存器，無延遲）。

## 8. 規則與陷阱

- 流程用 `/rtl-workflow` skill（`.claude/skills/rtl-workflow/`），RTL 改完、模擬過了以後請 `rtl-reviewer` agent 獨立審查。
- **不改** `hw/ple-cse/rtl/cse/`（CSE 組員負責），也不改 `hw/rtl`、`hw/sim`、`hw/vivado`、`hw/host`（前處理組員負責）。
- 規劃結果必須和 golden `fx_solve` 逐 bit 相同。這項改動不應該需要改 golden；如果需要，先停下來問。
- Verilator 的 `cond ? m[k] : d` 會兩邊都求值，對 associative array 會默默建出新的項目。testbench 裡查表請用 if/else。
- Vivado 2024.1 合成時，round-robin 的索引運算用 32 bit `int` 而大小不是 2 的冪次會 segfault，這類運算要用窄的位寬（見 `cse_dispatcher.sv`）。
- 模擬跑得比估計久很多時，先看 log，不要一直等。
- `pkill -f <pattern>` 如果 pattern 出現在同一個指令列，會把自己的 shell 也砍掉。

## 9. 參考

| 內容 | 位置 |
|---|---|
| CSE 的全域剔除 | `hw/ple-cse/rtl/cse/cse_cu.sv:104-167` |
| golden 的全域剔除與統計 | `sw_rrtc/fx/fx_env.c:125-140`、`sw_rrtc/host/main_fx.c`（`FXCSE`） |
| QD 的發送與回應 | `hw/ple-cse/rtl/ple/ple_qd.sv` |
| 測試用的真正 CSE 外殼 | `hw/ple-cse/tb/cse_rtl.sv` |
| golden 環境的讀取（DPI） | `sw_rrtc/fx/fx_ref.c` 的 `fx_ref_env_word` |
| PLE/CSE 介面 | `docs/INTERFACES.md`（I1、I3） |
| N_CU 與理想 CSE 的比較 | `records/20261110-ple-cse-integration.md`、`docs/PLE_ARCH_STUDY.md` 5.8 |
