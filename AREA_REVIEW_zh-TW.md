# VCC build engine 面積審查：15 位容量整合版（歷史基線）

> 歷史基線：本文使用 PLE-CSE `154c078` 的 15 位容量整合結果。與同學 Nexys A7-100T 報告對齊的 14 位容量重綜合／實作數字，見[更新後本週合併報告](../PLE-CSE/reports/2026-11-18_weekly_vcc_review_cse_integration_zh-TW.md)；下文的 146 與 121 不代表目前 14 位版本。

日期：2026-11-18  
目標：`xc7a100tcsg324-1`、100 MHz、單一 CSE CU、單一環境緩衝；**優化範圍限 VCC**  
性質：RTL 與既有綜合結果的設計審查；本文**沒有修改 RTL，也沒有重新綜合或實作**。

**版本基準**：VCC RTL `e7be637`；PLE-CSE 目前的 `feature/v2-vcc-adapter` 分支、commit `154c078`。資源數字取自該版本的 `build/accel/ooc_v2_vcc_ple_cse/` 綜合報告。本文只審查 VCC 面積，CSE 只用來確認 VCC 記憶體介面要求。

## 結論

目前阻擋實作的是 BRAM：整合 top 需要 **148/135 RAMB36**，超出 13 個；其中 **VCC build engine 占 146 個**，PLE 占 2 個、CSE 占 0 個。VCC 的 `HASH 57 + PTS 48 + CNT 16 + CID 14 + BOX 11 = 146`。本報告後續的實測值、估算值和兩張水位圖都只計 VCC；不提出 PLE／CSE 面積優化。最值得先做的兩個實驗是：

1. **把整合版的 hash／point 容量改為 VCC 已上板的 16K／12,288 profile**。就現有映射推估可省約 50 個 RAMB36，但密集測例已用掉 11,671 點，餘裕只有 617 點，須用真實目標場景決定是否可接受。
2. **若必須保留 24,576 點容量，建立分階段共用的 BRAM bank pool**。`HASH` 在排序搬移後釋放，原有實體 bank 可轉存 `CID`、`SC`、`BOX`。以 VCC 各 RAM 的實測 tile 數做階段加總，重排流程後最高同時需求是 `HASH 57 + CNT 16 + PTS 48 = 121` 個，對比 VCC 現有 146 個，有約 25 個 tile 的潛在節省。這是架構容量盤點，**不是重新綜合後的預測值**；共用 bank 的埠寬、讀取並行性及 mux 仍需設計。較簡單的第一步是 `HASH→CID`，可望先省 14 個。

另一個方向是將 `PTS` 的三軸各 16 位改存為粗格內各 11 位。座標可逐位還原，但目前 CSE 的一拍 point port 只給索引；若 CSE 不變，VCC 還須另存每點高位並同步讀取。因此 48→33 位的毛節省不能直接當淨節省，詳見第 2-C、3.1 節。

## 1. 審查範圍與基線

目前的整合 top 為 [`ple_cse_vcc_build_top.sv`](../PLE-CSE/accel/rtl/ple_cse_vcc_build_top.sv)。[`cse_vcc_build_system.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_build_system.sv) 第 69–94 行只接入 VCC 的 `build_engine`／`stage_a` 與其原生環境記憶體。VCC 原本的 `query_engine`、`fk_engine`、`rrt_engine`、UART 與 AXI 包裝器不在這次綜合內，因此本報告的 VCC 數字是 **build engine，不是獨立 VCC 全 core**。

目前的 [`cse_vcc_adapter.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_adapter.sv) 是純接線轉接：把 CSE 的 linear DIR 位址重排成 VCC Morton CID 位址，以 `FFFF` 判斷空格，轉出 SC 的 base/count、BOX 每軸低 11 位，並直通 48 位 PTS；其介面保留 VCC RAM 的一拍讀取延遲。[`cse_vcc_system.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_system.sv) 使用單一 CU，建表時阻擋查詢。這些是 VCC 儲存格式改動須滿足的條件；本文不優化 CSE 電路面積。

基線取自[目前版本的階層綜合資源報告](../PLE-CSE/build/accel/ooc_v2_vcc_ple_cse/ple_cse_vcc_build_top_v2_adapter_ncu1_synth_util_hier.rpt)。表中只摘錄 VCC build engine：

| 區塊 | LUT | LUTRAM | FF | RAMB36 | DSP |
|---|---:|---:|---:|---:|---:|
| VCC build engine | 7,857 | 2,368 | 1,020 | 146 | 4 |

Vivado 的階層數字顯示 VCC build engine 本身已用 146 個 RAMB36。因整合 top 的 BRAM 超過器件容量，這次沒有 post-route timing；本文不以整合 top 的 timing 當 VCC 單獨的時序數字。

### 獨立 VCC 的 112 與目前 VCC build 的 146

使用者提供的 [VCC FPGA 驗證報告](https://hackmd.io/@kd229/SkPM8RxRzl) 第 4.3 節列出 Nexys A7-100T **整顆獨立 VCC** 的用量：優化前 112、優化後 111.5 個 BRAM36 等效量。其 FPGA profile 最多 12,288 個代表點、每棵 RRT 樹 512 節點。VCC [`hw/README.md`](hw/README.md) 第 69–94 行與 [`vcc_top.v`](hw/rtl/vcc_top.v) 第 5–12 行對應 `HASH_AW=14`。目前整合則由 [`cse_vcc_pkg.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_pkg.sv) 第 9 行給出 `VCC_PT_AW=15`，並在 [`cse_vcc_build_system.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_build_system.sv) 第 69–73 行接到 VCC build engine；代表 **32K hash slot、最多 24,576 點**。兩次 VCC 量測的 top 與容量設定均不同：

| 量測 | 內容 | hash／代表點容量 | RAMB36 |
|---|---|---|---:|
| HackMD 優化前 | 獨立 VCC 全 core，含原查詢／FK／RRT | 16K／12,288 | 112 |
| HackMD 優化後 | 同一獨立 VCC，效能優化後 | 16K／12,288 | 111.5 等效 |
| 目前整合中的 VCC | 只有 build engine／Stage A，容量加倍 | 32K／24,576 | **146** |

容量從 16K／12K 增到 32K／24K，會使最大兩張 VCC 表 `HASH` 與 `PTS` 約增一倍；以現有 57 與 48 個的映射趨勢推回 16K profile，VCC build 可能約省 52 個而落在約 **94 個**，這只是估算，**尚無目前整合 top 的 16K 綜合實測**。獨立 VCC 的 112 包含其 RRT 等其他記憶體，故不能直接當作 VCC build 的基線。下文一律以 **VCC build 的 RAMB36** 為資源指標。

VCC 的 `build_engine` 自初次加入 RTL 時預設 `HASH_AW=15`，上板用的 `vcc_uart_top` 則覆寫為 14。目前整合選用 15，沒有沿用上板的 14 設定；本報告的優化方向 A 正是量測這個 VCC 容量選擇。

### VCC 記憶體分解

| RAM | RTL 邏輯容量 | 實測 RAMB36 | 作用／觀察 |
|---|---:|---:|---|
| `u_hash` | 32K × 59 位 | 57 | 建表時選每個細格代表點，完成後清空 |
| `u_pts` | 24K × 48 位 | 48 | 建表 scatter／格內排序及之後 CSE 查詢 |
| `u_cnt` | 32K × 16 位 | 16 | 粗格計數 → prefix offset → scatter 寫指標 |
| `u_cid` | 32K × 16 位 | 14 | 完成後供 CSE 由 coarse Morton key 找 cell id |
| `u_box` | 4K × 96 位 | 11 | 每格 AABB |
| `u_sc` | 4K × 32 位 | 0 | **實際映成 2,304 個 LUTRAM**，不是 BRAM |
| **合計** | | **146** | |

數字取自目前版本的階層資源報告；`u_sc` 的映射另可在 [`Vivado log`](../PLE-CSE/build/accel/ooc_v2_vcc_ple_cse/vivado.log) 的「Distributed RAM: Final Mapping Report」確認。原始 RTL 在 [`build_engine.v`](hw/rtl/build_engine.v) 第 74–104 行。表中邏輯位元數不能直接除以 36 Kb 當實際 RAMB36 個數：埠寬、深度、推斷方式與綜合最佳化都會影響映射，例如 `u_pts` 是 24K×48，實測仍用 48 個。

## 2. 設計上不合算的地方與突破口

### A. 整合版固定使用最高容量 profile（最高優先）

目前 [`cse_vcc_pkg.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_pkg.sv) 第 9 行把 `VCC_PT_AW` 設為 15；[`cse_vcc_build_system.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_build_system.sv) 第 69–73 行將它同時接到 VCC 的 `HASH_AW` 與 `PTS_AW`。所以整合中的 VCC build 是 32K hash slot、最多 24,576 代表點。VCC UART 上板 profile 使用 `HASH_AW=14`、最多 12,288 點（[`hw/README.md`](hw/README.md) 第 69–94 行）。

將 hash 與 `PTS` 深度減半，按現有 57 與 48 個 RAMB36 的映射趨勢，可能各省約 28 與 24 個，**VCC build 約從 146 降到 94 個**；這是粗估，實際數字必須以目前 top 重新綜合、讀 VCC 階層報告為準。縮小 hash 會改變 probing 路徑與容量錯誤界線，但在成功建表的場景應維持相同代表點及查詢結果，仍需 regression 驗證。

風險在容量：現有 `s6_dense` 為 11,671 點、3,457 格（[`hw/README.md`](hw/README.md) 第 54–66 行），在 12,288 點上限之下只剩 617 點，約 5.0% 餘裕。這是壓力測例，不代表所有實際場景；也不能僅憑 bookshelf 約 7–9K 點就宣稱 16K hash 足夠。先蒐集 RRTC 目標環境的 `kept/ncells/probes` 分布與最大值，再決定是否採此 profile，並保留 `CAPACITY_EXCEEDED` 的軟體處理路徑。

### B. 記憶體重用：分階段共用 BRAM bank pool（保留原容量的首選）

這個方向利用各階段的資料存活時間。「存活」包括之後階段還必須讀取，並非只看當下有沒有寫入：

| 階段 | 必須保留的主要資料 |
|---|---|
| FILTER | `HASH`、`CNT` |
| PREFIX | `HASH`、`CNT`；現有設計開始寫 `CID`、`SC` |
| SCATTER | 同時讀 `HASH`、更新 `CNT`、寫 `PTS`；`HASH` 逐格清空 |
| CELLFIX | `PTS`、`SC`、`BOX`；整張 `HASH` 已不再需要 |
| 建表完成後查詢 | `CID`、`SC`、`BOX`、`PTS`；不再需要 `HASH`、`CNT`（整合 top 沒接 offset 讀埠） |

以下兩張圖都只用**VCC 各表的實測 RAMB36 個數**標示；`SC` 目前使用 2,304 個 VCC LUTRAM，因此不列入 BRAM 數字。第一張區分當階段使用／須保留與暫時閒置的專屬 RAM；不做重用時，VCC 每一階段實際都配置 146 個 RAMB36。

```text
現況：每張表各有專屬 RAM                         當階段使用／須保留   暫時閒置
FILTER    HASH 57 + CNT 16                              73             73
PREFIX    HASH 57 + CNT 16 + CID 14                     87             59
SCATTER   HASH 57 + CNT 16 + CID 14 + PTS 48           135             11
CELLFIX   PTS 48 + CID 14 + BOX 11                      73             73
QUERY     PTS 48 + CID 14 + BOX 11                      73             73
                                                    實體配置每階段皆為 146
```

第二張是延後產生 `CID/SC`、把已釋放的 hash bank 改作查詢資料的**架構估算**。數值是同時需求，並非已經綜合出來的物理用量。

```text
共用後的階段需求                                  同時需要的資料（RAMB36）
FILTER    [ HASH 57 ][ CNT 16 ]                                      73
PREFIX    [ HASH 57 ][ CNT 16 ]                                      73
SCATTER   [ HASH 57 ][ CNT 16 ][ PTS 48 ]                           121 ← 高水位
CELLFIX   [ PTS 48 ][ CID 14 ][ BOX 11 ]                             73
QUERY     [ PTS 48 ][ CID 14 ][ BOX 11 ]                             73

HASH 的實體 bank：建表時放 HASH ──SCATTER 後──▶ CID／SC／BOX
VCC 理想重用容量目標：121（目前 VCC 配置 146）
```

因此可重用的是使用時間錯開的資料。`HASH` 與 `PTS` 在 SCATTER 階段必須同時讀寫，若要共用就得改建表演算法。RAMB36 的物理深度／寬度與埠模式在配置後也固定；寬度不同的資料須先設計打包、位址與讀取排程。

可把 VCC 的 BRAM 想成幾組**固定硬體配置、用途隨階段切換的 bank**，而非每張邏輯表各自永久占一組：

```text
Bank A：FILTER/SCATTER = HASH  ──完成後──▶ CID + SC + BOX
Bank B：FILTER/SCATTER = CNT   ──完成後──▶ 可供其他建表後資料使用
Bank C：SCATTER 起 = PTS      ────────────▶ 查詢階段仍保留
```

若把 `CID`／`SC` 的建立延到 SCATTER 之後，`BOX` 本來也只在 CELLFIX 寫入，以 VCC 各表的現有映射數字估算，最大同時 BRAM 需求就是 SCATTER 的 **57+16+48＝121 個**；查詢階段的 `PTS 48 + CID 14 + BOX 11＝73 個`，加上 `SC` 即使改用 BRAM，容量也低於這個高水位。VCC 從 146 降到約 121 是**理想重用的容量目標**，實際 bank 化可能因資料寬度、埠數或複製而超過，必須用 Vivado 驗證。目前 [`cse_vcc_adapter.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_adapter.sv) 第 15–21、35–53 行要求 VCC 仍以一拍讀取提供 `CID/SC/BOX/PTS`；尤其 `BOX` 是 96 位且須和 `SC` 同拍返回，不能只因 hash bank 總位元數足夠，就假設能維持原本埠時序。

[`build_engine.v`](hw/rtl/build_engine.v) 第 223–230、348–358 行顯示：SCATTER 讀完每個 hash slot 就將它清為零；全部 slot 掃完後才進 CELLFIX。`CID` 雖在 PREFIX 就被寫入，但查詢要等 `env_valid` 才開始（[`cse_vcc_build_system.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_build_system.sv) 第 65–94 行）。可把 `CID` 的產生延到 SCATTER 完成，直接寫在原本 32K×59 的 hash RAM 低 16 位，並保持最高的 hash-valid 位為零；下次成功 build 可再把它視為空 hash slot。`CNT` 在 SCATTER 後保存每格的結束 offset，因此順序掃描 `CNT`、比較前一格的結束 offset，便能識別非空格並依 Morton 順序給 cell id。空格仍可寫 `16'hffff`。這保留了 VCC offset 讀取可能性，也避開在 `CNT` RAM 裡同時處理 epoch 與 CID 編碼。

`SC` 還可往前延伸這個構想：目前它在 PREFIX 就寫入，但直到 CELLFIX 才第一次讀取；若改在 SCATTER 後由 `CNT` 的相鄰結束 offset 重建 start/count，`SC` 可放進 hash RAM 每筆尚未使用的位元。例如 `CID` 16 位 + `SC` 32 位合計 48 位，小於 hash word 的 59 位。CID 以 coarse key 定址，SC 以 cell id 定址；位址 0–4095 的 word 同時有兩種欄位，寫入時須做位元遮罩或 read-modify-write，避免覆蓋另一欄。目前 CSE 對 directory 與 cell metadata 是先後讀取（[`cse_cu.sv`](../PLE-CSE/accel/rtl/cse/cse_cu.sv) 第 250–313 行），讀埠有機會共用；`BOX` 仍留獨立 RAM，因為 CSE 需要它與 `SC` 同步返回。這個進一步方案可望減少 VCC 的 2,304 個 `SC` LUTRAM，但有較多控制與時序風險。

這個共用 pool 可分步驗證：先讓 `HASH→CID`，預期去掉獨立 `u_cid` 的 14 個 RAMB36；再評估將 `SC` 與 `BOX` 也放進建表後釋放的 bank。`SC` 可望省 VCC LUTRAM，`BOX` 可望再省最多其現有的 11 個 RAMB36，但需要多 bank 同拍讀出或調整 VCC 外部讀取時序。`CID/SC` 延後建立至少增加一趟 32,768 格掃描（100 MHz 下理想下界約 0.33 ms，實際取決於讀寫排程）；需測首／末格、連續 build、失敗後重建及同址寫入。單做 `HASH→CID` 的 VCC 推估是 **146−14＝132 個**；完整 pool 或後述 `PTS` 重編碼才有較實用的空間。

另一條可行路線是讓 `CNT` 與 `CID` 共用 32K×16 RAM：建表完成後把 offset 表原位改成 CID。這同樣可能省 14 個 RAMB36，但必須保留／重新編碼 epoch tag，否則下次 build 可能把舊 CID 誤當有效 counter；整合 top 的 offset 埠雖未使用，獨立 VCC 的 MMIO offset 視窗也會受影響。若希望保留現有 VCC 除錯介面，先研究 `HASH→CID` 較合適。

### C. `PTS` 重複存每點的粗格高位（保留原容量且不減精度）

`PTS` 每點存三軸 16 位、共 48 位（[`build_engine.v`](hw/rtl/build_engine.v) 第 90–94 行），粗格邊長則是每軸 2¹¹ Q14 LSB（[`cse_vcc_adapter.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_adapter.sv) 第 27–33、43–52 行）。在同一粗格內，三軸各只需低 11 位；高 5 位由粗格座標恢復。這是**無損儲存重編碼**，不同於降低點座標精度。

若 `PTS` 改為 24K×33 位，按 1K×36 的理想映射是 24 個 RAMB36，對比目前 48 個，**毛節省約 24 個**。但目前 CSE 的 point port 只送 point index，cell token 未把粗格座標傳到距離 pipeline（[`cse_cu_pkg.sv`](../PLE-CSE/accel/rtl/cse/cse_cu_pkg.sv) 第 48–54 行；[`cse_cu.sv`](../PLE-CSE/accel/rtl/cse/cse_cu.sv) 第 336–385 行），而 [`cse_vcc_adapter.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_adapter.sv) 第 31–33 行目前直接接線。若維持 CSE 完全不變，VCC 必須另存每點的 15 位粗格高位（`P_HI`），與 33 位 `PTS` **同步一拍讀出、重建原 48 位**。`P_HI` 的邏輯容量為 24,576×15 位，理論下界 10 個 RAMB36、實際映射待測；所以 VCC-only 的淨節省會明顯小於毛節省 24 個。若允許未來改 CSE 資料傳遞，才可能由 cell context 提供高位、避免 per-point sidecar；本報告不把那種情況計入 VCC-only 預估。格內排序也必須讓低位與高位資料同步重排。

### D. `SC` 實際花 LUTRAM，欄位卻過寬（LUT 壓力的低成本方向）

`SC` 寫入 `{count[15:0], start[15:0]}`（[`build_engine.v`](hw/rtl/build_engine.v) 第 96–100、210–219 行）。一個粗格只有 2×2×2 個細格座標、共 64 個細格，因此 `count` 的表示範圍僅需 0–64，**7 位足夠**；24,576 點上限下，非空 cell 的 `start` 至多 24,575，**15 位足夠**。實際內部只要 22 位，VCC 輸出時可零延伸回目前 32 位格式。Vivado 目前已自行裁掉部分未用位元，`u_sc` 實測占 **2,304 個 VCC LUTRAM**；進一步壓寬的收益須重綜合確認。若反而強迫 `SC` 用 BRAM，會加重當前超額問題，應在 BRAM 已有餘裕後才比較。

目前的 CSE adapter 只使用 VCC `BOX` 每個 16 位座標的低 11 位，CSE 再加上粗格原點還原絕對 AABB（[`cse_vcc_adapter.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_adapter.sv) 第 43–52 行；[`cse_cu.sv`](../PLE-CSE/accel/rtl/cse/cse_cu.sv) 第 291–303 行）。因此在此整合模式，VCC 的 `BOX` 可考慮從 4K×96 壓到 **4K×66 位**，輸出埠將每段零延伸回 16 位即可，CSE 核心與 adapter 都不必改。按總位元數下界至少需 8 個 RAMB36，現況 11 個；**實際可省多少未經綜合確認**。VCC 獨立 `query_engine` 使用絕對 BOX，若要維持其原介面，須將這個儲存格式做成可選的整合設定。

### E. 32³ dense coarse directory 與工作空間不相稱（較大架構變更）

`CNT` 與 `CID` 都按 15 位 Morton key 配 32,768 格。Stage A 剔除半徑 1.19 m 以外的點，通過剔除的座標相對 −2 m 原點，各軸 coarse index 僅約 6–25，最多 20³＝8,000 個候選粗格（[`stage_a.v`](hw/rtl/stage_a.v) 第 28–45、68–113 行；[`build_engine.v`](hw/rtl/build_engine.v) 第 152–159 行）。其餘 dense 表位置不會有代表點。將粗格位址改為 20³ 緊湊座標，可把兩個 32K 表各縮至約四分之一，亦減少清表／prefix 時間；但必須同時更改建表索引、cell 順序與 CSE directory 位址，原本以 Morton 順序輸出的 bit-exact golden snapshot 可能不再相同。這比 A／B／C 的整合與驗證成本高，適合作為下一代格式，不宜直接當作低風險修補。VCC [`spec.md`](spec.md) 第 6.3.6 節也提到 Morton dense 目錄與緊湊索引的空間取捨。

### F. hash entry 額外存可由點座標重算的 score（次要）

hash slot 存 `{valid, d[9:0], z, y, x}` 共 59 位（[`build_engine.v`](hw/rtl/build_engine.v) 第 74–79、146–150 行）。`d` 由點座標特定位元計算（[`stage_a.v`](hw/rtl/stage_a.v) 第 91–114 行），理論上可在碰撞比較時由已存點重算，將 entry 降至 49 位。但 hash 隨機讀後要立刻比較，重算會拉長路徑或增加 pipeline；49 位也未必跨過有利的 BRAM 埠寬門檻。**目前沒有可靠 RAMB36 節省數字**，故排在容量／生命週期／點格式之後。

## 3. 如何再降低 121 個 BRAM 的高水位

121＝`HASH 57 + CNT 16 + PTS 48`，發生在 SCATTER。因此單純把建表**結束後**才用的 `CID/SC/BOX` 放進共用 pool，不能讓 121 再下降。下一步要減少其中一張同時存活的表，或讓 hash 代表點與最終點序列不再各存一份完整 48 位座標。以下數值都是用既有映射做的**方向估算**，不是新 RTL 的綜合結果。

### 3.1 `PTS` 改存粗格內 33 位座標，另存高位以保持現有介面

第 2-C 節的無損格式將 `PTS` 從 48 位降到 33 位，但為維持目前 CSE 所需的一拍 48 位 point port，VCC 還須並行存取每點 15 位高位。若 33 位主表約用 24 個、15 位 sidecar 約用 12–16 個 RAMB36，SCATTER 高水位約為 **57+16+24+(12–16)＝109–113 個 VCC BRAM**；這是未實測的映射假設。單算 `57+16+24＝97` **不適用於目前 CSE 介面**。此法保留 VCC 的選點與 sorting 規則、24,576 點容量，但必須使兩張點表同址、同拍讀寫，並在 VCC 輸出端重建 48 位。

### 3.2 改變資料流：保留 hash 中的代表點，以排序後索引取代完整 `PTS`

目前 SCATTER 將每個 hash slot 的 48 位點複製到 `PTS`，然後清掉 hash（[`build_engine.v`](hw/rtl/build_engine.v) 第 223–230、348–358 行）。另一種架構是保留 hash 中的點，只把 **15 位 hash slot 位址**寫入依粗格排序的 `P_IDX`。CELLFIX 排序索引，AABB 仍讀 hash 裡的完整點；查詢時先讀 `P_IDX[base+i]`，下一拍再讀 `HASH[slot]`。最終點的順序與數值可保持不變，但目前 CSE adapter 明定 VCC 記憶體為**一拍讀取**（[`cse_vcc_adapter.sv`](../PLE-CSE/accel/rtl/cse/cse_vcc_adapter.sv) 第 1–3、31–33 行）；串接兩次同步 RAM 讀取不能直接符合現有 point port。此方向需要另訂介面延遲／握手並驗證 CSE pipeline，**不能當作只修改 VCC 且 CSE 完全不動的方案**。

`P_IDX` 的邏輯大小是 24,576×15＝368,640 bit，僅按總位元數的下界約為 10 個 RAMB36；實際映射可能更高，**不能在此報告直接認定為 12 個**。假設映射約 12–16 個，SCATTER 高水位約為 `57+16+(12–16)＝85–89`，查詢階段約為 `HASH 57 + P_IDX 12–16 + CID 14 + BOX 11＝94–98`。即使不進一步疊放 `CNT/CID`，專屬表的合計也可能落在約 110–114 個 VCC BRAM，與目前 146 個相比有明顯空間；全部仍待綜合確認。

此方案的 hash 在查詢期仍存放點，因此**不能同時採用第 2-B 節的 `HASH→CID/SC/BOX` 重用**；`CNT` 在建表後仍可另行重用。此方案也不能沿用目前「SCATTER 清 hash、成功後略過全表清除」的行為：下一次建表前必須清空 hash，或設計 epoch／valid 機制。VCC 獨立版本的點記憶體視窗、CELLFIX 的 RAM 讀取延遲、CSE 的 `P_IDX→HASH` pipeline 與錯誤後重建也須一起處理。基於本次「只研究 VCC 面積」的範圍，先保留為後續架構候選。

### 3.3 縮小 `CNT` 的 32K dense 位址範圍

第 2-E 節推得通過徑向剔除的點只會落在各軸 coarse index 約 6–25，最多 20³＝8,000 格。若 `CNT` 以緊湊格位址而非完整 15 位 Morton key 定址，16 位寬的表理論容量從 32K×16 降至 8K×16，按 RAMB36 總位元數的下界約需 4 個，而目前實測是 16 個。若實際也能接近 4，VCC 的 SCATTER 高水位約降至 **57+4+48＝109**。若再搭配第 3.1 節維持目前 CSE 介面的 33 位 `PTS`＋高位 sidecar，仍需計入 sidecar，約為 `57+4+24+(12–16)＝97–101`，不能寫成只有 85。改用 row-major 順序會使 cell id／點排序不再與現有 golden snapshot 逐位一致；若要保留 Morton 順序則需設計 compact-rank 映射。兩者都會影響 VCC 資料格式與 adapter 定址，屬較大架構選擇。

### 3.4 不宜直接採用的捷徑：固定 8-slot 存放區塊

[`spec.md`](spec.md) 第 6.3.6 節提出「細格直接存入 2×2×2 的小區塊」方案，確實可能省掉 hash→PTS 搬移。但它的容量依點分布變化。依現有 golden [`s1_shelf`](hw/vectors/s1_shelf/exp_points.hex) 與 [`s6_dense`](hw/vectors/s6_dense/exp_points.hex) 逐點計算 6.25 cm 小區塊：前者 7,067 點分布在 1,297 塊，8-slot 配置為 10,376 slot、填滿率約 68%；後者 11,671 點卻分布在 **9,665 塊**，需要 **77,320 slot、填滿率約 15%**。因此若沿用固定 8-slot 而不設計緊密表示／備援路徑，密集隨機場景會比目前的 hash＋PTS 更浪費空間。這些數字是從測試向量做的靜態計數，沒有合成方案 B RTL。

### 3.5 選型建議

若目標場景確實不超過 12,288 點，先測第 2-A 節的 16K profile，可同時降低 `HASH` 與 `PTS`。若必須保留 24,576 點且維持目前 CSE 一拍讀取介面，先測 **VCC 的 `HASH→CID` 共用**、`SC/BOX` 儲存壓寬，再評估 **33 位 `PTS`＋高位 sidecar**。`P_IDX`、`CNT` 緊湊化與兩層區塊屬介面／資料格式／演算法變更，應以實際 RRTC 場景分布和查詢吞吐量決定，不應只看單一測例的平均填滿率。

## 4. 建議的決策與驗證順序

| 順序 | 原型實驗 | 量測／通過條件 | 主要取捨 |
|---|---|---|---|
| 1 | 對目前整合 top 綜合 VCC `HASH_AW=14`、`PTS_AW=14` | **VCC build** RAMB36、真實環境及 `s6_dense` 建表與查詢等價 | 容量上限 12,288；密集測例只剩 617 點 |
| 2 | 若容量不足，VCC 維持 15 位容量並做 bank pool：先 `HASH→CID`，再評估 `SC/BOX` | 同一 top 的 **VCC build 階層** RAMB36／LUTRAM、每格資料與查詢等價 | 多掃至少 32,768 cycles；固定埠寬、同拍多資料讀取與同址欄位寫入要排程 |
| 3 | VCC `SC` 22 位、`BOX` 66 位儲存格式 | VCC LUTRAM／RAMB36 與原生輸出資料等價 | `BOX` 獨立 VCC 查詢用途需保留相容模式 |
| 4 | VCC 33 位 `PTS`＋每點 15 位高位 sidecar | VCC RAMB36 淨減、現有一拍 48 位 point port 與查詢等價 | 兩張點表須同址同拍重排；節省量小於單算 33 位 |
| 5 | `P_IDX` 或緊湊目錄 | VCC BRAM、接口延遲、點序與查詢等價 | 會動到目前一拍讀取介面或目錄格式，列為後續研究 |

驗證時以**同一個目前版本的整合 top、同一器件與時脈條件**，從階層報告只摘錄 `u_build` 的 VCC 資源；獨立 VCC 板級數字不能與 VCC build 直接相減。先跑既有 VCC build golden 與目前分支的 VCC adapter/build integration tests，再用實際 RRTC 場景的最大代表點／格數決定容量。BRAM pool 共用與 `PTS` 重編碼應特別測空環境、第一／最後 coarse key、64 點滿格、容量溢位及連續兩次建表。若 VCC BRAM 只勉強落在器件容量以內，仍需完成最終整合的 place/route；這是驗收條件，並非 PLE／CSE 面積優化工作。
