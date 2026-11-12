"""Aggregate saved measurements without treating a software PASS as design approval."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.run_validation import write_json
from simulation.timing import estimate_energy


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def table(headers,rows):
    return '| '+' | '.join(headers)+' |\n|'+ '|'.join(['---']*len(headers))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)
def dist(values):
    a=np.asarray(values,dtype=float)
    return dict(count=len(a),median=float(np.median(a)) if len(a) else None,p95=float(np.percentile(a,95)) if len(a) else None,maximum=float(a.max()) if len(a) else None)


def correct_saved_memory_units():
    # Historical vcc_stats ABI reports element count. Correct only metadata, never path/timing.
    changes=[]
    for name in ['release_v1','final_api_checks_v1']:
        for path in sorted((ROOT/'runs'/name).rglob('vcc_baseline.json')):
            br=read(path);c=br['construction']
            if 'point_pool_unit_correction' in c or c.get('point_pool_size_unit')=='bytes':continue
            old=c['allocated_point_pool_bytes'];c['allocated_point_pool_bytes']=old*4
            c['point_pool_unit_correction']='Saved v1 metadata originally mislabeled native float32 element count as bytes; multiplied by 4. No path, status or timing changed.'
            c['point_pool_size_unit']='bytes'
            write_json(path,br);changes.append(dict(path=path.relative_to(ROOT).as_posix(),old_float_count=old,bytes=old*4))
    audit=ROOT/'reports/memory_metadata_correction.json'
    if changes:
        previous=read(audit)['files'] if audit.exists() else []
        write_json(audit,dict(scope='Metadata-only unit conversion; original source mvt.hh point_coord_pool_size = total_floats',files=previous+changes))


def main():
    if read(ROOT/'spec/design_v1.json')['version']!='integer_soc_v1':raise RuntimeError('Historical assistant variant report; current restored-user results must not be merged into it')
    correct_saved_memory_units()
    release=ROOT/'runs/release_v1';summary=read(release/'summary.json')
    execution=read(release/'execution.json')
    if execution.get('status')=='RUNNING':
        write_json(release/'execution.json',{**execution,'status':'COMPLETE' if summary['valid'] else 'FAILED_VALIDATION',
            'completed_unix':summary['completed_unix'],'completion_metadata_note':'Finalized from completed summary; original runner did not close its initial RUNNING metadata.'})
    cases=[read(p) for p in sorted((release/'regression').glob('*/result.json'))]
    baseline=[read(p) for p in sorted((release/'regression').glob('*/vcc_baseline.json'))]
    if len(cases)!=700 or len(baseline)!=700:raise AssertionError('Full candidate and baseline results required')
    bs=Counter(b['status'] for b in baseline);reasons=Counter(b['independent_path_validation']['reason'] for b in baseline)
    certified=sum(b['independent_path_validation']['valid'] for b in baseline)
    initially_valid=sum(c['input_geometry_valid'] for c in cases)
    common=[(c,b) for c,b in zip(cases,baseline) if c['status']=='SUCCESS' and b['independent_path_validation']['valid']]
    peak=dist([c['stats']['peak_sram_bytes'] for c in cases]);cpu_all=dist([c['stats']['build_cpu_ns']+c['stats']['plan_cpu_ns'] for c in cases])
    capacity=read(ROOT/'runs/design_space_v1/capacity_sweep.json');timing=read(ROOT/'runs/design_space_v1/timing_sweep.json')
    kernel=read(ROOT/'runs/kernel_ablation_v1/results.json');diagnosis=read(ROOT/'reports/endpoint_diagnosis.json')
    bounds=read(ROOT/'reports/numeric_bounds.json');vectors=read(ROOT/'rtl_vectors/v1/manifest.json')
    coefficients=read(ROOT/'simulation/energy_coefficients.json');energy_groups={}
    for label,subset in [('successful',[c for c in cases if c['status']=='SUCCESS']),('unsuccessful',[c for c in cases if c['status']!='SUCCESS'])]:
        totals=Counter()
        for c in subset:totals.update(estimate_energy(c['stats'],{},coefficients)['operations'])
        energy_groups[label]=dict(case_count=len(subset),status='UNCHARACTERIZED',energy_j=None,operation_totals=dict(totals))
    amortization=[]
    for row in kernel['results']:
        for label,v in row['variants'].items():
            build=v.get('build_cpu_ns',0)+v.get('filter_cpu_ns',0)+v.get('sort_cpu_ns',0)+v.get('table_cpu_ns',0)
            for reuse in [1,4,16,64,256]:
                amortization.append(dict(case=row['case'],variant=label,reuse_batches=reuse,queries_per_batch=1000,
                    measured_build_cpu_ns=build,measured_query_batch_cpu_ns=v['query_batch_cpu_ns'],
                    amortized_cpu_ns_per_batch=build/reuse+v['query_batch_cpu_ns']))
    write_json(ROOT/'runs/kernel_ablation_v1/reuse_amortization.json',dict(rows=amortization,
        scope='Arithmetic amortization of measured one-time build across K identical 1000-query batches; excludes inter-frame overlap and is not planning throughput.'))
    source_files={}
    for directory in ['runner','soc_model','reference','simulation','tests','spec','tools']:
        for path in sorted((ROOT/directory).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                source_files[path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    report=dict(software_correctness_gate=summary['valid'],application_quality_gate='NOT_ACCEPTABLE_FOR_RTL_FREEZE',
        module_tests=summary['unit'],regression=summary['regression'],baseline_status_counts=dict(bs),baseline_independent_reasons=dict(reasons),
        baseline_certified_success_count=certified,baseline_overflow_flag_count=sum(bool(b['construction']['overflow_possible']) for b in baseline),
        analytic_initially_valid_count=initially_valid,candidate_success_rate_all700=summary['regression']['certified_success_count']/700,
        candidate_success_rate_analytic_valid=summary['regression']['certified_success_count']/initially_valid,
        cpu_total_ns_all_attempts=cpu_all,packed_sram_peak_bytes=peak,
        common_certified_successes=len(common),common_candidate_cpu_ns=dist([c['stats']['build_cpu_ns']+c['stats']['plan_cpu_ns'] for c,b in common]),
        common_baseline_cpu_ns=dist([b['construction']['filter_cpu_ns']+b['construction']['build_cpu_ns']+b['planning_cpu_ns'] for c,b in common]),
        endpoint_diagnosis=diagnosis['counts'],energy_groups=energy_groups,source_sha256=source_files,
        limitations=['No device energy coefficients or synthesized power/area','Serial timing baseline, no multi-engine cycle accurate microarchitecture',
                    'Pinned sphere robot and analytic primitive scene scope','Default coverage differs from filtered-only original proposal',
                    'First goal per problem, fixed single planning seed and fixed budgets','Full-system baseline Connect and edge semantics differ'])
    write_json(ROOT/'reports/software_release_summary.json',report)
    rows=[[k,v] for k,v in summary['regression']['status_counts'].items()]
    kernel_rows=[]
    for r in kernel['results']:
        v=r['variants'];kernel_rows.append([r['case'].removeprefix('panda_'),*[f"{v[k]['query_batch_cpu_ns']/1000:.1f}" for k in ['A_original_vcc','B_matched_grid_vcc','C_float_morton','D_integer_filtered','D_integer_conservative']]])
    capacity_rows=[]
    for name in sorted({c['case'] for c in capacity}):
        feasible=[c for c in capacity if c['case']==name and c['status']=='SUCCESS'];best=min(feasible,key=lambda c:c['sram_budget_bytes'])
        capacity_rows.append([name.removeprefix('panda_'),best['hash_capacity'],best['sram_budget_bytes']//1024,f"{best['full_resident_peak_bytes']/1024:.1f}"])
    t=min(timing,key=lambda r:r['estimated_cycles']);slow=max(timing,key=lambda r:r['estimated_cycles'])
    system_checks=[read(p) for p in sorted((release/'system').glob('*/trace_replay.json'))]
    incomplete=sum(c['status']=='INCOMPLETE_TRACE' for c in system_checks)
    completion=read(ROOT/'runs/complete_control_replay_v1/summary.json')
    if not completion['valid'] or completion['case_count']!=incomplete:raise AssertionError('Incomplete control checks remain unresolved')
    report['control_replay_completion']=completion
    report['goal_count_distribution']=dict(Counter(len(read(p)['goals']) for p in sorted((release/'regression').glob('*/problem.json'))))
    write_json(ROOT/'reports/software_release_summary.json',report)
    text=f'''# SoC 軟體模型實作與驗證結果

已完成規畫中的整體軟體功能流程、獨立參考、模組與整合測試、700 組回歸及比較工具。**目前方案的應用品質不達標：保守版本只成功 6/700，尚無證據支持比原始 VCC 快或省電，因此不應直接凍結 RTL。**

資料與執行入口見 [操作說明](../README_執行與結果.md)，實際數值規格見 [介面與數值規格](../spec/介面與數值規格.md)。所有結果可追到 `runs/release_v1/` 的逐案例檔案；機器可讀統計在 [software_release_summary.json](software_release_summary.json)。

## 1 已完成與仍需硬體階段的工作

| 規畫階段 | 實際交付與狀態 |
|---|---|
| P0–P1 | 固定來源、數值與介面、可執行原版基準、獨立參考、刻意故障偵測 |
| P2 | 整數 ingress、Morton、中心點選擇、radix sort、格目錄、AABB、任意半徑 query |
| P3 | 全 59 球 FK、SRDF 自碰撞、保守區間邊驗證、數值誤差與溢位界 |
| P4 | 整數 NN、取樣、steering、雙樹 RRT Connect、插入及回溯 |
| P5 | 雙緩衝、版本、在途 query、延遲、亂序、取消 drain、backpressure |
| P6 | 高精度獨立路徑認證、705 個端到端執行、其中 700 個 MBM 回歸 |
| P7 | live functional-first 循序時間基準、SRAM/cache/外部交易、容量與條件掃描；多引擎重疊與持續吞吐量尚未建立 |
| P8 | 操作量、流量與可替換能量係數框架；無器件係數，因此總能量及功率未知 |
| P9 | 匯出軟體 golden vectors；未實作 RTL、綜合、實測或 RTL cycles 校正 |

P7 的單引擎基準可用來估計相依運算成本，但不能替代整體並行 SoC 的吞吐量驗證。這是軟體驗證版本的明確限制。實體相機與機器人亦尚未測試。

## 2 正確性測試

- 模組測試 18/18 通過；整合測試 7/7 通過。最後補強批次 API 的非有限數值與越界檢查、VCC 記憶體單位後再測，結果見 `runs/final_api_checks_v1/`。
- 覆蓋 Morton 階層、負座標與格界、hash 平手與容量、排序與目錄、暴力 query、CORDIC、任意精度 FK、上游 FK 球集合、NN、深度反投影、完整規劃及故障偵測。
- 五個系統案例有獨立建構比對、100 筆 query 比對、保存中間結果與 trace；其中兩個合成案例輸出有效路徑。原始詳細 memory trace 有 {incomplete}/{len(system_checks)} 超過儲存上限；已另以完整控制事件串流補驗所有 NN、steering 與 parent 規則並通過，見 `runs/complete_control_replay_v1/`。box 案例補驗 236 次 NN 與 77 次插入，且功能狀態、路徑及操作數與全量執行一致。
- 本次保守候選回報成功的 8 條路徑（2 合成 + 6 MBM）均通過獨立連續區間檢查，獨立拒絕的候選成功路徑為 0。

獨立檢查範圍為固定球形機器人、SRDF 配對、box/cylinder/sphere 解析實體場景與分段直線關節路徑。有限測試通過不等同形式驗證或實體機器人安全保證；沒有用動畫取代幾何檢查。

## 3 完整 MBM 回歸的品質

固定 seed=1、300 iterations、30,000 次 state 預算、每 primitive 10,000 表面點、相同輸入陣列給各方案。資料集保存全部 goals，本次對第一個 goal 規劃；實際 goal 數分布已記錄於機器可讀報告。

{table(['候選狀態','案例數'],rows)}
解析起終點檢查有效的問題共 {initially_valid}/700。成功率是 6/700 = {600/700:.2f}%；在解析有效集合中是 6/{initially_valid} = {600/initially_valid:.2f}%。大量 INVALID_GOAL 不是資料集原本都無效，而是候選表示過於保守。

端點診斷重建相同 SHA-256 點雲，對 {diagnosis['case_count']} 筆實際拒絕逐筆比較：{diagnosis['counts']['COVERAGE_ONLY']} 筆只在開啟 raw coverage 後拒絕，{diagnosis['counts']['FILTERED_POINT_ENVIRONMENT']} 筆在代表點版本也會拒絕；其中 {diagnosis['counts']['analytic_valid_rejected']} 個被拒絕端點在解析模型下合法。細節與球編號見 [endpoint_diagnosis.json](endpoint_diagnosis.json)。

例如 bookshelf_small_0001 的目標球 51，原始點距離 clearance 約 13.67 mm，卻被半徑約 29.54 mm 的代表點覆蓋球擋住。這支持先改善覆蓋表示，避免把 voxel 內所有點用一個過大的球包起來。

相反地，`runs/integer_system_v1/system/synthetic_detour_0000/` 保留代表點版本的反例：原生模型回報成功，獨立檢查在運動邊發現碰撞，總結果為 PATH_VALIDATION_FAILED。因此直接移除覆蓋放大，不能當成已解決問題。

## 4 原始 VCC 比較

原始固定 commit 的 RRTC、FKCC、CenterVox 與 MVT 已在 Windows 編譯執行，原始 repository 保持乾淨。只在副本轉接對齊配置；Eigen 自動向量化關閉，顯式 AVX2 保留。

{table(['原始基準狀態','案例數'],bs.items())}
原始基準在更強的獨立檢查下：

{table(['獨立檢查結果','案例數'],reasons.items())}
原版成功且完整認證的路徑為 {certified}/700。GOAL_MISMATCH 是指定的 Q24 等級端點容差檢查，不能把微小端點誤差算成碰撞；碰撞拒絕也只適用本包明確列出的球模型與實體障礙範圍，不能延伸為上游所有用途都錯誤。CenterVox 容量警告共 {report['baseline_overflow_flag_count']} 筆。

兩方使用相同表面點雲、固定 xorshift 與每十次目標取樣，且關閉原版 dynamic domain、交替雙樹。但原版 Connect 增長、離散邊檢查與點剔除半徑仍不同，因此整體延遲差不能全部歸因於 Morton。共同通過獨立認證的案例僅 {len(common)} 筆，沒有拿單方成功的短時間冒充同品質加速比。

## 5 CPU 查詢與環境重用

下表為本機原生 1,000 筆 query 的時間，單位 µs；一輪暖機後五輪取中位數，排除 Python 逐筆呼叫開銷。

{table(['案例','A 原始 VCC','B 同網格 VCC','C 浮點 Morton','D 整數代表點','D 整數覆蓋'],kernel_rows)}
本次 D 的查詢皆慢於 A；尚無 CPU 實測支持目前版本比較快。A/B 使用 SIMD，C 為 scalar FP32，D 有整數與計數成本；數值與保守語意也不同。A/B/C 各案例前 100 筆暴力比對沒有差異；D 的數值與查詢另由模組測試驗證。

建構、保留點數與操作計數見 `runs/kernel_ablation_v1/results.json`。環境重用 K=1/4/16/64/256 的攤提另外輸出 `reuse_amortization.json`，公式為 build/K + 一批 query；這是靜態環境重用的算術估計，並非多 frame 並行規劃吞吐量。

700 筆候選所有嘗試的 build+plan CPU 中位數 {cpu_all['median']/1e6:.3f} ms、P95 {cpu_all['p95']/1e6:.3f} ms；成功案例中位數 {summary['regression']['successful_cpu_total_ns_median']/1e6:.3f} ms、P95 {summary['regression']['successful_cpu_total_ns_p95']/1e6:.3f} ms。大量很早拒絕的問題會降低全集合時間，不能只看這個數字判斷好壞。

## 6 SRAM 與時間假設

預設 32,768-slot hash 的候選 packed SRAM 峰值，中位數 {peak['median']/1048576:.3f} MiB、最大 {peak['maximum']/1048576:.3f} MiB。包含 hash、雙份環境、目錄、排序暫存與預留雙樹；不是 CPU RSS 或已綜合面積。

36 組容量掃描中，各範例在已測選項內可通過的最小容量如下；不代表 700 場景全都能用這個容量。

{table(['範例','hash slots','SRAM 上限 KiB','該配置峰值 KiB'],capacity_rows)}
另有 12 組時間掃描：常駐 SRAM 與 4/32/256 KiB 外部快取、距離 lanes=1/4/8。synthetic_detour 在候選 500 MHz 等假設下，串行註記範圍約 {t['estimated_seconds']*1000:.3f}–{slow['estimated_seconds']*1000:.3f} ms；這是條件估計，並非已達成的晶片延遲。live callback 與相同 trace replay 已由整合測試比對。

完整設定、cycles、bytes 見 `runs/design_space_v1/`。缺少多單元仲裁、跨 frame 重疊與器件延遲校正時，不報持續吞吐量或達標時脈。

## 7 能耗與數值界

使用者目前沒有 SRAM、DDR/NOR、時脈與製程資料，所有能量係數保留 null，所有能量輸出為 UNCHARACTERIZED。成功與失敗的操作量已分組於 software_release_summary.json；不輸出虛構的 W、J 或省電百分比。

數值分析的 FK 球心總界約 {bounds['total_fk_center_bound_m']*1000:.3f} mm，實際量化 margin 為 {bounds['configured_fk_margin_m']*1000:.3f} mm；矩陣累加界低於 int64 上限。1.6 m 剔除大於所推導必要界 {bounds['conservative_cull_required_m']:.4f} m。分析適用固定 URDF 與所列算術格式，細節見 [numeric_bounds.json](numeric_bounds.json)。

## 8 後續使用與設計決策

軟體框架可直接重新跑、改設定、定位漏判與測量成本。RTL 向量已包含 {vectors['query_count']} 筆查詢與 {vectors['fk_case_count']} 組 FK，每組 {vectors['spheres_per_fk']} 顆球，以及二進位環境快照。

建議先針對目前端點反例，評估更細 voxel、每格多代表點、較貼合的 bbox/局部覆蓋表示，並同時量測漏判、額外阻擋、成功率與 SRAM。必要誤差保護不能為了拉高成功率直接拿掉。改善後用同一套 700 組與更多 seed 重跑，再決定是否值得加入 FractalCloud 或進入 RTL。

本次完成的是可執行軟體模型與驗證證據；目前品質閘門未過。真實相機、完整並行微架構、RTL 合成及器件功耗校正是後續工作，沒有以軟體 PASS 代替它們。
'''
    (ROOT/'reports/實作與驗證結果報告.md').write_text(text,encoding='utf-8')
    print(json.dumps({k:report[k] for k in ['baseline_status_counts','baseline_independent_reasons','baseline_certified_success_count','packed_sram_peak_bytes','endpoint_diagnosis']},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
