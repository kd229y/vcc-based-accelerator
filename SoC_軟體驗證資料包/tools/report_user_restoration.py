"""Record restoration provenance and separate native planner results from external checks."""
from collections import Counter
from pathlib import Path
import csv
import hashlib
import json
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.run_validation import write_json


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    config=read(ROOT/'spec/design_v1.json');smoke=read(ROOT/'runs/user_restored_v1/summary.json')
    run=ROOT/'runs/user_restored_regression_v1';summary=read(run/'summary.json')
    if config['version']!='user_morton_restored_v1' or config['raw_point_coverage']:raise AssertionError('Wrong active profile')
    if not all(u['valid'] for u in smoke['unit'].values()):raise AssertionError('Unit checks failed')
    paths=sorted((run/'regression').glob('*/result.json'))
    if len(paths)!=700:raise AssertionError('Full 700-case restoration run required')
    results=[read(p) for p in paths];baselines=[read(p.parent/'vcc_baseline.json') for p in paths]
    build=read(ROOT/'build/build_report.json');native_hash=sha(ROOT/build['core_binary'])
    for path in paths:
        if read(path.parent/'config.json')!=config:raise AssertionError('Mixed restoration configurations')
        if read(path.parent/'run_manifest.json')['native_binary_sha256']!=native_hash:raise AssertionError('Mixed native binary versions')
    source=ROOT/'docs/source/user_hackmd_original.md'
    if sha(source)!=read(ROOT/'docs/source/source_manifest.json')['sha256']:raise AssertionError('Source snapshot changed')
    old=read(ROOT/'archive/assistant_variant_before_restore/spec/design_v1.json')
    changes={k:dict(previous=old.get(k),restored=config.get(k)) for k in sorted(set(old)|set(config)) if old.get(k)!=config.get(k)}
    native=Counter(r['planner_status'] for r in results);checks=Counter(r['path_validation']['reason'] for r in results)
    bs=Counter(b['status'] for b in baselines);bc=Counter(b['independent_path_validation']['reason'] for b in baselines)
    bcert=sum(b['independent_path_validation']['valid'] for b in baselines)
    record=dict(active_profile=config['version'],source_spec=source.relative_to(ROOT).as_posix(),source_spec_sha256=sha(source),
        native_binary=build['core_binary'],native_binary_sha256=native_hash,configuration_changes=changes,
        module_tests=smoke['unit'],system_results=smoke['system'],regression=summary['regression'],
        native_status_counts=dict(native),independent_check_reasons=dict(checks),baseline_status_counts=dict(bs),
        baseline_independent_check_reasons=dict(bc),baseline_certified_success_count=bcert,
        source_sha256={p.relative_to(ROOT).as_posix():sha(p) for directory in ['runner','soc_model','reference','tests','spec'] for p in sorted((ROOT/directory).rglob('*')) if p.is_file() and '__pycache__' not in p.parts},
        limitations=['RRT DD, balance and exact upstream Connect control are not fully aligned; no full-SoC equivalence claim',
                    'Q24 angles/Q30 internal FK, budgets and SRAM envelope remain explicit software choices',
                    'No extra discarded-point coverage or continuous interval certificates in the active model'],
        old_results_scope='Archive and all earlier assistant profiles are historical; 6/700 does not measure this restored profile')
    write_json(ROOT/'reports/user_restoration_manifest.json',record)
    marker='<!-- RESTORED_RESULTS -->';report_path=ROOT/'reports/原方案還原說明.md'
    base=report_path.read_text(encoding='utf-8').split(marker)[0].rstrip()
    appendix=f'''\n\n{marker}

## 本次 700 組結果

| 指標 | 還原設定的目前整數模型 | 本次 VCC 基準 |
|---|---:|---:|
| 原生規劃器回報 SUCCESS | {native['SUCCESS']}/700（{native['SUCCESS']/7:.2f}%） | {bs['SUCCESS']}/700（{bs['SUCCESS']/7:.2f}%） |
| 通過本包獨立檢查 | {summary['regression']['certified_success_count']}/700 | {bcert}/700 |
| 原生成功但獨立檢查拒絕 | {summary['regression']['independently_rejected_successes']} | {bs['SUCCESS']-bcert} |
| 預算耗盡 | {native['BUDGET_EXHAUSTED']} | {bs['BUDGET_EXHAUSTED']} |
| 起終點被原生模型拒絕 | {native['INVALID_START']+native['INVALID_GOAL']} | {bs['INVALID_START']+bs['INVALID_GOAL']} |

這裡的「通過獨立檢查」包含本包的固定球模型、解析實體障礙、連續邊認證與端點標準；不能把所有拒絕都稱為 Morton 索引錯誤。拒絕原因已逐筆保存，RRT 與數值的未對齊項目也仍列明。因此本表是目前軟體實作的實測，不是原筆記每個尚未實作模組的等價性證明。

兩方共用同一份點雲陣列、相同 xorshift 取樣及 300 次迭代預算；網格大小與原點、定點運算、Connect 控制及模型 state 預算仍有差異，不能稱為只換資料結構的消融。完整輸入與版本見每例 manifest，整體版本見 user_restoration_manifest.json。

還原本身完成；幾何正確性驗收目前仍為 FAIL，原因是獨立拒絕的成功路徑存在。此結果照實保留，沒有為了通過檢查再改動原設計。
'''
    report_path.write_text(base+appendix,encoding='utf-8')
    # Include the raw status in tabular exports as well as individual JSON files.
    keys=['case_id','planner_status','status','input_geometry_valid','wall_seconds','kept_points','build_cpu_ns','plan_cpu_ns','peak_sram_bytes','distance_tests','state_calls','iterations','baseline_status','baseline_path_valid']
    with (run/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore');writer.writeheader()
        for r in results:writer.writerow({**{k:v for k,v in r.items() if k!='stats'},**r['stats']})
    print(json.dumps(dict(native=dict(native),independent=dict(checks),baseline=dict(bs),baseline_certified=bcert),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
