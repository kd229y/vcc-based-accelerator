"""Recheck saved valid paths and bind unchanged native planning code to the prior full run."""
from pathlib import Path
import hashlib
import json
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from reference.oracles import RobotReference,validate_path
from runner.run_validation import write_json


def read(p):return json.loads(p.read_text(encoding='utf-8'))


def main():
    robot=RobotReference();build=read(ROOT/'build/build_report.json')
    native_hash=hashlib.sha256((ROOT/build['core_binary']).read_bytes()).hexdigest()
    if native_hash!=build['core_sha256']:raise AssertionError('Native build hash mismatch')
    manifests=list((ROOT/'runs/release_v1/regression').glob('*/run_manifest.json'))
    if len(manifests)!=700:raise AssertionError('Expected all 700 native provenance records')
    for p in manifests:
        if read(p)['native_binary_sha256']!=native_hash:raise AssertionError('Native core changed; full planning regression required')
    paths=list((ROOT/'runs/release_v1/regression').glob('*/result.json'))
    paths+=list((ROOT/'runs/correctness_priority_v2/system').glob('*/result.json'))
    checked=[]
    for path in sorted(paths):
        result=read(path)
        if result['status']!='SUCCESS':continue
        validation=validate_path(result['path'],read(path.parent/'problem.json'),robot)
        checked.append(dict(case_id=path.parent.name,source=path.relative_to(ROOT).as_posix(),validation=validation))
        if not validation['valid']:raise AssertionError('Saved success path now fails: '+path.parent.name)
    fixture=read(ROOT/'tests/fixtures/unsafe_filtered_detour.json')
    unsafe=validate_path(fixture['path'],fixture['problem'],robot)
    if unsafe['valid'] or unsafe['reason'] not in ('COLLISION','WAYPOINT_COLLISION'):raise AssertionError('Unsafe path control was not caught')
    if len(checked)!=8:raise AssertionError('Expected six MBM and two synthetic success paths')
    write_json(ROOT/'runs/correctness_priority_v2/retained_paths_revalidation.json',dict(valid=True,checked_success_paths=len(checked),
        paths=checked,unsafe_control=unsafe,native_binary=build['core_binary'],native_binary_sha256=native_hash,
        matching_prior_native_manifests=700,source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'runner/model.py',ROOT/'reference/oracles.py',Path(__file__)]},
        scope='No new 700-case planning run. Unchanged native core is SHA-256 matched to all prior cases; all six saved MBM successes and two current synthetic successes are independently revalidated.'))
    print('8 success paths certified; unsafe control rejected; 700 prior native hashes match')


if __name__=='__main__':main()
