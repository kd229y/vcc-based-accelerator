"""Recheck a long release case with a bounded-memory, complete control-event stream."""
from pathlib import Path
import json
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import Model
from runner.data import digest_array
from runner.run_validation import write_json
from reference.trace_check import validate_rrt_trace


def read(p):return json.loads(p.read_text(encoding='utf-8'))


def main():
    out=ROOT/'runs/complete_control_replay_v1';out.mkdir(parents=True,exist_ok=True);rows=[]
    for case in sorted((ROOT/'runs/release_v1/system').iterdir()):
        previous=read(case/'trace_replay.json')
        if previous.get('status')!='INCOMPLETE_TRACE':continue
        points=np.load(case/'input_points_f32.npy');manifest=read(case/'run_manifest.json')
        if digest_array(points)!=manifest['pointcloud_sha256']:raise AssertionError('Cloud SHA-256 mismatch')
        problem=read(case/'problem.json');config=read(case/'config.json');original=read(case/'result.json')
        start=time.perf_counter()
        with Model({**config,'trace_capacity':0}) as m:
            if m.build(points)!='SUCCESS':raise AssertionError('Replay build failed')
            controls=m.capture_control_trace();result=m.plan(problem['start'],problem['goals'][0]);stats=m.stats()
            if result['status']!=original['status'] or result['joint_codes']!=original.get('joint_codes',[]):raise AssertionError('Replay functional result changed')
            for key in ['queries','distance_tests','fk_calls','nn_calls','nn_distance_tests','edge_calls','rrt_nodes','iterations','state_calls']:
                if stats[key]!=original['stats'][key]:raise AssertionError('Replay operation count changed: '+key)
            trace=np.asarray(controls,dtype=np.int64).reshape((-1,8));validation=validate_rrt_trace(trace,stats,config)
            if not validation['valid']:raise AssertionError(validation)
            case_out=out/case.name;case_out.mkdir(exist_ok=True);np.save(case_out/'rrt_control_i64.npy',trace)
            r=dict(case_id=case.name,validation=validation,status=result['status'],pointcloud_sha256=manifest['pointcloud_sha256'],
                callback_event_count=m.callback_event_count,control_event_count=len(trace),stats=stats,wall_seconds=time.perf_counter()-start,
                scope='All native events delivered; only NN, roots and insertion payloads retained. Operation counts, path and status match release_v1. Detailed memory events are not stored.')
            write_json(case_out/'result.json',r);rows.append(r);print(case.name,validation,flush=True)
    write_json(out/'summary.json',dict(valid=all(r['validation']['valid'] for r in rows),case_count=len(rows),cases=rows))


if __name__=='__main__':main()
