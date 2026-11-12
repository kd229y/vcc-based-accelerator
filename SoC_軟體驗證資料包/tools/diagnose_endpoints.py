"""Classify endpoint rejections using saved release inputs and independent geometry."""
from pathlib import Path
from collections import Counter
import json
import math
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.data import load_problem,sample_cloud,digest_array
from runner.model import Model,load_config
from runner.run_validation import write_json
from reference.oracles import RobotReference


def classify(model,q,robot):
    centers=model.fk(q).astype(np.int64)
    radius=np.ceil(robot.radii*8192).astype(np.int64)
    margin=math.ceil(model.config['fk_margin_m']*8192)
    i,j=robot.pairs.T
    self_hit=bool(np.any(np.sum((centers[i]-centers[j])**2,axis=1)<=(radius[i]+radius[j]+2*margin)**2))
    hits=[k for k in range(59) if model.query(centers[k],int(radius[k]+margin),codes=True)]
    return dict(self_collision=self_hit,environment_spheres=hits)


def main():
    robot=RobotReference();rows=[];counts=Counter()
    for case in sorted((ROOT/'runs/release_v1/regression').iterdir()):
        result=load_problem(case/'result.json')
        if result['status'] not in ('INVALID_START','INVALID_GOAL'):continue
        p=load_problem(case/'problem.json');points=sample_cloud(p)
        expected=load_problem(case/'run_manifest.json')['pointcloud_sha256']
        if digest_array(points)!=expected:raise AssertionError('Endpoint input hash mismatch')
        label='start' if result['status']=='INVALID_START' else 'goal';q=p['start'] if label=='start' else p['goals'][0]
        config=load_problem(case/'config.json')
        with Model(config) as conservative,Model({**config,'raw_point_coverage':False}) as filtered:
            if conservative.build(points)!='SUCCESS' or filtered.build(points)!='SUCCESS':raise AssertionError('Endpoint reconstruction failed')
            a,b=classify(conservative,q,robot),classify(filtered,q,robot)
            if not (a['self_collision'] or a['environment_spheres']):raise AssertionError('Rejection did not reproduce')
            e,s=robot.clearance(q,p);analytic_valid=robot.valid_state(q,p)
            coverage_only=not (b['self_collision'] or b['environment_spheres'])
            reason='COVERAGE_ONLY' if coverage_only else ('SELF_COLLISION' if a['self_collision'] else 'FILTERED_POINT_ENVIRONMENT')
            counts[reason]+=1;counts['analytic_valid_rejected']+=int(analytic_valid)
            witness=None
            if a['environment_spheres']:
                k=a['environment_spheres'][0];snap=conservative.snapshot();center=conservative.fk(q)[k]
                ds=np.sum((snap[:,1:4]-center)**2,axis=1);rr=math.ceil(robot.radii[k]*8192)+math.ceil(config['fk_margin_m']*8192)+math.ceil(config['point_radius_m']*8192)+snap[:,6]
                ix=int(np.flatnonzero(ds<=rr*rr)[0]);raw=points.astype(np.float64)-center/8192
                witness=dict(sphere=k,representative_source=int(snap[ix,5]),coverage_radius_m=float(snap[ix,6]/8192),
                             actual_raw_point_clearance_m=float(np.sqrt(np.min(np.sum(raw*raw,axis=1)))-robot.radii[k]-config['point_radius_m']),
                             analytic_environment_clearance_m=float(e))
            rows.append(dict(case_id=case.name,endpoint=label,reason=reason,analytic_valid=bool(analytic_valid),
                             analytic_self_clearance_m=float(s),conservative=a,filtered=b,witness=witness))
        if len(rows)%100==0:print('endpoint diagnoses',len(rows),flush=True)
    write_json(ROOT/'reports/endpoint_diagnosis.json',dict(counts=dict(counts),case_count=len(rows),cases=rows,
        scope='Only endpoints actually rejected by release_v1, same input SHA-256; coverage ablation changes no other setting. Analytic solid geometry is checked separately.'))
    print(dict(counts),flush=True)


if __name__=='__main__':main()
