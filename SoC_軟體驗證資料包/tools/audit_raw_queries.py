"""Correctness-first query audit against original binary32 points, without Morton in the oracle."""
from pathlib import Path
import hashlib
import json
import math
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import Model,load_config,array_pointer,IP
from runner.run_validation import write_json
from runner.data import digest_array
from reference.oracles import raw_point_collision


def main():
    if not load_config().get('raw_point_coverage'):raise RuntimeError('Historical assistant coverage audit: not applicable to the restored representative-point user profile')
    out=ROOT/'runs/correctness_priority_v2/raw_query_audit';out.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(20261104);config=load_config(max_range_m=None);results=[]
    # No radial culling in this audit: every original input point is in scope.
    groups=[]
    point=np.full((1,3),(1-1e-6)/8192,np.float32)
    n=np.arange(15,2048);centers=np.repeat(((n+.500001)/8192)[:,None],3,axis=1)
    radii=np.ceil((np.linalg.norm(centers-point[0].astype(float),axis=1)-.0025)*8192)/8192
    groups.append(('positive_diagonal_near_contact',point,centers,radii))
    groups.append(('negative_diagonal_near_contact',-point,-centers,radii))
    points=rng.uniform(-.35,.35,(2000,3)).astype(np.float32)
    centers=rng.uniform(-.6,.6,(2500,3));radii=rng.uniform(0,.12,2500)
    groups.append(('random_original_cloud',points,centers,radii))
    selected=points[rng.integers(len(points),size=1500)].astype(float)
    direction=rng.normal(size=(1500,3));direction/=np.linalg.norm(direction,axis=1)[:,None]
    radii=rng.uniform(.0001,.05,1500);gaps=rng.choice([-1e-6,-1e-7,0,1e-7,1e-6],size=1500)
    centers=selected+direction*(radii+.0025+gaps)[:,None]
    groups.append(('random_near_contact',points,centers,radii))
    for name,p,c,r in groups:
        expected=np.array([raw_point_collision(p,a,float(b)) for a,b in zip(c,r)],dtype=bool)
        with Model(config) as model:
            if model.build(p)!='SUCCESS':raise AssertionError('Audit construction failed')
            actual,_=model.query_many(c,r)
            scalar=np.array([model.query(a,float(b)) for a,b in zip(c,r)],dtype=bool)
            if not np.array_equal(actual,scalar):raise AssertionError('Batch/scalar disagreement')
            false_negative=np.flatnonzero(expected & ~actual);false_positive=np.flatnonzero(~expected & actual)
            np.savez_compressed(out/(name+'.npz'),points=p,centers=c,radii=r,expected=expected,actual=actual)
            results.append(dict(name=name,query_count=len(c),raw_collisions=int(expected.sum()),false_negatives=len(false_negative),
                false_positives=len(false_positive),scalar_batch_mismatches=int(np.count_nonzero(actual!=scalar)),
                pointcloud_sha256=digest_array(p),first_false_negative=int(false_negative[0]) if len(false_negative) else None))
    # An intentional unsafe filtered-only control must demonstrate information loss.
    p=np.array([[.0001,.0001,.0001],[.03,.03,.03]],np.float32);c=p[1].astype(float)
    if not raw_point_collision(p,c,0):raise AssertionError('Control oracle invalid')
    with Model({**config,'raw_point_coverage':False}) as unsafe,Model(config) as safe:
        unsafe.build(p);safe.build(p)
        control=dict(raw_collision=True,filtered_only_collision=unsafe.query(c,0),coverage_collision=safe.query(c,0))
        if control['filtered_only_collision'] or not control['coverage_collision']:raise AssertionError('Expected filtered-loss control changed')
    # Preserve the exact newly discovered ingress counterexample and its old encoding.
    c=np.full(3,25.500001/8192);radius=22/8192
    with Model(config) as m:
        m.build(point);legacy_center=np.floor(c*8192+.5).astype(np.int32)
        legacy=bool(m.lib.soc_query(m.handle,array_pointer(legacy_center,IP),22))
        counterexample=dict(point=point[0].tolist(),center=c.tolist(),radius_m=radius,
            original_point_clearance_m=float(np.linalg.norm(point[0].astype(float)-c)-radius-.0025),
            raw_oracle=raw_point_collision(point,c,radius),legacy_float_adapter_result=legacy,fixed_float_adapter_result=m.query(c,radius))
        if legacy or not counterexample['fixed_float_adapter_result']:raise AssertionError('Ingress regression not reproduced')
    valid=not any(r['false_negatives'] or r['scalar_batch_mismatches'] for r in results)
    raw_hits=sum(r['raw_collisions'] for r in results);total=sum(r['query_count'] for r in results)
    report=dict(valid=valid,query_count=total,raw_collisions=raw_hits,raw_free_queries=total-raw_hits,false_negatives=sum(r['false_negatives'] for r in results),
        false_positives=sum(r['false_positives'] for r in results),groups=results,seed=20261104,config=config,
        filtered_loss_control=control,ingress_counterexample=counterexample,
        source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'runner/model.py',ROOT/'reference/oracles.py',Path(__file__)]},
        native_binary=json.loads((ROOT/'build/build_report.json').read_text())['core_binary'],
        scope='Original binary32 point-radius spheres; independent float64 brute distances; no culling; fixed finite test set, not a proof for unseen sensor geometry or solid interiors. Conservative extra hits are counted, not hidden.')
    write_json(out/'summary.json',report)
    print(json.dumps({k:report[k] for k in ['valid','query_count','false_negatives','false_positives','filtered_loss_control','ingress_counterexample']},indent=2))
    if not valid:raise SystemExit(1)


if __name__=='__main__':main()
