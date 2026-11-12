"""Run the actual pinned VCC RRTC/FKCC baseline on all saved MBM problem definitions."""
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
import argparse
import csv
import json
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import VCCBaseline,load_config
from runner.data import load_problem,sample_cloud,digest_array,planning_samples
from runner.run_validation import write_json
from reference.oracles import RobotReference,validate_path


def worker(arguments):
    entry,output,config,n=arguments;problem=load_problem(ROOT/entry['problem_path']);points=sample_cloud(problem,n)
    case_id=f"{problem['scenario']}_{problem['source_index']:04d}";out=Path(output)/case_id;out.mkdir(parents=True,exist_ok=True)
    robot=RobotReference();samples=planning_samples(config['seed'],robot.limits,config['rrt_max_iterations'],goal=problem['goals'][0])
    start=time.perf_counter()
    with VCCBaseline(points) as b:
        result=b.plan(problem['start'],problem['goals'][0],samples,iterations=config['rrt_max_iterations'],maxnodes=config['rrt_max_nodes_per_tree']*2,range_rad=config['rrt_range_rad'])
        result['construction']=b.stats()
    baseline_problem={**problem,'start':np.asarray(problem['start'],np.float32).astype(float).tolist(),
                      'goals':np.asarray(problem['goals'],np.float32).astype(float).tolist()}
    result['path_validation']=validate_path(result['path'],baseline_problem,robot) if result['status']=='SUCCESS' else {'valid':False,'reason':'NO_SUCCESSFUL_PATH'}
    result['endpoint_representation']='binary32 actual baseline input; compared with that input, not the pre-conversion JSON decimals'
    result.update(case_id=case_id,input_sha256=digest_array(points),wall_seconds=time.perf_counter()-start,
                  analytic_initially_valid=bool(robot.valid_state(problem['start'],problem) and any(robot.valid_state(g,problem) for g in problem['goals'])),
                  source='pinned VCC RRTC/FKCC/MVT, Windows allocation adapters, matched xorshift plus every-tenth goal sample, dynamic_domain false and alternating trees; Connect growth and edge checking differ from integer driver',
                  samples_per_primitive=n)
    write_json(out/'result.json',result)
    return {k:v for k,v in result.items() if k!='path'}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--workers',type=int,default=4);parser.add_argument('--limit',type=int);parser.add_argument('--samples-per-primitive',type=int,default=10000)
    parser.add_argument('--output',type=Path,default=ROOT/'runs/baseline_700_v1');args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    index=json.loads((ROOT/'data/panda_mbm/index.json').read_text());config=load_config()
    if args.limit:index=index[:args.limit]
    results=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(worker,(entry,str(args.output),config,args.samples_per_primitive)) for entry in index]
        for future in as_completed(futures):
            result=future.result();results.append(result)
            if len(results)%10==0:print(f"baseline {len(results)}/{len(index)} {result['case_id']} {result['status']}",flush=True)
    counts={};valid=0;unsafe=0
    for r in results:
        counts[r['status']]=counts.get(r['status'],0)+1;valid+=r['status']=='SUCCESS' and r['path_validation']['valid'];unsafe+=r['status']=='SUCCESS' and not r['path_validation']['valid']
    summary=dict(case_count=len(results),status_counts=counts,independently_certified_successes=int(valid),independently_rejected_successes=int(unsafe),
                 analytic_initially_valid_count=sum(r['analytic_initially_valid'] for r in results),config=config,
                 note='Baseline original discrete validation does not guarantee continuous solid-scene validity; independently rejected paths remain visible.')
    write_json(args.output/'summary.json',summary)
    write_json(args.output/'all_results.json',sorted(results,key=lambda r:r['case_id']))
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
