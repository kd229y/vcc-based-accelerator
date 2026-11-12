"""Recheck saved paths after reference-validator improvements, without rerunning planners."""
from collections import Counter
from pathlib import Path
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from reference.oracles import RobotReference,validate_path
from runner.run_validation import write_json


def main():
    robot=RobotReference();index=json.loads((ROOT/'data/panda_mbm/index.json').read_text());out=ROOT/'runs/baseline_700_v1'
    rows=[];start_errors=[];goal_errors=[]
    for i,entry in enumerate(index):
        problem=json.loads((ROOT/entry['problem_path']).read_text());case=f"{problem['scenario']}_{problem['source_index']:04d}"
        result=json.loads((out/case/'result.json').read_text())
        expected={**problem,'start':np.asarray(problem['start'],np.float32).astype(float).tolist(),'goals':np.asarray(problem['goals'],np.float32).astype(float).tolist()}
        if result['status']=='SUCCESS':
            start_errors.append(float(np.max(np.abs(np.array(result['path'][0])-expected['start']))))
            goal_errors.append(float(np.max(np.abs(np.array(result['path'][-1])-expected['goals'][0]))))
            validation=validate_path(result['path'],expected,robot)
        else:validation=dict(valid=False,reason='NO_SUCCESSFUL_PATH')
        rows.append(dict(case_id=case,planner_status=result['status'],path_validation=validation))
        if (i+1)%100==0:print('revalidate',i+1,flush=True)
    counts=Counter(x['path_validation']['reason'] for x in rows)
    write_json(out/'independent_revalidation.json',dict(endpoint_representation='binary32 actual baseline input',
        note='Corrects reference endpoint comparison precision; uses saved paths and preserves original planner outputs.',
        case_count=len(rows),reason_counts=dict(counts),certified_successes=sum(x['path_validation']['valid'] for x in rows),
        maximum_start_error_rad=max(start_errors),maximum_goal_error_rad=max(goal_errors),results=rows))
    print(dict(counts),flush=True);print('maximum endpoint errors',max(start_errors),max(goal_errors),flush=True)


if __name__=='__main__':main()
