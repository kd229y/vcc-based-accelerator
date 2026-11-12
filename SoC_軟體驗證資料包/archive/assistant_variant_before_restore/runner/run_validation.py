"""Executable entry point for modules, integration, point-cloud planning and regression."""
from pathlib import Path
import argparse
import csv
import hashlib
import io
import json
import platform
import subprocess
import sys
import time
import traceback
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import Model,VCCBaseline,load_config
from runner.data import sample_cloud,load_problem,digest_array,planning_samples,synthetic_cases
from reference.oracles import RobotReference,build_reference,brute_collision,validate_path
from reference.trace_check import validate_rrt_trace
from simulation.system import EnvironmentManager
from simulation.timing import simulate_trace,estimate_energy


def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False,default=lambda x:x.item() if isinstance(x,np.generic) else str(x))+'\n',encoding='utf-8')


def run_tests(out,pattern):
    start=time.perf_counter();loader=unittest.TestLoader();suite=loader.discover(str(ROOT/'tests'),pattern=pattern)
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    log=stream.getvalue();(out/(pattern.replace('.py','')+'.log')).write_text(log,encoding='utf-8')
    print(log,flush=True)
    report=dict(tests_run=result.testsRun,passed=result.testsRun-len(result.failures)-len(result.errors)-len(result.skipped),
                failures=[dict(test=str(t),traceback=s) for t,s in result.failures],errors=[dict(test=str(t),traceback=s) for t,s in result.errors],
                skipped=[dict(test=str(t),reason=s) for t,s in result.skipped],elapsed_seconds=time.perf_counter()-start,valid=result.wasSuccessful())
    write_json(out/(pattern.replace('.py','')+'.json'),report);return report


def export_environment(model,out):
    out.mkdir(parents=True,exist_ok=True);s=model.snapshot();cells=model.cells()
    np.savez_compressed(out/'snapshot.npz',points=s,offsets=model.offsets(),cells=cells)
    points=np.empty(len(s),dtype=np.dtype([('x','<i2'),('y','<i2'),('z','<i2'),('coverage','<u2')]))
    for a,name in enumerate(['x','y','z']):points[name]=s[:,1+a]
    points['coverage']=s[:,6];points.tofile(out/'points_q13.bin');model.offsets().astype('<u4').tofile(out/'offsets_u32.bin')
    dense=np.full(32768,65535,dtype='<u2')
    for i,c in enumerate(cells):dense[int(c[0])]=i
    dense.tofile(out/'cell_ids_u16.bin')
    dtype=np.dtype([('start','<u4'),('count','<u4'),('lo','<i2',(3,)),('hi','<i2',(3,))])
    packed=np.empty(len(cells),dtype=dtype)
    packed['start']=cells[:,1];packed['count']=cells[:,2];packed['lo']=cells[:,3:6];packed['hi']=cells[:,6:9];packed.tofile(out/'cells.bin')
    s[:,0].astype('<u4').tofile(out/'fine_keys_debug_u32.bin');s[:,5].astype('<u4').tofile(out/'source_indices_debug_u32.bin')
    write_json(out/'layout.json',dict(byte_order='little',coordinate_fraction_bits=13,point_record_bytes=8,cell_record_bytes=20,
        point_fields=['int16 x','int16 y','int16 z','uint16 coverage'],cell_fields=['uint32 start','uint32 count','int16 lo[3]','int16 hi[3]'],
        dense_cell_id_empty=65535,point_count=len(s),cell_count=len(cells),padding='none in scalar v1; a SIMD layout requires a versioned change',
        scope='RTL-ready software vectors; not evidence that RTL exists'))


def run_case(problem,points,config,out,baseline=False,trace=False,deep_reference=False):
    start=time.perf_counter();out.mkdir(parents=True,exist_ok=True);robot=RobotReference()
    write_json(out/'problem.json',problem);write_json(out/'config.json',config)
    manifest=dict(case_id=f"{problem['scenario']}_{problem['source_index']:04d}",point_count=len(points),pointcloud_sha256=digest_array(points),
                  raw_point_dtype='little endian float32 Nx3',config_sha256=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
                  source_commit='96a0d556e9677bb91c755aed7d268fa2772035b9',python=sys.version,platform=platform.platform())
    write_json(out/'run_manifest.json',manifest)
    if trace or deep_reference:np.save(out/'input_points_f32.npy',points)
    state_valid=robot.valid_state(problem['start'],problem) and any(robot.valid_state(g,problem) for g in problem['goals'])
    manager=EnvironmentManager(config);build_status=manager.prepare(points)
    result=dict(status=build_status,path=[],input_geometry_valid=bool(state_valid),case_id=manifest['case_id'])
    validation=dict(valid=False,reason='NO_SUCCESSFUL_PATH');checks={}
    if build_status=='SUCCESS':
        manager.publish();model=manager.slots[manager.active]['model']
        native_path=Path(model.lib._name)
        manifest['native_binary']=native_path.relative_to(ROOT).as_posix();manifest['native_binary_sha256']=hashlib.sha256(native_path.read_bytes()).hexdigest()
        write_json(out/'run_manifest.json',manifest)
        snap=model.snapshot()
        if deep_reference:
            expected=build_reference(points,config['selector_bits'],config['raw_point_coverage'],config.get('max_range_m'))
            np.testing.assert_array_equal(snap,expected)
            rng=np.random.default_rng(15)
            with Model(config) as probe:
                if probe.build(points)!='SUCCESS':raise AssertionError('Probe construction failed')
                for _ in range(100):
                    c=rng.integers(-14000,14000,3);r=int(rng.integers(0,3000))
                    if probe.query(c,r,codes=True)!=brute_collision(expected,c,r):raise AssertionError('Case query mismatch')
            checks['filter_sort_query_reference']='PASS'
        if trace:
            export_environment(model,out/'environment_snapshot');model.set_trace(800000)
            profile=json.loads((ROOT/'simulation/timing_assumptions.json').read_text());scheduler=model.attach_timing(profile)
        result={**result,**manager.plan(problem['start'],problem['goals'][0])}
        if result['status']=='SUCCESS':
            validation=validate_path(result['path'],problem,robot)
            if not validation['valid']:result['candidate_status']='SUCCESS';result['status']='PATH_VALIDATION_FAILED'
        if trace:
            records=model.trace();np.save(out/'trace_i64.npy',records)
            control=np.asarray(model.control_records,dtype=np.int64).reshape((-1,8))
            np.save(out/'rrt_control_i64.npy',control)
            replay=validate_rrt_trace(control,{**result['stats'],'trace_dropped':0},config)
            replay['capture_scope']='Complete control events streamed independently of bounded detailed memory trace'
            replay['detailed_trace_dropped']=result['stats']['trace_dropped']
            checks['rrt_trace']=replay
            write_json(out/'trace_replay.json',replay)
            timing=scheduler.result();timing['execution_mode']='live_native_event_callback_functional_first'
            timing['host_timing_note']='CPU planning time includes Python simulation callbacks in this run; use uninstrumented regression or native kernel batches for CPU performance.'
            coefficients=json.loads((ROOT/'simulation/energy_coefficients.json').read_text())
            write_json(out/'timing_estimate.json',timing);write_json(out/'energy_estimate.json',estimate_energy(result['stats'],timing,coefficients))
            # Control and all planned configuration inputs remain inspectable without huge JSON memory traces.
            with (out/'planner_events.jsonl').open('w',encoding='utf-8') as f:
                for row in records:
                    if row[0] in (4,5,6,7,8,9,30,31,32,33,34,35,38,39):f.write(json.dumps(list(map(int,row)))+'\n')
        result['stats']=model.stats()
    if baseline:
        samples=planning_samples(config['seed'],robot.limits,max(1,config['rrt_max_iterations']),goal=problem['goals'][0])
        with VCCBaseline(points) as b:
            br=b.plan(problem['start'],problem['goals'][0],samples,iterations=config['rrt_max_iterations'],maxnodes=2*config['rrt_max_nodes_per_tree'],range_rad=config['rrt_range_rad'])
            br['construction']=b.stats()
            baseline_problem={**problem,'start':np.asarray(problem['start'],np.float32).astype(float).tolist(),'goals':np.asarray(problem['goals'],np.float32).astype(float).tolist()}
            br['independent_path_validation']=validate_path(br['path'],baseline_problem,robot) if br['status']=='SUCCESS' else dict(valid=False,reason='NO_SUCCESSFUL_PATH')
            br['comparison_note']='Pinned original RRTC and FKCC, alternating trees and dynamic_domain disabled, matched xorshift plus every-tenth goal sample; original Connect growth and discrete edges differ from the integer driver. Full-system timing is not a Morton-only ablation.'
            write_json(out/'vcc_baseline.json',br)
            result['baseline_status']=br['status'];result['baseline_path_valid']=br['independent_path_validation']['valid']
    manager.close();result['path_validation']=validation;result['checks']=checks;result['wall_seconds']=time.perf_counter()-start
    write_json(out/'path.json',{k:result[k] for k in ['status','path','request_id','environment_version'] if k in result})
    write_json(out/'path_validation.json',validation);write_json(out/'result.json',result)
    return result


def summarize(results):
    counts={};valid_input=0;success=0;unsafe=0
    for r in results:
        counts[r['status']]=counts.get(r['status'],0)+1;valid_input+=r.get('input_geometry_valid',False)
        success+=r['status']=='SUCCESS';unsafe+=r['status']=='PATH_VALIDATION_FAILED'
    times=[r['stats']['build_cpu_ns']+r['stats']['plan_cpu_ns'] for r in results if r['status']=='SUCCESS']
    return dict(case_count=len(results),status_counts=counts,analytic_initially_valid_count=int(valid_input),certified_success_count=int(success),
                independently_rejected_successes=int(unsafe),successful_cpu_total_ns_median=float(np.median(times)) if times else None,
                successful_cpu_total_ns_p95=float(np.percentile(times,95)) if times else None,
                note='CPU timings are functional implementation measurements, not predicted silicon latency. Success/timeout counts are retained.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',choices=['smoke','modules','integration','system','regression','all'],default='smoke')
    parser.add_argument('--config',type=Path,default=ROOT/'spec/design_v1.json')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--samples-per-primitive',type=int,default=10000)
    parser.add_argument('--limit',type=int)
    parser.add_argument('--baseline',action='store_true')
    parser.add_argument('--state-budget',type=int)
    parser.add_argument('--iterations',type=int)
    args=parser.parse_args()
    if args.samples_per_primitive<=0:parser.error('sample count must be positive')
    config=load_config(args.config)
    if args.state_budget:config['state_evaluation_budget']=args.state_budget
    if args.iterations is not None:config['rrt_max_iterations']=args.iterations
    out=args.output or ROOT/'runs'/f"{time.strftime('%Y%m%d_%H%M%S')}_{args.suite}"
    out.mkdir(parents=True,exist_ok=True);overall=True;unit={};results=[];system_results=[];regression_results=[]
    write_json(out/'execution.json',dict(arguments=vars(args),config=config,started_unix=time.time(),status='RUNNING'))
    if args.suite in ('smoke','modules','all'):
        unit['modules']=run_tests(out,'test_modules.py');overall &= unit['modules']['valid']
    if args.suite in ('smoke','integration','all'):
        unit['integration']=run_tests(out,'test_integration.py');overall &= unit['integration']['valid']
    if not overall:
        write_json(out/'summary.json',dict(valid=False,unit=unit))
        write_json(out/'execution.json',dict(arguments=vars(args),config=config,status='FAILED_UNIT_CHECKS',completed_unix=time.time()))
        return 1
    if args.suite in ('smoke','system','all'):
        robot=RobotReference();cases=synthetic_cases(robot)
        for folder in sorted((ROOT/'examples').iterdir()):cases.append(load_problem(folder/'problem.json'))
        if args.limit:cases=cases[:args.limit]
        for p in cases:
            case_id=f"{p['scenario']}_{p['source_index']:04d}";points=sample_cloud(p,args.samples_per_primitive)
            r=run_case(p,points,config,out/'system'/case_id,baseline=args.baseline,trace=True,deep_reference=True);results.append(r);system_results.append(r)
            print(case_id,r['status'],f"{r['wall_seconds']:.3f}s",flush=True)
            overall &= r['status'] not in ('PATH_VALIDATION_FAILED','INTERNAL_ERROR')
            if r.get('checks',{}).get('rrt_trace',{}).get('status')!='INCOMPLETE_TRACE':overall &= r.get('checks',{}).get('rrt_trace',{}).get('valid',True)
    if args.suite in ('regression','all'):
        index=json.loads((ROOT/'data/panda_mbm/index.json').read_text())
        if args.limit:index=index[:args.limit]
        for i,entry in enumerate(index):
            p=load_problem(ROOT/entry['problem_path']);points=sample_cloud(p,args.samples_per_primitive)
            case_id=f"{p['scenario']}_{p['source_index']:04d}"
            r=run_case(p,points,config,out/'regression'/case_id,baseline=args.baseline);results.append(r);regression_results.append(r)
            overall &= r['status'] not in ('PATH_VALIDATION_FAILED','INTERNAL_ERROR')
            if (i+1)%10==0:print(f"regression {i+1}/{len(index)} {case_id} {r['status']}",flush=True)
            if (i+1)%50==0:write_json(out/'progress.json',summarize(results))
    summary=dict(valid=bool(overall),unit=unit,system=summarize(system_results),regression=summarize(regression_results),combined=summarize(results),completed_unix=time.time())
    write_json(out/'summary.json',summary)
    execution=json.loads((out/'execution.json').read_text(encoding='utf-8'))
    write_json(out/'execution.json',{**execution,'status':'COMPLETE' if overall else 'FAILED_VALIDATION','completed_unix':summary['completed_unix']})
    with (out/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
        keys=['case_id','status','input_geometry_valid','wall_seconds','kept_points','build_cpu_ns','plan_cpu_ns','peak_sram_bytes','distance_tests','state_calls','iterations','baseline_status','baseline_path_valid']
        writer=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore');writer.writeheader()
        for r in results:writer.writerow({**{k:v for k,v in r.items() if k!='stats'},**r.get('stats',{})})
    (out/'report.md').write_text('# SoC 軟體驗證結果\n\n'+f"必要正確性檢查 {'PASS' if overall else 'FAIL'}。此狀態不代表規劃成功率或硬體效益達標。\n\n"+'```json\n'+json.dumps({k:summary[k] for k in ['system','regression']},ensure_ascii=False,indent=2)+'\n```\n\n所有成功路徑均由獨立球模型與解析障礙物驗證器檢查。預算用完與保守拒絕保留在結果中；CPU 時間不代表晶片速度。\n',encoding='utf-8')
    print(json.dumps(summary['combined'],ensure_ascii=False,indent=2),flush=True)
    print('RESULT_DIRECTORY',out,flush=True)
    return 0 if overall else 1


if __name__=='__main__':
    try:sys.exit(main())
    except Exception:
        traceback.print_exc();sys.exit(2)
