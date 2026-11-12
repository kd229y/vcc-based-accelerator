"""CPU kernel ablations A/B/C/D on identical saved input and query arrays."""
from pathlib import Path
import ctypes as ct
import json
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import Model,VCCBaseline,load_config,array_pointer,FP,LP,BP
from runner.data import load_problem,sample_cloud,digest_array
from runner.run_validation import write_json


class FloatMorton:
    def __init__(self,p):
        import os
        self.lib=ct.CDLL(str(ROOT/'build'/('float_morton.dll' if os.name=='nt' else 'libfloat_morton.so')))
        self.lib.float_create.argtypes=[FP,ct.c_int,ct.c_float];self.lib.float_create.restype=ct.c_void_p
        self.lib.float_destroy.argtypes=[ct.c_void_p];self.lib.float_destroy.restype=None
        self.lib.float_points.argtypes=[ct.c_void_p,FP];self.lib.float_points.restype=ct.c_int
        self.lib.float_query.argtypes=[ct.c_void_p,FP,ct.c_float];self.lib.float_query.restype=ct.c_int
        self.lib.float_stats.argtypes=[ct.c_void_p,LP];self.lib.float_stats.restype=None
        self.lib.float_query_many.argtypes=[ct.c_void_p,FP,FP,ct.c_int,BP];self.lib.float_query_many.restype=ct.c_int64
        self.handle=self.lib.float_create(array_pointer(p,FP),len(p),1.6)
        if not self.handle:raise MemoryError('Float Morton allocation')
    def close(self):self.lib.float_destroy(self.handle)
    def points(self):
        n=self.lib.float_points(self.handle,None);p=np.empty((n,3),np.float32);self.lib.float_points(self.handle,array_pointer(p,FP));return p
    def query(self,c,r):
        a=np.ascontiguousarray(c,np.float32);return bool(self.lib.float_query(self.handle,array_pointer(a,FP),r))
    def stats(self):
        p=np.empty(6,np.int64);self.lib.float_stats(self.handle,array_pointer(p,LP));return dict(zip(['filter_cpu_ns','sort_cpu_ns','table_cpu_ns','queries','distance_tests','logical_environment_bytes'],map(int,p)))
    def query_many(self,centers,radii):
        c,r=np.ascontiguousarray(centers,np.float32),np.ascontiguousarray(radii,np.float32);out=np.empty(len(c),np.uint8)
        duration=self.lib.float_query_many(self.handle,array_pointer(c,FP),array_pointer(r,FP),len(c),array_pointer(out,BP));return out.astype(bool),int(duration)


def main():
    out=ROOT/'runs/kernel_ablation_v1';out.mkdir(parents=True,exist_ok=True);results=[]
    for folder in sorted((ROOT/'examples').iterdir()):
        problem=load_problem(folder/'problem.json');points=sample_cloud(problem,10000);rng=np.random.default_rng(123)
        centers=rng.uniform(-1.3,1.3,(1000,3)).astype(np.float32);radii=rng.uniform(.01,.25,1000).astype(np.float32)
        row=dict(case=folder.name,input_sha256=digest_array(points),query_seed=123,query_count=len(centers),variants={})
        for label,kw in [('A_original_vcc',{}),('B_matched_grid_vcc',dict(fine=.03125,max_range=1.6,half=2.,coarse_radius=.1225))]:
            with VCCBaseline(points,**kw) as b:
                filtered=b.points();b.query_many(centers,radii);batches=[b.query_many(centers,radii) for _ in range(5)];hits=batches[-1][0].tolist();query_ns=int(np.median([v[1] for v in batches]))
                mismatches=0
                for c,r,h in zip(centers[:100],radii[:100],hits[:100]):
                    expected=bool(np.any(np.sum((filtered.astype(float)-c.astype(float))**2,axis=1)<=(float(r)+.0025)**2));mismatches+=expected!=h
                row['variants'][label]=dict(**b.stats(),query_batch_cpu_ns=query_ns,kept_points=len(filtered),query_hits=sum(hits),brute_mismatches_first100=mismatches)
        f=FloatMorton(points);filtered=f.points();f.query_many(centers,radii);batches=[f.query_many(centers,radii) for _ in range(5)];hits=batches[-1][0].tolist();query_ns=int(np.median([v[1] for v in batches]))
        mismatch=sum(bool(np.any(np.sum((filtered.astype(float)-c.astype(float))**2,axis=1)<=(float(r)+.0025)**2))!=h for c,r,h in zip(centers[:100],radii[:100],hits[:100]))
        row['variants']['C_float_morton']=dict(**f.stats(),query_batch_cpu_ns=query_ns,kept_points=len(filtered),query_hits=sum(hits),brute_mismatches_first100=mismatch);f.close()
        for label,coverage in [('D_integer_user_restored',False)]:
            with Model(load_config(raw_point_coverage=coverage)) as m:
                status=m.build(points)
                if status!='SUCCESS':row['variants'][label]=dict(status=status);continue
                m.query_many(centers,radii);batches=[m.query_many(centers,radii) for _ in range(5)];hits=batches[-1][0].tolist();query_ns=int(np.median([v[1] for v in batches]))
                row['variants'][label]=dict(**m.stats(),query_batch_cpu_ns=query_ns,query_hits=sum(hits))
        results.append(row);print(folder.name,{k:v.get('query_batch_cpu_ns') for k,v in row['variants'].items()},flush=True)
    write_json(out/'results.json',dict(results=results,note='CPU kernel microbenchmark. A/B use pinned VCC AVX2 with Windows allocation adapters and Eigen autovectorization disabled; C is scalar FP32 Morton. D is the instrumented integer model with optional coverage. Query times are native batch medians of five warm repeats, excluding Python per-query overhead. Different numeric/coverage semantics and SIMD widths are explicit; no silicon speedup is inferred.'))


if __name__=='__main__':main()
