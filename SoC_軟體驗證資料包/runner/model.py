"""Typed host adapter for the C++ integer functional model."""
from pathlib import Path
import ctypes as ct
import json
import math
import os
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
COORDINATE_SCALE = 1 << 14
STAT_NAMES = 'input_points kept_points filter_updates hash_probes max_hash_probe radix_reads radix_writes queries global_rejects cells_visited cell_rejects distance_tests fk_calls nn_calls nn_distance_tests edge_calls edge_samples rrt_nodes iterations sram_read_requests sram_write_requests logical_read_bytes logical_write_bytes build_cpu_ns query_cpu_ns fk_cpu_ns plan_cpu_ns peak_sram_bytes trace_dropped state_calls culled_points'.split()
STATUSES = ['SUCCESS','BUDGET_EXHAUSTED','INVALID_START','INVALID_GOAL','CAPACITY_EXCEEDED','INVALID_INPUT','INTERNAL_ERROR']
IP = ct.POINTER(ct.c_int32)
LP = ct.POINTER(ct.c_int64)
FP = ct.POINTER(ct.c_float)
DP = ct.POINTER(ct.c_double)
BP = ct.POINTER(ct.c_uint8)
UP = ct.POINTER(ct.c_uint32)


def load_config(path=None, **overrides):
    p = Path(path) if path else ROOT/'spec/design_v1.json'
    c = json.loads(p.read_text(encoding='utf-8'))
    c.update(overrides)
    return c


def array_pointer(a, dtype):
    return a.ctypes.data_as(dtype)


def joint_codes(q):
    values = np.asarray(q, dtype=np.float64)
    if values.shape != (7,) or not np.isfinite(values).all() or (np.abs(values)>4).any():
        raise ValueError('Expected seven finite joint angles in radians within +/-4')
    return np.floor(values*(1<<24)+0.5).astype(np.int32)


def native_library():
    if os.name == 'nt':
        os.add_dll_directory(r'C:\mingw64\bin') if Path(r'C:\mingw64\bin').is_dir() else None
    report_path=ROOT/'build/build_report.json'
    report=json.loads(report_path.read_text()) if report_path.exists() else {}
    binary=ROOT/report['core_binary'] if 'core_binary' in report else ROOT/'build'/('soc_model.dll' if os.name == 'nt' else 'libsoc_model.so')
    lib = ct.CDLL(str(binary))
    signatures = {
        'soc_create': ([LP],ct.c_void_p), 'soc_destroy': ([ct.c_void_p],None),
        'soc_set_version': ([ct.c_void_p,ct.c_int64],None),
        'soc_trace_config': ([ct.c_void_p,ct.c_int64,ct.c_int],None),
        'soc_set_callback': ([ct.c_void_p,ct.c_void_p,ct.c_void_p],None),
        'soc_build': ([ct.c_void_p,FP,ct.c_int],ct.c_int),
        'soc_query': ([ct.c_void_p,IP,ct.c_int],ct.c_int),
        'soc_query_many': ([ct.c_void_p,IP,IP,ct.c_int,BP],ct.c_int64),
        'soc_plan': ([ct.c_void_p,IP,IP],ct.c_int),
        'soc_stats': ([ct.c_void_p,LP],None),
        'soc_snapshot': ([ct.c_void_p,LP],ct.c_int),
        'soc_offsets': ([ct.c_void_p,UP],None),
        'soc_cells': ([ct.c_void_p,LP],ct.c_int),
        'soc_path': ([ct.c_void_p,IP],ct.c_int),
        'soc_trace': ([ct.c_void_p,LP],ct.c_int),
        'soc_fk': ([IP,IP],None), 'soc_upstream_fk': ([DP,DP],None),
        'soc_morton': ([ct.c_int,ct.c_int,ct.c_int],ct.c_uint32),
        'soc_quantize': ([ct.c_float,ct.POINTER(ct.c_int)],ct.c_int),
        'soc_sincos': ([ct.c_int,LP],None),
        'soc_nearest': ([IP,ct.c_int,IP],ct.c_int),
        'soc_depth': ([ct.POINTER(ct.c_uint16),ct.c_int,ct.c_int,LP,LP,IP],ct.c_int)
    }
    for name,(args,result) in signatures.items():
        f=getattr(lib,name);f.argtypes=args;f.restype=result
    return lib


class Model:
    def __init__(self, config=None):
        self.config = config or load_config()
        c=self.config
        for k,v in [('coordinate_fraction_bits',14),('coordinate_bits',16),('angle_fraction_bits',24),('rotation_fraction_bits',30),('fine_voxel_m',.03125),('coarse_voxel_m',.125)]:
            if c[k] != v:
                raise ValueError(f'v1 fixes {k}={v}; changing it requires a new numeric version')
        if c['workspace_min_m'] != [-2,-2,-2] or c['workspace_max_m'] != [2,2,2]:
            raise ValueError('v1 workspace is [-2,2)^3')
        if not 1<=c['hash_capacity']<=1<<21 or c['hash_capacity']&(c['hash_capacity']-1):
            raise ValueError('hash_capacity must be a bounded power of two')
        if c.get('raw_point_coverage'):
            raise ValueError('Restored user model does not include the assistant raw-voxel coverage variant')
        if c.get('edge_validation_mode')!='vamp_rake' or c.get('edge_rake')!=8 or c.get('edge_resolution',0)<=0:
            raise ValueError('Restored profile uses VAMP rake=8 and an explicit resolution')
        if c['rrt_max_nodes_per_tree']<1 or c['rrt_range_rad']<=0:
            raise ValueError('Invalid planner capacity or step')
        self.lib=native_library()
        self.cfg=np.array([c['hash_capacity'],c['selector_bits'],int(c['raw_point_coverage']),c['rrt_max_nodes_per_tree'],
                           math.floor(c['rrt_range_rad']*(1<<24)),c['edge_rake'],
                           math.ceil(c['fk_margin_m']*COORDINATE_SCALE),math.ceil(c['point_radius_m']*COORDINATE_SCALE),c['sram_budget_bytes'],
                           c['seed'],c['trace_capacity'],c['state_evaluation_budget'],c['rrt_max_iterations'],int(c.get('speculative_sphere_batch',False)),math.ceil(c['max_range_m']*COORDINATE_SCALE) if c.get('max_range_m') else 0,c['edge_resolution'],c.get('goal_bias_interval',0)],dtype=np.int64)
        self.handle=self.lib.soc_create(array_pointer(self.cfg,LP))
        if not self.handle:
            raise MemoryError('Native allocation failed')
        self.build_status=None
    def close(self):
        if self.handle:
            self.lib.soc_destroy(self.handle);self.handle=None
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def build(self, points):
        p=np.ascontiguousarray(points,dtype=np.float32)
        if p.ndim!=2 or p.shape[1]!=3:
            raise ValueError('Expected Nx3 point cloud')
        code=self.lib.soc_build(self.handle,array_pointer(p,FP),len(p))
        self.build_status=STATUSES[code]
        return self.build_status
    def query(self, center, radius, *, codes=False):
        if self.build_status!='SUCCESS':raise RuntimeError('Environment not built successfully')
        values=np.asarray(center,dtype=np.float64)
        if values.shape!=(3,) or not np.isfinite(values).all() or (np.abs(values)>(32767 if codes else 32767/COORDINATE_SCALE)).any():raise ValueError('Unsupported center')
        c=np.ascontiguousarray(values if codes else np.trunc(values*COORDINATE_SCALE),dtype=np.int32)
        if not np.isfinite(radius) or radius<0 or radius* (1 if codes else COORDINATE_SCALE)>65535:
            raise ValueError('Unsupported query')
        # Moving the query center can increase its distance from an obstacle.
        # Q14 truncation moves each axis by < 1 unit; ceil(sqrt(3))=2.
        # Point ingress coverage alone does not cover this independent error.
        center_margin=0 if codes else 2*int(np.any(values*COORDINATE_SCALE!=c))
        r=(int(radius) if codes else math.ceil(radius*COORDINATE_SCALE))+center_margin
        if r>65535:raise ValueError('Query exceeds v1 radius after center error allowance')
        result=self.lib.soc_query(self.handle,array_pointer(c,IP),r)
        if result<0:raise RuntimeError('Native query error')
        return bool(result)
    def build_depth(self,depth,intrinsics,world_from_camera=None,depth_units_per_m=1000):
        d=np.asarray(depth)
        if d.ndim!=2 or d.dtype!=np.uint16:raise ValueError('Expected uint16 HxW depth image, zero means invalid')
        if depth_units_per_m<=0:raise ValueError('Invalid depth scale')
        fx,fy,cx,cy=map(float,intrinsics)
        if min(fx,fy)<=0 or not np.isfinite([fx,fy,cx,cy]).all():raise ValueError('Invalid intrinsics')
        t=np.eye(4) if world_from_camera is None else np.asarray(world_from_camera,dtype=np.float64)
        if t.shape!=(4,4) or not np.isfinite(t).all():raise ValueError('Invalid camera transform')
        camera=np.array([round(v*65536) for v in [fx,fy,cx,cy]]+[int(depth_units_per_m)],dtype=np.int64)
        transform=np.zeros((3,4),np.int64);transform[:,:3]=np.floor(t[:3,:3]*(1<<30)+.5).astype(np.int64);transform[:,3]=np.floor(t[:3,3]*COORDINATE_SCALE+.5).astype(np.int64)
        image=np.ascontiguousarray(d);out=np.empty((d.size,3),np.int32);start=time.perf_counter_ns()
        n=self.lib.soc_depth(image.ctypes.data_as(ct.POINTER(ct.c_uint16)),d.shape[1],d.shape[0],array_pointer(camera,LP),array_pointer(transform,LP),array_pointer(out,IP))
        if n<0:raise ValueError(f'Depth conversion failed: {n}')
        self.depth_metadata=dict(input_bytes=int(d.nbytes),valid_pixel_count=n,projection_cpu_ns=time.perf_counter_ns()-start,
                                 camera_q16=camera.tolist(),transform_q30_q14=transform.tolist(),
                                 adapter_note='Q14 coordinates are losslessly repacked as binary32 for the common build ABI; native projection and quantization use integer arithmetic.')
        return self.build((out[:n].astype(np.float32)/COORDINATE_SCALE).astype(np.float32))
    def fk(self,q,*,codes=False):
        a=np.ascontiguousarray(q,dtype=np.int32) if codes else joint_codes(q)
        if a.shape!=(7,) or (np.abs(a.astype(np.int64))>4*(1<<24)).any():raise ValueError('Expected seven supported angle codes')
        out=np.empty((59,3),dtype=np.int32)
        self.lib.soc_fk(array_pointer(a,IP),array_pointer(out,IP))
        return out
    def query_many(self,centers,radii):
        if self.build_status!='SUCCESS':raise RuntimeError('Environment not built successfully')
        values=np.asarray(centers,dtype=np.float64);radius=np.asarray(radii,dtype=np.float64)
        if values.ndim!=2 or values.shape[1]!=3 or radius.shape!=(len(values),) or not np.isfinite(values).all() or not np.isfinite(radius).all():raise ValueError('Invalid query batch')
        if (np.abs(values)>32767/COORDINATE_SCALE).any() or (radius<0).any() or (radius*COORDINATE_SCALE>65535).any():raise ValueError('Unsupported query batch')
        c=np.ascontiguousarray(np.trunc(values*COORDINATE_SCALE),dtype=np.int32)
        center_margin=2*np.any(values*COORDINATE_SCALE!=c,axis=1).astype(np.int32)
        r=np.ascontiguousarray(np.ceil(radius*COORDINATE_SCALE)+center_margin,dtype=np.int32)
        if (r>65535).any():raise ValueError('Query batch exceeds v1 radius after center error allowance')
        out=np.empty(len(c),np.uint8);duration=self.lib.soc_query_many(self.handle,array_pointer(c,IP),array_pointer(r,IP),len(c),array_pointer(out,BP))
        return out.astype(bool),int(duration)
    def upstream_fk(self,q):
        a=np.ascontiguousarray(q,dtype=np.float64);out=np.empty((59,4),dtype=np.float64)
        if a.shape!=(7,) or not np.isfinite(a).all():raise ValueError('Expected seven finite angles')
        self.lib.soc_upstream_fk(array_pointer(a,DP),array_pointer(out,DP));return out
    def plan(self,start,goal):
        if self.build_status!='SUCCESS':raise RuntimeError('Environment not built successfully')
        a,b=joint_codes(start),joint_codes(goal)
        code=self.lib.soc_plan(self.handle,array_pointer(a,IP),array_pointer(b,IP))
        if getattr(self,'timing_error',None):raise RuntimeError(f'Timing callback failed: {self.timing_error}')
        n=self.lib.soc_path(self.handle,None)
        p=np.empty((n,7),dtype=np.int32)
        if n:self.lib.soc_path(self.handle,array_pointer(p,IP))
        return dict(status=STATUSES[code],joint_codes=p.tolist(),path=(p.astype(float)/(1<<24)).tolist(),stats=self.stats())
    def stats(self):
        out=np.zeros(len(STAT_NAMES),dtype=np.int64);self.lib.soc_stats(self.handle,array_pointer(out,LP))
        return dict(zip(STAT_NAMES,map(int,out)))
    def snapshot(self):
        n=self.lib.soc_snapshot(self.handle,None);out=np.empty((n,10),dtype=np.int64)
        if n:self.lib.soc_snapshot(self.handle,array_pointer(out,LP))
        return out
    def offsets(self):
        out=np.empty(32769,dtype=np.uint32);self.lib.soc_offsets(self.handle,array_pointer(out,UP));return out
    def cells(self):
        n=self.lib.soc_cells(self.handle,None);out=np.empty((n,9),dtype=np.int64)
        if n:self.lib.soc_cells(self.handle,array_pointer(out,LP))
        return out
    def trace(self):
        n=self.lib.soc_trace(self.handle,None);out=np.empty((n,8),dtype=np.int64)
        if n:self.lib.soc_trace(self.handle,array_pointer(out,LP))
        return out
    def set_trace(self,capacity,clear=True):
        self.lib.soc_trace_config(self.handle,capacity,int(clear))
    def attach_timing(self,profile):
        return self._attach_events(profile)
    def capture_control_trace(self):
        self._attach_events(None)
        return self.control_records
    def _attach_events(self,profile):
        from simulation.timing import OnlineTiming
        self.scheduler=OnlineTiming(self.stats(),profile) if profile is not None else None
        self.timing_error=None;self.control_records=[];self.callback_event_count=0
        control_kinds={4,7,32,33,34,35,38,39}
        callback_type=ct.CFUNCTYPE(None,LP,ct.c_void_p)
        def receive(row,_):
            try:
                self.callback_event_count+=1
                selected=int(row[0]) in control_kinds
                if self.scheduler is not None or selected:
                    values=[int(row[i]) for i in range(8)]
                    if selected:self.control_records.append(values)
                    if self.scheduler is not None:self.scheduler.consume(values)
            except Exception as error:self.timing_error=repr(error)
        self._timing_callback=callback_type(receive)
        self.lib.soc_set_callback(self.handle,ct.cast(self._timing_callback,ct.c_void_p),None)
        return self.scheduler


class VCCBaseline:
    def __init__(self,points,*,fine=.031,max_range=1.19,half=1.19,coarse_radius=.08):
        if os.name=='nt' and Path(r'C:\mingw64\bin').is_dir():os.add_dll_directory(r'C:\mingw64\bin')
        self.lib=ct.CDLL(str(ROOT/'build'/('vcc_baseline.dll' if os.name=='nt' else 'libvcc_baseline.so')))
        self.lib.vcc_create.argtypes=[FP,ct.c_int,ct.c_float,ct.c_float,ct.c_float,ct.c_float];self.lib.vcc_create.restype=ct.c_void_p
        self.lib.vcc_destroy.argtypes=[ct.c_void_p];self.lib.vcc_destroy.restype=None
        self.lib.vcc_points.argtypes=[ct.c_void_p,FP];self.lib.vcc_points.restype=ct.c_int
        self.lib.vcc_query.argtypes=[ct.c_void_p,FP,ct.c_float];self.lib.vcc_query.restype=ct.c_int
        self.lib.vcc_query_many.argtypes=[ct.c_void_p,FP,FP,ct.c_int,BP];self.lib.vcc_query_many.restype=ct.c_int64
        self.lib.vcc_stats.argtypes=[ct.c_void_p,LP];self.lib.vcc_stats.restype=None
        self.lib.vcc_plan.argtypes=[ct.c_void_p,FP,FP,FP,ct.c_int,ct.c_int,ct.c_int,ct.c_float,FP,LP];self.lib.vcc_plan.restype=ct.c_int
        p=np.ascontiguousarray(points,dtype=np.float32)
        self.handle=self.lib.vcc_create(array_pointer(p,FP),len(p),fine,max_range,half,coarse_radius)
        if not self.handle:raise RuntimeError('VCC baseline construction failed')
    def close(self):
        if self.handle:self.lib.vcc_destroy(self.handle);self.handle=None
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def points(self):
        n=self.lib.vcc_points(self.handle,None);out=np.empty((n,3),np.float32)
        if n:self.lib.vcc_points(self.handle,array_pointer(out,FP))
        return out
    def query(self,c,r):
        p=np.ascontiguousarray(c,np.float32);return bool(self.lib.vcc_query(self.handle,array_pointer(p,FP),r))
    def stats(self):
        out=np.empty(6,np.int64);self.lib.vcc_stats(self.handle,array_pointer(out,LP))
        # Upstream MVT point_coord_pool_size counts float32 elements, not bytes.
        out[3]*=np.dtype(np.float32).itemsize
        return {**dict(zip(['filter_cpu_ns','build_cpu_ns','overflow_possible','allocated_point_pool_bytes','allocated_hierarchy_bytes','voxel_storage_count'],map(int,out))),
                'point_pool_size_unit':'bytes'}
    def query_many(self,centers,radii):
        c,r=np.ascontiguousarray(centers,np.float32),np.ascontiguousarray(radii,np.float32);out=np.empty(len(c),np.uint8)
        duration=self.lib.vcc_query_many(self.handle,array_pointer(c,FP),array_pointer(r,FP),len(c),array_pointer(out,BP));return out.astype(bool),int(duration)
    def plan(self,start,goal,samples,iterations=300,maxnodes=COORDINATE_SCALE,range_rad=.35):
        a,b,s=map(lambda x:np.ascontiguousarray(x,dtype=np.float32),(start,goal,samples))
        path=np.empty((maxnodes+4,7),np.float32);stats=np.zeros(3,np.int64)
        code=self.lib.vcc_plan(self.handle,array_pointer(a,FP),array_pointer(b,FP),array_pointer(s,FP),len(s),iterations,maxnodes,range_rad,array_pointer(path,FP),array_pointer(stats,LP))
        return dict(status=STATUSES[code] if 0<=code<len(STATUSES) else 'INTERNAL_ERROR',path=path[:int(stats[2])].tolist(),planning_cpu_ns=int(stats[0]),iterations=int(stats[1]))
