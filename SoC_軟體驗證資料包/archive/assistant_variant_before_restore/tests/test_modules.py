import ctypes as ct
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from runner.model import Model,VCCBaseline,load_config,joint_codes,array_pointer,IP,LP
from reference.oracles import RobotReference,build_reference,brute_collision,morton,sincos_integer,validate_path,raw_point_collision

READY=np.array([0,-.785,0,-2.356,0,1.571,.785])


class Modules(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.robot=RobotReference();cls.rng=np.random.default_rng(5021)
    def test_01_morton_exhaustive_and_hierarchy(self):
        with Model() as m:
            for x in range(16):
                for y in range(16):
                    for z in range(16):
                        k=m.lib.soc_morton(x,y,z)
                        self.assertEqual(k,morton(x,y,z))
                        self.assertEqual(k>>6,morton(x//4,y//4,z//4))
            self.assertEqual(m.lib.soc_morton(5,3,6),371)
            self.assertEqual(m.lib.soc_morton(127,127,127),(1<<21)-1)
    def test_02_ieee_ingress_boundary_and_nonfinite(self):
        values=[0.,-0.,-1e-40,1e-40,-2.,np.nextafter(np.float32(2),np.float32(0)),.03125,-.03125]
        values+=self.rng.uniform(-2,2,2000).astype(np.float32).tolist()
        with Model() as m:
            for x in values:
                status=ct.c_int();actual=m.lib.soc_quantize(np.float32(x),ct.byref(status))
                self.assertEqual(status.value,0);self.assertEqual(actual,math.floor(float(np.float32(x))*8192))
            for x in [math.nan,math.inf,-math.inf]:
                status=ct.c_int();m.lib.soc_quantize(x,ct.byref(status));self.assertEqual(status.value,5)
            self.assertEqual(m.build([[math.nan,0,0]]),'INVALID_INPUT')
            with self.assertRaises(RuntimeError):m.query([0,0,0],.1)
    def test_03_filter_sort_offsets_and_coverage(self):
        p=self.rng.uniform(-.8,.8,(9000,3)).astype(np.float32)
        p=np.concatenate([p,p[:300],np.array([[0,0,0],[-.00001,0,0],[.03125,0,0]],np.float32)])
        with Model(load_config(sram_budget_bytes=4*1024*1024,raw_point_coverage=True,max_range_m=None)) as m:
            self.assertEqual(m.build(p),'SUCCESS');snap=m.snapshot();ref=build_reference(p)
            np.testing.assert_array_equal(snap,ref)
            expected=np.bincount(ref[:,7],minlength=32768)
            np.testing.assert_array_equal(m.offsets(),np.r_[0,np.cumsum(expected)])
            quant=np.floor(p.astype(float)*8192).astype(int)
            lookup={int(r[0]):r for r in snap}
            for point in quant:
                r=lookup[morton(*map(int,(point+16384)//256))]
                self.assertLessEqual(np.linalg.norm(point-r[1:4]),r[6])
    def test_04_ties_and_input_order(self):
        p=np.array([[.002,.002,.002],[.0021,.0021,.0021],[.002,.002,.002]],np.float32)
        with Model() as m:
            self.assertEqual(m.build(p),'SUCCESS');self.assertEqual(int(m.snapshot()[0,5]),0)
    def test_05_query_against_brute_force(self):
        p=self.rng.uniform(-.5,.5,(5000,3)).astype(np.float32)
        with Model() as m:
            self.assertEqual(m.build(p),'SUCCESS');snap=m.snapshot()
            cases=[(np.array([0,0,0]),0),(np.array([0,0,0]),5000),(np.array([-20000,0,0]),100),(snap[0,1:4],0)]
            cases += [(self.rng.integers(-15000,15000,3),int(self.rng.integers(0,3500))) for _ in range(1000)]
            for c,r in cases:self.assertEqual(m.query(c,r,codes=True),brute_collision(snap,c,r))
    def test_06_empty_and_explicit_capacity(self):
        with Model() as m:
            self.assertEqual(m.build(np.empty((0,3))),'SUCCESS');self.assertFalse(m.query([0,0,0],1))
        with Model(load_config(hash_capacity=8)) as m:
            self.assertEqual(m.build(np.array([[i*.04,0,0] for i in range(10)])),'CAPACITY_EXCEEDED')
        with Model(load_config(sram_budget_bytes=100)) as m:self.assertEqual(m.build([[0,0,0]]),'CAPACITY_EXCEEDED')
    def test_07_cordic_numeric_and_bit_reference(self):
        with Model() as m:
            for angle in self.rng.uniform(-math.pi,math.pi,1000):
                q=int(np.floor(angle*(1<<24)+.5));out=np.empty(2,np.int64)
                m.lib.soc_sincos(q,array_pointer(out,LP));self.assertEqual(tuple(out),sincos_integer(q))
                np.testing.assert_allclose(out/(1<<30),[math.sin(q/(1<<24)),math.cos(q/(1<<24))],atol=2e-7,rtol=0)
    def test_08_fk_bits_and_independent_geometry(self):
        maximum=0.
        with Model() as m:
            for i in range(100):
                q=self.rng.uniform(self.robot.limits[:,0],self.robot.limits[:,1]);codes=joint_codes(q)
                actual=m.fk(codes,codes=True);np.testing.assert_array_equal(actual,self.robot.integer_fk(codes))
                maximum=max(maximum,float(np.max(np.linalg.norm(actual/8192-self.robot.fk(codes/(1<<24)),axis=1))))
            self.assertLess(maximum,.001)
    def test_09_upstream_fk_independent_set_comparison(self):
        with Model() as m:
            for _ in range(10):
                q=self.rng.uniform(self.robot.limits[:,0],self.robot.limits[:,1]);up=m.upstream_fk(q);own=self.robot.fk(q)
                distance=np.linalg.norm(own[:,None,:]-up[None,:,:3],axis=2)+np.abs(self.robot.radii[:,None]-up[None,:,3])
                self.assertLess(float(np.max(np.min(distance,axis=0))),1e-6)
                self.assertLess(float(np.max(np.min(distance,axis=1))),1e-6)
    def test_10_nearest_and_ties(self):
        with Model() as m:
            nodes=self.rng.integers(-50000000,50000000,(400,7),dtype=np.int32)
            for _ in range(100):
                q=self.rng.integers(-50000000,50000000,7,dtype=np.int32)
                expected=int(np.argmin(np.sum((nodes.astype(np.int64)-q)**2,axis=1)))
                self.assertEqual(m.lib.soc_nearest(array_pointer(nodes,IP),len(nodes),array_pointer(q,IP)),expected)
            q=nodes[0].copy();nodes[1]=q
            self.assertEqual(m.lib.soc_nearest(array_pointer(nodes,IP),len(nodes),array_pointer(q,IP)),0)
    def test_11_zero_motion_and_path_fault_detection(self):
        problem=dict(start=READY.tolist(),goals=[READY.tolist()],primitives=[])
        with Model() as m:
            self.assertEqual(m.build(np.empty((0,3))),'SUCCESS');result=m.plan(READY,READY)
            self.assertEqual(result['status'],'SUCCESS');self.assertTrue(validate_path(result['path'],problem,self.robot)['valid'])
            broken=np.array(result['path']);broken[0,0]+=.1
            self.assertFalse(validate_path(broken,problem,self.robot)['valid'])
    def test_12_depth_input_and_invalid_pixel(self):
        depth=np.array([[1000,0,2000],[1000,1000,1000]],dtype=np.uint16)
        camera=np.array([100*65536,100*65536,65536,0,1000],dtype=np.int64)
        transform=np.zeros((3,4),dtype=np.int64);transform[:,:3]=np.eye(3,dtype=np.int64)*(1<<30)
        out=np.empty((6,3),np.int32)
        with Model() as m:
            n=m.lib.soc_depth(depth.ctypes.data_as(ct.POINTER(ct.c_uint16)),3,2,array_pointer(camera,LP),array_pointer(transform,LP),array_pointer(out,IP))
            expected=[]
            for v in range(2):
                for u in range(3):
                    d=int(depth[v,u])
                    if d:
                        z=(d*8192+500)//1000
                        expected.append([((u-1)*z+50)//100,(v*z+50)//100,z])
            np.testing.assert_array_equal(out[:n],expected)
    def test_13_original_vcc_kernel_queries(self):
        p=self.rng.uniform(-.5,.5,(4000,3)).astype(np.float32)
        with VCCBaseline(p) as b:
            filtered=b.points();self.assertFalse(b.stats()['overflow_possible'])
            self.assertGreaterEqual(b.stats()['allocated_point_pool_bytes'],len(filtered)*3*4)
            for _ in range(200):
                c=self.rng.uniform(-.7,.7,3).astype(np.float32);r=float(self.rng.uniform(.001,.25))
                expected=bool(np.any(np.sum((filtered.astype(np.float64)-c.astype(np.float64))**2,axis=1)<=(r+.0025)**2))
                self.assertEqual(b.query(c,r),expected)
    def test_14_reference_comparator_detects_corruption(self):
        p=np.array([[.1,.2,.3],[.4,.5,.6]],np.float32)
        with Model() as m:
            m.build(p);snap=m.snapshot();broken=snap.copy();broken[0,0]^=1
            with self.assertRaises(AssertionError):np.testing.assert_array_equal(broken,build_reference(p))
            with self.assertRaises(AssertionError):self.assertEqual(False,brute_collision(snap,snap[0,1:4],0))
    def test_15_depth_to_complete_plan(self):
        goal=READY.copy();goal[0]=.2
        p=dict(start=READY.tolist(),goals=[goal.tolist()],primitives=[])
        with Model() as m:
            self.assertEqual(m.build_depth(np.full((4,4),1000,np.uint16),[100,100,1.5,1.5]),'SUCCESS')
            result=m.plan(READY,goal)
            self.assertEqual(result['status'],'SUCCESS');self.assertTrue(validate_path(result['path'],p,self.robot)['valid'])
            self.assertEqual(m.depth_metadata['input_bytes'],32)
    def test_16_large_input_culling_and_api_shapes(self):
        with Model() as m:
            self.assertEqual(m.build([[240000,240000,240000]]),'SUCCESS')
            self.assertEqual(m.stats()['culled_points'],1);self.assertEqual(len(m.snapshot()),0)
            with self.assertRaises(ValueError):m.fk([1,2,3],codes=True)
            with self.assertRaises(ValueError):m.query([math.nan,0,0],.1)
            with self.assertRaises(ValueError):m.query_many([[math.nan,0,0]],[.1])
            with self.assertRaises(ValueError):m.query_many([[0,0,0]],[math.inf])
            with self.assertRaises(ValueError):m.query_many([[100,0,0]],[.1])
    def test_17_complete_original_vcc_planner(self):
        from runner.data import planning_samples
        goal=READY.copy();goal[0]=.2
        with VCCBaseline(np.empty((0,3),np.float32)) as b:
            result=b.plan(READY,goal,planning_samples(1,self.robot.limits,20,goal=goal),iterations=20,maxnodes=100)
            self.assertEqual(result['status'],'SUCCESS')
            problem=dict(start=READY.astype(np.float32).astype(float).tolist(),goals=[goal.astype(np.float32).astype(float).tolist()],primitives=[])
            self.assertTrue(validate_path(result['path'],problem,self.robot)['valid'])
    def test_18_native_batches_match_scalar(self):
        from runner.compare_variants import FloatMorton
        p=self.rng.uniform(-.5,.5,(200,3)).astype(np.float32);c=self.rng.uniform(-.6,.6,(100,3)).astype(np.float32);r=np.full(100,.08,np.float32)
        with Model() as m:
            m.build(p);actual,duration=m.query_many(c,r)
            np.testing.assert_array_equal(actual,[m.query(a,float(b)) for a,b in zip(c,r)]);self.assertGreater(duration,0)
        with VCCBaseline(p) as m:
            actual,duration=m.query_many(c,r);np.testing.assert_array_equal(actual,[m.query(a,float(b)) for a,b in zip(c,r)])
        f=FloatMorton(p)
        try:
            actual,duration=f.query_many(c,r);np.testing.assert_array_equal(actual,[f.query(a,float(b)) for a,b in zip(c,r)])
        finally:f.close()
    def test_19_floating_query_center_cannot_lose_raw_collision(self):
        # Before the host ingress fix this single-point query was a false negative
        # with about 5.46 um penetration; both scalar and batch paths must catch it.
        points=np.full((1,3),(1-1e-6)/8192,np.float32)
        center=np.full(3,(25+.500001)/8192);radius=22/8192
        self.assertTrue(raw_point_collision(points,center,radius))
        with Model() as m:
            self.assertEqual(m.build(points),'SUCCESS')
            self.assertTrue(m.query(center,radius))
            hits,_=m.query_many([center],[radius]);self.assertTrue(hits[0])
            # Exact code inputs retain their original bit-level query semantics.
            snap=m.snapshot();c=np.array([26,26,26],np.int32)
            self.assertEqual(m.query(c,22,codes=True),brute_collision(snap,c,22))
            with self.assertRaises(ValueError):m.query(center,65535/8192)
            with self.assertRaises(ValueError):m.query_many([center],[65535/8192])
    def test_20_known_filtered_path_is_rejected_by_geometry_gate(self):
        import json
        fixture=json.loads((ROOT/'tests/fixtures/unsafe_filtered_detour.json').read_text(encoding='utf-8'))
        self.assertTrue(self.robot.valid_state(fixture['problem']['start'],fixture['problem']))
        self.assertTrue(self.robot.valid_state(fixture['problem']['goals'][0],fixture['problem']))
        validation=validate_path(fixture['path'],fixture['problem'],self.robot)
        self.assertFalse(validation['valid'])
        self.assertIn(validation['reason'],['COLLISION','WAYPOINT_COLLISION'])


if __name__=='__main__':unittest.main(verbosity=2)
