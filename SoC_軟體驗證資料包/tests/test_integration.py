from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import Model,load_config
from simulation.system import EnvironmentManager,Busy
from simulation.timing import Memory,simulate_trace,estimate_energy
import json


class Integration(unittest.TestCase):
    def test_double_buffer_versions_and_backpressure(self):
        m=EnvironmentManager();self.assertEqual(m.prepare([[.1,0,0]]),'SUCCESS');self.assertEqual(m.publish(),1)
        a=m.submit([.1,0,0],.001,delay=20)
        self.assertEqual(m.prepare([[.8,0,0]]),'SUCCESS');self.assertEqual(m.publish(),2)
        b=m.submit([.1,0,0],.001,delay=5)
        with self.assertRaises(Busy):m.prepare([[.2,0,0]])
        m.advance(5);self.assertFalse(m.responses[b]['hit']);self.assertNotIn(a,m.responses)
        m.advance(20);self.assertTrue(m.responses[a]['hit']);self.assertEqual(m.responses[a]['environment_version'],1)
        self.assertEqual(m.responses[b]['environment_version'],2);self.assertEqual(m.prepare([[.2,0,0]]),'SUCCESS');m.publish();m.close()
    def test_cancel_drains_physical_requests_once(self):
        m=EnvironmentManager();m.prepare([[0,0,0]]);m.publish();q=m.submit([0,0,0],.01,delay=10)
        self.assertTrue(m.cancel(q));self.assertFalse(m.cancel(q));m.prepare([[1,0,0]]);m.publish()
        with self.assertRaises(Busy):m.prepare([[.2,0,0]])
        m.drain();self.assertEqual(len(m.responses),1);self.assertEqual(m.responses[q]['status'],'CANCELLED');m.close()
    def test_failed_build_does_not_publish(self):
        m=EnvironmentManager(load_config(hash_capacity=8));m.prepare([[0,0,0]]);m.publish()
        self.assertEqual(m.prepare([[i*.04,0,0] for i in range(10)]),'CAPACITY_EXCEEDED')
        with self.assertRaises(Busy):m.publish()
        q=m.submit([0,0,0],0);m.drain();self.assertTrue(m.responses[q]['hit']);m.close()
    def test_order_variation_and_no_loss(self):
        for delays in ([1,2,3,4],[4,1,3,2],[2,2,2,2]):
            m=EnvironmentManager();m.prepare([[0,0,0]]);m.publish()
            ids=[m.submit([i*.5,0,0],.01,d) for i,d in enumerate(delays)];m.drain()
            self.assertEqual(set(ids),set(m.responses));self.assertEqual(sum(r['hit'] for r in m.responses.values()),1);m.close()
    def test_memory_cache_and_bank_busy(self):
        p=json.loads((ROOT/'simulation/timing_assumptions.json').read_text());p['query_cache_bytes']=64;p['environment_storage']='external_cached'
        m=Memory(p);t=m.access(0,8,0,0);u=m.access(0,8,0,t)
        self.assertEqual(m.counts['external_reads'],1);self.assertGreater(u,t)
        m.access(64,8,0,u);m.access(0,8,0,u+1);self.assertEqual(m.counts['external_reads'],3)
    def test_timing_missing_trace_and_unknown_energy(self):
        p=json.loads((ROOT/'simulation/timing_assumptions.json').read_text())
        self.assertEqual(simulate_trace([],{'trace_dropped':1},p)['status'],'INCOMPLETE_TRACE')
        stats=dict(input_points=10,hash_probes=11,distance_tests=2,nn_distance_tests=3,fk_calls=1,logical_read_bytes=64,logical_write_bytes=32)
        coefficients=json.loads((ROOT/'simulation/energy_coefficients.json').read_text())
        self.assertIsNone(estimate_energy(stats,{},coefficients)['energy_j'])
        filled={k:1 for k in coefficients if k!='source'};filled['source']='unit-test-only'
        e=estimate_energy(stats,{'estimated_seconds':2},filled);self.assertGreater(e['energy_j'],2)
    def test_online_timing_matches_replay_and_preserves_path(self):
        q=np.array([0,-.785,0,-2.356,0,1.571,.785]);goal=q.copy();goal[0]=.2
        p=json.loads((ROOT/'simulation/timing_assumptions.json').read_text())
        with Model() as m:
            self.assertEqual(m.build(np.empty((0,3))),'SUCCESS');m.set_trace(100000);scheduler=m.attach_timing(p)
            first=m.plan(q,goal);self.assertEqual(first['status'],'SUCCESS')
            replay=simulate_trace(m.trace(),m.stats(),p)
            self.assertEqual(scheduler.result()['estimated_cycles'],replay['estimated_cycles'])
        with Model() as m:
            m.build(np.empty((0,3)));second=m.plan(q,goal)
            self.assertEqual(first['joint_codes'],second['joint_codes'])


if __name__=='__main__':unittest.main(verbosity=2)
