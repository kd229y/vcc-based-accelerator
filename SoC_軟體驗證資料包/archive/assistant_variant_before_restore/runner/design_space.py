"""Memory-capacity and conditional timing sweeps; no invented device energy coefficients."""
from pathlib import Path
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import Model,load_config
from runner.data import sample_cloud,load_problem
from runner.run_validation import write_json
from simulation.timing import simulate_trace,estimate_energy


def main():
    out=ROOT/'runs/design_space_v1';out.mkdir(parents=True,exist_ok=True);capacity=[]
    for folder in sorted((ROOT/'examples').iterdir()):
        points=sample_cloud(load_problem(folder/'problem.json'),10000)
        for hash_capacity in [8192,16384,32768]:
            for sram in [512*1024,1024*1024,2*1024*1024,4*1024*1024]:
                with Model(load_config(hash_capacity=hash_capacity,sram_budget_bytes=sram)) as m:
                    status=m.build(points);stats=m.stats()
                    capacity.append(dict(case=folder.name,hash_capacity=hash_capacity,sram_budget_bytes=sram,status=status,
                                         full_resident_peak_bytes=stats['peak_sram_bytes'],kept_points=stats['kept_points'],hash_probes=stats['hash_probes'],max_hash_probe=stats['max_hash_probe']))
    source=ROOT/'runs/release_v1/system/synthetic_detour_0000'
    stats=json.loads((source/'result.json').read_text())['stats'];trace=np.load(source/'trace_i64.npy')
    base=json.loads((ROOT/'simulation/timing_assumptions.json').read_text());coeff=json.loads((ROOT/'simulation/energy_coefficients.json').read_text());timing=[]
    for storage in ['resident_sram','external_cached']:
        for cache in ([0] if storage=='resident_sram' else [4096,32768,262144]):
            for lanes in [1,4,8]:
                p={**base,'environment_storage':storage,'query_cache_bytes':cache or base['query_cache_bytes'],'distance_lanes':lanes}
                result=simulate_trace(trace,stats,p)
                timing.append(dict(storage=storage,cache_bytes=cache,distance_lanes=lanes,**result,energy=estimate_energy(stats,result,coeff)))
    write_json(out/'capacity_sweep.json',capacity);write_json(out/'timing_sweep.json',timing)
    print('capacity configurations',len(capacity),'timing configurations',len(timing),flush=True)


if __name__=='__main__':main()
