"""Export bit-accurate software input/expected-output vectors, not RTL implementation."""
from pathlib import Path
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from runner.model import Model,load_config,joint_codes
from runner.data import synthetic_cases,sample_cloud
from runner.run_validation import write_json,export_environment
from reference.oracles import RobotReference,brute_collision


def main():
    out=ROOT/'rtl_vectors/user_original_v1';out.mkdir(parents=True,exist_ok=True);robot=RobotReference();problem=synthetic_cases(robot)[1];points=sample_cloud(problem,10000)
    points.astype('<f4').tofile(out/'input_points_f32.bin');rng=np.random.default_rng(449)
    with Model() as m:
        if m.build(points)!='SUCCESS':raise AssertionError('Vector environment failed')
        export_environment(m,out/'environment');snapshot=m.snapshot()
        centers=rng.integers(-15000,15000,(512,3),dtype=np.int32);radii=rng.integers(0,3000,512,dtype=np.int32)
        centers[:min(100,len(snapshot))]=snapshot[:100,1:4];radii[:100]=0
        expected=np.array([brute_collision(snapshot,c,int(r)) for c,r in zip(centers,radii)],np.uint8)
        actual=np.array([m.query(c,int(r),codes=True) for c,r in zip(centers,radii)],np.uint8);np.testing.assert_array_equal(actual,expected)
        (centers+32768).astype('<u2').tofile(out/'query_centers_q14_u16.bin');radii.astype('<u2').tofile(out/'query_radii_q14_u16.bin');expected.tofile(out/'query_expected_u8.bin')
        q=np.array([joint_codes(rng.uniform(robot.limits[:,0],robot.limits[:,1])) for _ in range(128)],np.int32)
        fk_expected=np.array([robot.integer_fk(v) for v in q],np.int32)
        for v,answer in zip(q,fk_expected):np.testing.assert_array_equal(m.fk(v,codes=True),answer)
        q.astype('<i4').tofile(out/'fk_input_q24_i32.bin');(fk_expected+32768).astype('<u2').tofile(out/'fk_expected_q14_u16.bin')
        roundtrip=np.fromfile(out/'query_centers_q14_u16.bin',dtype='<u2').astype(np.int32).reshape((-1,3))-32768;np.testing.assert_array_equal(roundtrip,centers)
    write_json(out/'manifest.json',dict(scope='Software golden vectors only; RTL and synthesis are not implemented',byte_order='little',query_count=512,
        fk_case_count=128,spheres_per_fk=59,query_center_format='unsigned int16 workspace-relative Q14 xyz; world=(code-32768)/16384',query_radius_format='unsigned uint16 Q14',
        fk_input_format='signed int32 Q24 seven angles (software model internal format)',fk_output_format='unsigned int16 workspace-relative Q14 59 xyz centers',seed=449,config=load_config(),problem=problem))
    print('RTL software vectors exported',out,flush=True)


if __name__=='__main__':main()
