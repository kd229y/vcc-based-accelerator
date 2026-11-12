"""Conservative fixed-point error and intermediate-width bounds for the frozen v1 model."""
from pathlib import Path
import json
import math
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from reference.oracles import RobotReference


def main():
    robot=RobotReference();rotation_scale=1<<30;position_scale=1<<24;coordinate_scale=16384;angle_scale=1<<24
    # CORDIC: table/angle error, arithmetic-shift error amplified by total gain,
    # initial gain rounding and residual last micro-rotation.
    angle_error=(31*.5+1)/(1<<28)+math.atan(2.**-29)
    shift_error=1.647*math.sqrt(2)*30/rotation_scale
    cordic_bound=angle_error+shift_error+1.647/(2*rotation_scale)
    if cordic_bound>2e-7:raise AssertionError('Configured CORDIC bound insufficient')
    round_matrix=1.5/rotation_scale;round_vector=math.sqrt(3)/(2*position_scale)
    errors={robot.links[0]:(0.,0.)}
    for parent,child,index,t,_ in robot.joints:
        re,te=errors[parent]
        local_rotation=1.5/rotation_scale
        if index>=0:local_rotation=local_rotation*(1+2*cordic_bound)+2*cordic_bound+round_matrix
        new_rotation=re*(1+local_rotation)+local_rotation+round_matrix
        new_translation=te+re*float((t[:3,3]@t[:3,3])**.5)+(1+re)*round_vector+round_vector
        errors[child]=(new_rotation,new_translation)
    maximum=0.
    for name,p,_ in robot.spheres:
        re,te=errors[name]
        maximum=max(maximum,te+re*float((p[:3]@p[:3])**.5)+(1+re)*round_vector+round_vector)
    output_round=math.sqrt(3)/(2*coordinate_scale)
    angle_input=7*2/(2*angle_scale)
    interpolation_round=angle_input
    total=maximum+output_round+angle_input+interpolation_round+1e-10
    config=json.loads((ROOT/'spec/design_v1.json').read_text())
    margin=math.ceil(config['fk_margin_m']*coordinate_scale)/coordinate_scale
    if total>=margin:raise AssertionError('FK margin does not cover derived bound')
    report=dict(status='BOUND_CHECK_PASS',scope='Pinned sphere URDF; Q30 rotations, Q24 internal positions and angles, Q14 output; no physical model uncertainty',
        cordic_sin_cos_absolute_bound=cordic_bound,fk_internal_position_bound_m=maximum,output_rounding_bound_m=output_round,
        angle_ingress_and_interpolation_bound_m=angle_input+interpolation_round,total_fk_center_bound_m=total,configured_fk_margin_m=margin,
        maximum_chain_length_m=float(robot.maximum_chain_length),global_joint_reach_bound_m=2.,
        raw_point_ingress_bound_m=math.sqrt(3)/16384,raw_coverage_extra_units=2,
        maximum_rep_coverage_units=2,
        matrix_accumulator_abs_bound=math.ceil(3*(rotation_scale*(1+1e-4))**2),
        int64_max=(1<<63)-1,
        notes=['Restored user profile keeps representatives only; no discarded-point bbox or voxel coverage.',
               'A radius is rounded upward. Self-collision inflates both sphere radii.',
               'Edges use VAMP rake samples, not continuous interval certificates. Independent geometry checks remain external to the planner.'])
    cull_required=robot.maximum_chain_length+max(robot.radii)+margin+config['point_radius_m']+2/16384
    report['conservative_cull_required_m']=float(cull_required)
    report['configured_cull_m']=config['max_range_m']
    report['cull_bound_sufficient']=bool(config['max_range_m']>=cull_required)
    report['cull_scope']='Restored 1.19m source VCC culling; not a guarantee to retain every point reachable by the full robot chain'
    if report['matrix_accumulator_abs_bound']>=report['int64_max']:raise AssertionError('Matrix accumulator overflow')
    path=ROOT/'reports/numeric_bounds.json';path.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
