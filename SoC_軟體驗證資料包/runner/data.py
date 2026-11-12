from pathlib import Path
import hashlib
import json

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def load_problem(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def sample_cloud(problem,n=10000,seed=0):
    """Vectorized uniform-area surface samples, shared unchanged by all compared variants."""
    rng=np.random.default_rng(seed);parts=[]
    for shape in problem['primitives']:
        dims=shape['dimensions']
        if shape['type']=='box':
            dx,dy,dz=dims;weights=np.array([dy*dz,dy*dz,dx*dz,dx*dz,dx*dy,dx*dy]);weights/=weights.sum()
            sides=rng.choice(6,n,p=weights);p=rng.uniform(-.5,.5,(n,3))*dims
            for side in range(6):p[sides==side,side//2]=dims[side//2]/2*(1 if side%2==0 else -1)
        elif shape['type']=='cylinder':
            h,r=dims;weights=np.array([np.pi*r*r,2*np.pi*r*h,np.pi*r*r]);weights/=weights.sum()
            sides=rng.choice(3,n,p=weights);angle=rng.uniform(-np.pi,np.pi,n)
            rho=np.where(sides==1,r,r*np.sqrt(rng.random(n)));z=rng.uniform(-h/2,h/2,n)
            z[sides==0]=-h/2;z[sides==2]=h/2;p=np.column_stack((rho*np.cos(angle),rho*np.sin(angle),z))
        elif shape['type']=='sphere':
            z=rng.uniform(-1,1,n);angle=rng.uniform(-np.pi,np.pi,n);rho=np.sqrt(1-z*z)
            p=np.column_stack((rho*np.cos(angle),rho*np.sin(angle),z))*dims[0]
        else:raise ValueError('Unsupported primitive')
        t=np.asarray(shape['world_from_local']);parts.append(p@t[:3,:3].T+t[:3,3])
    return np.ascontiguousarray(np.concatenate(parts) if parts else np.empty((0,3)),dtype=np.float32)


def digest_array(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def planning_samples(seed,limits,n,goal=None):
    state=int(seed)|1;result=[]
    lo=np.ceil(limits[:,0]*(1<<24)).astype(np.int64);hi=np.floor(limits[:,1]*(1<<24)).astype(np.int64)
    for iteration in range(n):
        q=[]
        for j in range(7):
            state^=(state<<13)&0xffffffff;state^=state>>17;state^=(state<<5)&0xffffffff;state&=0xffffffff
            q.append((int(lo[j])+((state*int(hi[j]-lo[j]))>>32))/(1<<24))
        if goal is not None and iteration%10==0:q=(np.floor(np.asarray(goal)*(1<<24)+.5)/(1<<24)).tolist()
        result.append(q)
    return np.asarray(result,dtype=np.float32)


def synthetic_cases(robot):
    ready=np.array([0,-.785,0,-2.356,0,1.571,.785])
    free=dict(schema='soc-panda-input-v1',robot='panda',scenario='synthetic_free',source_index=0,
              joint_names=[f'panda_joint{i}' for i in range(1,8)],start=ready.tolist(),
              goals=[(ready+np.array([.2,0,0,0,0,0,0])).tolist()],primitives=[])
    start=ready.copy();start[0]=-.9;goal=ready.copy();goal[0]=.9
    center=robot.fk(ready)[-1]
    t=np.eye(4);t[:3,3]=center
    detour=dict(**{**free,'scenario':'synthetic_detour','start':start.tolist(),'goals':[goal.tolist()],
                  'primitives':[dict(type='box',dimensions=[.08,.08,.08],world_from_local=t.tolist(),object_id='blocking_box')]})
    return [free,detour]
