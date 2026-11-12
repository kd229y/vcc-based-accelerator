"""Independent specification and high-precision references. No native model calls."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def morton(x,y,z):
    bits=''.join(f'{z:07b}'[i]+f'{y:07b}'[i]+f'{x:07b}'[i] for i in range(7))
    return int(bits,2)


def build_reference(points,selector_bits=4,raw_coverage=False,max_range=None):
    if raw_coverage:raise ValueError('Raw voxel coverage is not part of the restored user specification')
    p=np.trunc(np.asarray(points,np.float32).astype(np.float64)*16384).astype(np.int64)
    if not np.isfinite(np.asarray(points)).all():
        raise ValueError('Input outside defined workspace')
    buckets={}
    for index,point in enumerate(p):
        if max_range is not None and sum(int(v)**2 for v in point)>=math.ceil(max_range*16384)**2:continue
        if (point<-32768).any() or (point>=32768).any():continue
        grid=(point+32768)//512
        key=morton(*map(int,grid))
        sub=((point+32768)%512)//(512//(1<<selector_bits))
        d=int(sum((2*int(v)+1-(1<<selector_bits))**2 for v in sub))
        if key not in buckets:
            buckets[key]=[point.copy(),d,index,point.copy(),point.copy()]
        else:
            entry=buckets[key]
            entry[3]=np.minimum(entry[3],point);entry[4]=np.maximum(entry[4],point)
            if d<entry[1]:entry[0],entry[1],entry[2]=point.copy(),d,index
    rows=[]
    for index,(key,(point,d,source,lo,hi)) in enumerate(sorted(buckets.items())):
        ds=sum(max(abs(int(a)-int(c)),abs(int(b)-int(c)))**2 for a,b,c in zip(lo,hi,point))
        cover=2
        rows.append([key,*map(int,point),d,source,cover,key>>6,index,1])
    return np.asarray(rows,dtype=np.int64).reshape((-1,10))


def brute_collision(snapshot,center,radius,point_radius=41):
    c=np.asarray(center,dtype=np.int64)
    if not len(snapshot):return False
    d=np.sum((snapshot[:,1:4]-c)**2,axis=1)
    r=np.int64(radius)+point_radius+snapshot[:,6]
    return bool(np.any(d<=r*r))


def qround(n,d):return (int(n)+int(d)//2)//int(d)


def raw_point_collision(points,center,radius,point_radius=.0025):
    """Direct float64 distances to every original binary32 point, no voxel or quantization.

    The caller must separately define culling and sensor/solid-geometry coverage.
    This oracle describes a union of point-radius spheres, not an occupied solid.
    """
    p=np.asarray(points,dtype=np.float32).astype(np.float64)
    c=np.asarray(center,dtype=np.float64)
    if p.ndim!=2 or p.shape[1]!=3 or c.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(c).all():
        raise ValueError('Invalid raw point query')
    if not math.isfinite(radius) or not math.isfinite(point_radius) or min(radius,point_radius)<0:
        raise ValueError('Invalid radius')
    return bool(np.any(np.sum((p-c)**2,axis=1)<=(radius+point_radius)**2))


def sincos_integer(angle):
    z=int(angle)*16;pi=843314857;half=421657428;sign=1
    if z>half:z-=pi;sign=-1
    elif z<-half:z+=pi;sign=-1
    x,y=652032874,0
    for i in range(30):
        step=round(math.atan(2.**-i)*(1<<28))
        direction=1 if z>=0 else -1
        x,y=x-direction*(y>>i),y+direction*(x>>i)
        z-=direction*step
    return sign*y,sign*x


class RobotReference:
    def __init__(self):
        model=ET.parse(ROOT/'upstream/vamp-vcc/resources/panda/panda_spherized.urdf').getroot()
        self.links=[x.attrib['name'] for x in model.findall('link')]
        self.joints=[];self.spheres=[]
        for j in model.findall('joint'):
            origin=j.find('origin');a=origin.attrib if origin is not None else {}
            xyz=np.array(list(map(float,a.get('xyz','0 0 0').split())))
            roll,pitch,yaw=map(float,a.get('rpy','0 0 0').split())
            cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
            rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
            ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
            rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
            t=np.eye(4);t[:3,:3]=rz@ry@rx;t[:3,3]=xyz
            index=int(j.attrib['name'][-1])-1 if j.attrib['type']=='revolute' else -1
            limit=j.find('limit')
            self.joints.append((j.find('parent').attrib['link'],j.find('child').attrib['link'],index,t,
                                [float(limit.attrib['lower']),float(limit.attrib['upper'])] if index>=0 else None))
        for link in model.findall('link'):
            for c in link.findall('collision'):
                sphere=c.find('geometry/sphere')
                if sphere is not None:
                    origin=c.find('origin')
                    xyz=list(map(float,origin.attrib.get('xyz','0 0 0').split())) if origin is not None else [0,0,0]
                    self.spheres.append((link.attrib['name'],np.array([*xyz,1.]),float(sphere.attrib['radius'])))
        srdf=ET.parse(ROOT/'upstream/vamp-vcc/resources/panda/panda.srdf').getroot()
        allowed={tuple(sorted((x.attrib['link1'],x.attrib['link2']))) for x in srdf.findall('disable_collisions')}
        self.pairs=np.array([(i,j) for i,(a,_,_) in enumerate(self.spheres) for j,(b,_,_) in enumerate(self.spheres[:i])
                             if a!=b and tuple(sorted((a,b))) not in allowed],dtype=int)
        self.radii=np.array([x[2] for x in self.spheres])
        self.limits=np.array([next(j[4] for j in self.joints if j[2]==i) for i in range(7)])
        distances={self.links[0]:0.}
        for parent,child,_,t,_ in self.joints:distances[child]=distances[parent]+np.linalg.norm(t[:3,3])
        self.maximum_chain_length=max(distances[name]+np.linalg.norm(p[:3]) for name,p,_ in self.spheres)
        if self.maximum_chain_length>=2:raise ValueError('2 m movement bound invalid')
        parent_joints={child:(parent,index,t) for parent,child,index,t,_ in self.joints}
        self.reach=np.zeros((len(self.spheres),7));ancestors=np.zeros_like(self.reach,dtype=bool)
        for si,(name,p,_) in enumerate(self.spheres):
            length=float(np.linalg.norm(p[:3]));link=name
            while link in parent_joints:
                parent,index,t=parent_joints[link]
                if index>=0:self.reach[si,index]=length;ancestors[si,index]=True
                length+=float(np.linalg.norm(t[:3,3]));link=parent
        i,j=self.pairs.T
        # Shared ancestor rotations are a common rigid transform and do not change pair distance.
        self.pair_reach=self.reach[i]*(~ancestors[j])+self.reach[j]*(~ancestors[i])
    def fk(self,q):
        q=np.asarray(q,float);transforms={self.links[0]:np.eye(4)}
        for parent,child,index,t,_ in self.joints:
            if index>=0:
                c,s=math.cos(q[index]),math.sin(q[index]);turn=np.eye(4);turn[:2,:2]=[[c,-s],[s,c]]
                transforms[child]=transforms[parent]@t@turn
            else:transforms[child]=transforms[parent]@t
        return np.array([(transforms[name]@p)[:3] for name,p,_ in self.spheres])
    def integer_fk(self,q):
        # Python arbitrary precision arithmetic independently interprets the specified fixed-point matrices.
        scale=1<<30
        eye=[[scale,0,0],[0,scale,0],[0,0,scale]]
        transforms={self.links[0]:(eye,[0,0,0])}
        def matmul(a,b):return [[qround(sum(int(a[i][k])*int(b[k][j]) for k in range(3)),scale) for j in range(3)] for i in range(3)]
        for parent,child,index,t,_ in self.joints:
            r=[[round(float(t[i,j])*scale) for j in range(3)] for i in range(3)]
            local_t=[round(float(v)*(1<<24)) for v in t[:3,3]]
            if index>=0:
                s,c=sincos_integer(q[index]);r=matmul(r,[[c,-s,0],[s,c,0],[0,0,scale]])
            pr,pt=transforms[parent]
            transforms[child]=(matmul(pr,r),[pt[i]+qround(sum(pr[i][k]*local_t[k] for k in range(3)),scale) for i in range(3)])
        result=[]
        for name,p,_ in self.spheres:
            r,t=transforms[name];local=[round(float(v)*(1<<24)) for v in p[:3]]
            result.append([qround(t[i]+qround(sum(r[i][k]*local[k] for k in range(3)),scale),1<<10) for i in range(3)])
        return np.asarray(result,dtype=np.int32)
    def self_clearance(self,centers):
        return float(np.min(self.self_clearances(centers)))
    def self_clearances(self,centers):
        i,j=self.pairs.T
        return np.linalg.norm(centers[i]-centers[j],axis=1)-self.radii[i]-self.radii[j]
    def primitive_clearance(self,centers,problem,point_radius=.0025):
        return float(np.min(self.primitive_clearances(centers,problem,point_radius)))
    def primitive_clearances(self,centers,problem,point_radius=.0025):
        minimum=np.full(len(centers),math.inf)
        for primitive in problem['primitives']:
            t=np.array(primitive['world_from_local']);p=(centers-t[:3,3])@t[:3,:3]
            if primitive['type']=='box':
                d=np.abs(p)-np.array(primitive['dimensions'])/2
                sdf=np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0)
            elif primitive['type']=='cylinder':
                h,r=primitive['dimensions'];d=np.column_stack((np.linalg.norm(p[:,:2],axis=1)-r,np.abs(p[:,2])-h/2))
                sdf=np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0)
            elif primitive['type']=='sphere':sdf=np.linalg.norm(p,axis=1)-primitive['dimensions'][0]
            else:raise ValueError('Unsupported geometry')
            minimum=np.minimum(minimum,sdf-self.radii-point_radius)
        return minimum
    def clearance(self,q,problem):
        p=self.fk(q)
        return self.primitive_clearance(p,problem),self.self_clearance(p)
    def valid_state(self,q,problem):
        q=np.asarray(q)
        if (q<self.limits[:,0]).any() or (q>self.limits[:,1]).any():return False
        e,s=self.clearance(q,problem);return e>0 and s>0


def validate_path(path,problem,robot=None,max_depth=18):
    """Certify continuous interpolation using clearance and a 2 m joint reach bound.

    Scope: the pinned sphere robot versus analytic solid primitives and SRDF self pairs.
    This does not assert physical robot or sensor-model completeness.
    """
    robot=robot or RobotReference();p=np.asarray(path,dtype=float)
    if p.ndim!=2 or p.shape[1]!=7 or len(p)==0 or not np.isfinite(p).all():return dict(valid=False,reason='INVALID_PATH_FORMAT')
    tol=1/(1<<24)
    if np.max(np.abs(p[0]-problem['start']))>tol:return dict(valid=False,reason='START_MISMATCH')
    if not any(np.max(np.abs(p[-1]-g))<=tol for g in problem['goals']):return dict(valid=False,reason='GOAL_MISMATCH')
    if (p<robot.limits[:,0]-tol).any() or (p>robot.limits[:,1]+tol).any():return dict(valid=False,reason='JOINT_LIMIT')
    poses=0;min_clearance=math.inf;cache={}
    def clearance(q):
        nonlocal poses,min_clearance
        key=tuple(q)
        if key not in cache:
            centers=robot.fk(q);cache[key]=(robot.primitive_clearances(centers,problem),robot.self_clearances(centers));poses+=1;min_clearance=min(min_clearance,float(np.min(cache[key][0])),float(np.min(cache[key][1])))
        return cache[key]
    def interval(a,b,depth):
        mid=(a+b)/2;e,s=clearance(mid)
        if np.any(e<=0) or np.any(s<=0):return 'COLLISION'
        half_delta=np.abs(b-a)/2
        environment_motion=robot.reach@half_delta;self_motion=robot.pair_reach@half_delta
        if np.all(e>environment_motion+1e-9) and np.all(s>self_motion+1e-9):return None
        if depth>=max_depth:return 'CONTINUOUS_CERTIFICATE_UNRESOLVED'
        return interval(a,mid,depth+1) or interval(mid,b,depth+1)
    for q in p:
        e,s=clearance(q)
        if np.any(e<=0) or np.any(s<=0):return dict(valid=False,reason='WAYPOINT_COLLISION',poses_checked=poses)
    for i in range(len(p)-1):
        reason=interval(p[i],p[i+1],0)
        if reason:return dict(valid=False,reason=reason,edge_index=i,poses_checked=poses)
    return dict(valid=True,reason='CERTIFIED_SPHERE_ROBOT_ANALYTIC_SCENE',poses_checked=poses,
                min_sampled_clearance_m=min_clearance,reach_bound_m=2.,certificate='per-sphere descendant-chain bounds; self-pair common ancestor rigid transforms cancel',maximum_model_chain_m=robot.maximum_chain_length,
                path_length_rad=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()))
