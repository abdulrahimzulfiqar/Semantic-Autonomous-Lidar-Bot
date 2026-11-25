"""Paper-inspired 2D reference; NOT released AGLoc/AGLoc++.

No raster, ground truth, AMCL pose, ROS publisher, or navigation action input.
Original Eq.8 weights; deterministic weighting approximates Eq.10 downsampling.
Odometry prediction/null-space retention are declared adaptations, not the
original odometry-free tracker or the 2026 adaptive Kalman fusion implementation.
"""
import math
import time
from pathlib import Path
import sys
import numpy as np


def wrap(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],wrap(a[2]+b[2])])


def relative(a,b):
    c,s=math.cos(a[2]),math.sin(a[2]);x,y=np.asarray(b[:2])-a[:2]
    return np.array([c*x+s*y,-s*x+c*y,wrap(b[2]-a[2])])


def paper_weights(signed_distance):
    d=np.asarray(signed_distance,dtype=float);w=np.zeros_like(d)
    inside=(d>-1)&(d<=0);outside=(d>0)&(d<3)
    w[inside]=1/(1+1.5*np.abs(d[inside]));w[outside]=1/(1+3*d[outside])
    return w


def cross(a,b):
    return a[...,0]*b[...,1]-a[...,1]*b[...,0]


class StructuralMap:
    def __init__(self,segments,doors=()):
        self.segments=self.valid_segments(segments)
        self.doors=self.valid_segments(doors,empty_ok=True)
        if not len(self.segments):raise ValueError('empty_structural_map')

    @staticmethod
    def valid_segments(value,empty_ok=False):
        a=np.asarray(value,dtype=float)
        if not a.size and empty_ok:return np.empty((0,2,2))
        if a.ndim!=3 or a.shape[1:]!=(2,2) or not np.isfinite(a).all():raise ValueError('invalid_segments')
        if np.any(np.linalg.norm(a[:,1]-a[:,0],axis=1)<1e-8):raise ValueError('zero_length_segment')
        return a

    @classmethod
    def from_osm(cls,path):
        # Reuse only the accepted vector parser. No PaperPlanner/PGM is loaded.
        root=Path(__file__).resolve().parents[3]
        sys.path.insert(0,str(root/'scripts'))
        from osmag_rolling_raster import load_osm_geometry
        from osmag_constraint_mask import _lines
        from shapely.ops import unary_union
        from shapely.geometry import LineString
        leaves,passages,_,_=load_osm_geometry(Path(path))
        bounds=unary_union([p.boundary for p in leaves.values()])
        if passages:bounds=bounds.difference(unary_union([LineString(p) for p in passages.values()]).buffer(1e-7))
        segments=[(a,b) for line in _lines(bounds) for a,b in zip(line,line[1:]) if math.dist(a,b)>1e-8]
        return cls(segments,list(passages.values()))

    @staticmethod
    def intersect(segments,origin,directions):
        if not len(segments):return np.full(len(directions),np.inf),np.zeros(len(directions),dtype=int)
        v=segments[:,1]-segments[:,0];q=segments[:,0]-origin
        den=cross(directions[:,None,:],v[None,:,:]);valid=np.abs(den)>1e-10
        safe=np.where(valid,den,1.)
        t=cross(q,v)[None,:]/safe;u=cross(q[None,:,:],directions[:,None,:])/safe
        t=np.where(valid&(t>1e-5)&(u>=-1e-8)&(u<=1+1e-8),t,np.inf)
        index=np.argmin(t,axis=1)
        return t[np.arange(len(t)),index],index

    def correspond(self,pose,points,angles,ranges,extrinsic,door_tolerance):
        sensor=compose(pose,extrinsic)
        directions=np.column_stack([np.cos(angles+sensor[2]),np.sin(angles+sensor[2])])
        expected,index=self.intersect(self.segments,sensor[:2],directions)
        segments=self.segments[index].copy()
        if len(self.doors):
            distance,ident=self.intersect(self.doors,sensor[:2],directions)
            closed=np.isfinite(distance)&(distance<=expected+1e-7)&(np.abs(distance-ranges)<=door_tolerance)
            expected[closed]=distance[closed];segments[closed]=self.doors[ident[closed]]
        vector=segments[:,1]-segments[:,0];normal=np.column_stack([-vector[:,1],vector[:,0]])
        normal/=np.linalg.norm(normal,axis=1)[:,None]
        c,s=math.cos(pose[2]),math.sin(pose[2]);rot=np.array([[c,-s],[s,c]])
        rotated=points@rot.T;world=rotated+pose[:2]
        residual=np.sum(normal*(world-segments[:,0]),axis=1)
        signed=np.where(ranges<=expected,-np.abs(residual),np.abs(residual))
        weights=paper_weights(signed);weights[~np.isfinite(expected)]=0
        orientations=np.mod(np.arctan2(vector[:,1],vector[:,0]),math.pi)
        bins=np.minimum((orientations/(math.pi/36)).astype(int),35)
        matched=weights>0;hist=np.bincount(bins[matched],minlength=36)
        dominant=int(np.argmax(hist));corr=float(hist[dominant]/max(1,np.sum(hist)))
        rate=max(1.,10*corr-4)
        weights[(bins==dominant)&matched]/=rate
        jac=np.column_stack([normal,np.sum(normal*np.column_stack([-rotated[:,1],rotated[:,0]]),axis=1)])
        return residual,weights,jac,segments,normal,corr


class Tracker:
    def __init__(self,model,initial_pose,config):
        self.model=model;self.pose=np.asarray(initial_pose,dtype=float);self.config=dict(config)
        if self.pose.shape!=(3,) or not np.isfinite(self.pose).all():raise ValueError('invalid_initial_pose')
        self.last_odom=None;self.last_stamp=None;self.rejected=0

    def update(self,stamp,angles,ranges,odom_pose,extrinsic,range_min,range_max):
        begin=time.perf_counter();cfg=self.config
        angles=np.asarray(angles,dtype=float);ranges=np.asarray(ranges,dtype=float);odom_pose=np.asarray(odom_pose,dtype=float)
        if not math.isfinite(stamp) or (self.last_stamp is not None and stamp<=self.last_stamp):raise ValueError('nonmonotonic_scan')
        if angles.ndim!=1 or ranges.shape!=angles.shape or not np.isfinite(angles).all():raise ValueError('invalid_scan_geometry')
        if odom_pose.shape!=(3,) or not np.isfinite(odom_pose).all() or len(extrinsic)!=3 or not np.isfinite(extrinsic).all():raise ValueError('invalid_pose_input')
        if not 0<range_min<range_max:raise ValueError('invalid_range_limits')
        gap=self.last_stamp is not None and stamp-self.last_stamp>cfg['maximum_scan_gap_s']
        predicted=self.pose.copy() if self.last_odom is None else compose(self.pose,relative(self.last_odom,odom_pose))
        self.last_stamp=stamp;self.last_odom=odom_pose.copy()
        keep=np.isfinite(ranges)&(ranges>=range_min)&(ranges<range_max)
        angles=angles[keep];ranges=ranges[keep]
        sensor_points=np.column_stack([ranges*np.cos(angles),ranges*np.sin(angles)])
        c,s=math.cos(extrinsic[2]),math.sin(extrinsic[2]);points=sensor_points@np.array([[c,s],[-s,c]])+extrinsic[:2]
        pose=predicted.copy();accepted=False;reason='scan_gap' if gap else 'too_few_returns';corr=0.;ratio=0.;rms=None;count=0
        if not gap and len(points)>=cfg['minimum_matches']:
            for iteration in range(cfg['maximum_iterations']):
                residual,weights,jac,segments,normal,corr=self.model.correspond(pose,points,angles,ranges,extrinsic,cfg['door_tolerance_m'])
                good=weights>0;count=int(good.sum())
                if count<cfg['minimum_matches']:reason='too_few_structural_matches';break
                h=jac.T@(weights[:,None]*jac);g=jac.T@(weights*residual)
                eigen,vectors=np.linalg.eigh(h);observable=eigen>max(1e-9,eigen[-1]*cfg['rank_relative_threshold'])
                step=-vectors[:,observable]@((vectors[:,observable].T@g)/eigen[observable])
                factor=max(1.,np.linalg.norm(step[:2])/cfg['iteration_translation_limit_m'],abs(step[2])/cfg['iteration_yaw_limit_rad']);step/=factor
                old=float(np.sum(weights*residual**2));improved=False
                for alpha in [1.,.5,.25,.125,.0625]:
                    candidate=pose+alpha*step;candidate[2]=wrap(candidate[2]);c,s=math.cos(candidate[2]),math.sin(candidate[2])
                    world=points@np.array([[c,s],[-s,c]])+candidate[:2]
                    updated=np.sum(normal*(world-segments[:,0]),axis=1)
                    if float(np.sum(weights*updated**2))<=old+1e-10:pose=candidate;improved=True;break
                if not improved or np.linalg.norm(step)<1e-5:break
            residual,weights,jac,_,_,corr=self.model.correspond(pose,points,angles,ranges,extrinsic,cfg['door_tolerance_m'])
            count=int(np.sum(weights>0));rms=float(np.sqrt(np.sum(weights*residual**2)/max(weights.sum(),1e-9)))
            xy=jac[:,:2].T@(weights[:,None]*jac[:,:2]);vals=np.linalg.eigvalsh(xy);ratio=float(max(0,vals[0])/max(vals[-1],1e-9))
            correction=relative(predicted,pose)
            accepted=(count>=cfg['minimum_matches'] and rms<=cfg['maximum_residual_rms_m'] and np.linalg.norm(correction[:2])<=cfg['maximum_correction_m'] and abs(correction[2])<=cfg['maximum_correction_rad'])
            reason=('weak_geometry' if ratio<cfg['weak_geometry_ratio'] else 'structural_match') if accepted else 'match_rejected'
        self.pose=pose if accepted else predicted;self.rejected=0 if accepted else self.rejected+1
        return dict(stamp=float(stamp),pose=self.pose.tolist(),accepted=bool(accepted),reason=reason,
            lost=self.rejected>=cfg['lost_after_rejected_frames'],matches=count,returns=len(points),corridorness=corr,
            translation_observability_ratio=ratio,residual_rms_m=rms,wall_s=time.perf_counter()-begin,
            scope='2D paper-inspired reference, odometry prediction; no global rescue or author-runtime equivalence')
