#!/usr/bin/env python3
"""Conservative XY collision bounds from an expanded URDF, at zero joint positions.

Primitive collision geometry only. Mesh collisions or disconnected frames fail
explicitly. This is a model audit, not swept-path or costmap validation.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def origin(element):
    transform=np.eye(4)
    if element is None:return transform
    transform[:3,3]=[float(v) for v in element.get('xyz','0 0 0').split()]
    r,p,y=[float(v) for v in element.get('rpy','0 0 0').split()]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    transform[:3,:3]=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return transform


def audit(path,radius=.18):
    robot=ET.parse(path).getroot();frames={'base_footprint':np.eye(4)}
    pending=list(robot.findall('joint'))
    while pending:
        ready=[j for j in pending if j.find('parent').get('link') in frames]
        if not ready:raise ValueError('Disconnected URDF frames')
        for joint in ready:
            frames[joint.find('child').get('link')]=frames[joint.find('parent').get('link')]@origin(joint.find('origin'))
            pending.remove(joint)
    records=[];all_points=[]
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            geometry=collision.find('geometry');box=geometry.find('box');sphere=geometry.find('sphere');cylinder=geometry.find('cylinder')
            if box is not None:half=np.array([float(v)/2 for v in box.get('size').split()]);kind='box'
            elif sphere is not None:half=np.full(3,float(sphere.get('radius')));kind='sphere (conservative enclosing box)'
            elif cylinder is not None:half=np.array([float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2]);kind='cylinder (conservative enclosing box)'
            else:raise ValueError('Unsupported collision geometry in '+link.get('name'))
            if link.get('name') not in frames:raise ValueError('Collision frame unavailable')
            transform=frames[link.get('name')]@origin(collision.find('origin'))
            corners=np.array([list(np.asarray(sign)*half)+[1] for sign in itertools.product([-1,1],repeat=3)])
            points=(transform@corners.T).T[:,:2];all_points.extend(points)
            records.append({'link':link.get('name'),'geometry':kind,'maximum_xy_radius_bound_m':float(np.max(np.linalg.norm(points,axis=1)))})
    if not all_points:raise ValueError('No collision geometry')
    points=np.asarray(all_points);bound=float(np.max(np.linalg.norm(points,axis=1)))
    return {'reference_frame':'base_footprint','joint_positions':'zero / nominal level chassis','collision_geometries':len(records),
        'minimum_xy_m':np.min(points,axis=0).tolist(),'maximum_xy_m':np.max(points,axis=0).tolist(),
        'conservative_radius_bound_m':bound,'proposed_radius_m':radius,'radius_covers_nominal_collision_bounds':radius>=bound,
        'radial_margin_m':radius-bound,'collisions':records,
        'limitations':'Sphere/cylinder bounds are conservative. No swept-path, costmap, clearance, contact or articulated-joint-range acceptance is claimed.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--urdf',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    report=audit(args.urdf);args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='collisions'},indent=2))
