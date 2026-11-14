"""Select the first route exit from a padded window; never jump to re-entry.

This is geometric selection only. Native feasibility and consumed raster/mask
revisions must be verified separately before dispatching any navigation goal.
"""
from dataclasses import dataclass
import math

from gcu_plan_validation import validate_path


@dataclass(frozen=True)
class WindowGoal:
    point: tuple
    yaw: float
    proxy: bool
    prefix: tuple
    window_sha256: str
    window_revision: int


def select_window_goal(window, route, robot_pose, *, radius=.18, padding=.02,
                       edge_margin=.45, final_yaw=0.):
    if (not route or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in route)
            or len(robot_pose)!=2 or not all(math.isfinite(v) for v in robot_pose)
            or not all(math.isfinite(v) for v in [radius,padding,edge_margin,final_yaw])
            or radius<=0 or padding<0 or edge_margin<radius+padding+window.resolution):
        raise ValueError('invalid_window_goal_request')
    if math.dist(route[0],robot_pose)>1e-5:
        raise ValueError('route_must_start_at_current_robot_pose')
    h,w=window.grid.shape;r=window.resolution;x,y=window.origin
    low=(x+edge_margin,y+edge_margin);high=(x+w*r-edge_margin,y+h*r-edge_margin)
    def inside(p):return all(a<=v<=b for a,v,b in zip(low,p,high))
    if any(a>=b for a,b in zip(low,high)) or not inside(robot_pose):
        raise ValueError('robot_outside_usable_window')
    prefix=[tuple(route[0])];proxy=False
    for first,second in zip(route,route[1:]):
        if math.dist(first,second)<1e-10:continue
        if inside(second):prefix.append(tuple(second));continue
        fraction=1.
        for a,b,lo,hi in zip(first,second,low,high):
            delta=b-a
            if delta>0:fraction=min(fraction,(hi-a)/delta)
            elif delta<0:fraction=min(fraction,(lo-a)/delta)
        fraction=max(0.,min(1.,fraction))
        point=tuple(a+fraction*(b-a) for a,b in zip(first,second))
        if math.dist(prefix[-1],point)>1e-8:prefix.append(point)
        proxy=True
        break
    if proxy and (len(prefix)<2 or math.dist(prefix[0],prefix[-1])<.05):
        raise ValueError('no_useful_window_progress')
    validation=validate_path(window.grid,r,window.origin,prefix,radius+padding)
    if not validation['valid']:raise ValueError('route_prefix_not_footprint_feasible')
    yaw=final_yaw
    if proxy:
        a,b=prefix[-2:];yaw=math.atan2(b[1]-a[1],b[0]-a[0])
    return WindowGoal(prefix[-1],yaw,proxy,tuple(prefix),window.sha256,window.revision)


def route_suffix(route,distance):
    if not math.isfinite(distance) or distance<0:raise ValueError('invalid_route_distance')
    for index,(a,b) in enumerate(zip(route,route[1:])):
        length=math.dist(a,b)
        if length>0 and distance<length:
            point=[x+(y-x)*distance/length for x,y in zip(a,b)]
            return [point]+[list(p) for p in route[index+1:]]
        distance-=length
    return [list(route[-1])]
