"""Validate 2D user goals and require a confirmed stop before recovery."""
import math
from osmag_mission_contract import MissionError


def parse_goal(frame,position,quaternion):
    if frame.lstrip('/')!='map':raise MissionError('goal_frame_must_be_map')
    if not all(math.isfinite(v) for v in (*position,*quaternion)):raise MissionError('nonfinite_goal')
    x,y,z,w=quaternion
    if abs(x)+abs(y)>1e-6 or abs(sum(v*v for v in quaternion)-1)>1e-3:raise MissionError('invalid_planar_goal_orientation')
    return [*position,math.atan2(2*w*z,1-2*z*z)]


def require_recoverable(model,handles):
    if model.state not in ['cancelled','failed'] or not model.stop_confirmed or model._unresolved() or handles:raise MissionError('recovery_requires_confirmed_stop')
