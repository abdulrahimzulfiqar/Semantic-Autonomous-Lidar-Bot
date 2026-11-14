"""Connect a localized pose to the remaining reference prefix, without shortcuts."""
import math
from shapely.geometry import LineString,Point
from shapely.ops import substring


def prefix_to_target(path,robot,target):
    if len(path)<2:
        return [tuple(robot),tuple(target)] if math.dist(robot,target)>1e-8 else [tuple(robot)]
    line=LineString(path);end=line.project(Point(target));start=line.project(Point(robot))
    if line.distance(Point(target))>1e-5:raise ValueError('segment_target_off_reference')
    piece=substring(line,start,end)
    points=list(piece.coords)
    result=[tuple(robot)]
    for p in points+[tuple(target)]:
        if math.dist(result[-1],p)>1e-8:result.append(tuple(p))
    return result
