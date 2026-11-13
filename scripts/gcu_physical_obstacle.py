"""Evaluation-only conservative box raster; never supplied to navigation."""
import numpy as np


def overlay_box(grid,resolution,origin,box):
    x,y=box['center'];sx,sy,sz=box['size']
    if not all(np.isfinite(v) for v in [x,y,sx,sy,sz]) or min(sx,sy,sz)<=0:raise ValueError('invalid_obstacle_box')
    rows,cols=np.indices(grid.shape);ox,oy=origin
    hit=(ox+cols*resolution<=x+sx/2)&(ox+(cols+1)*resolution>=x-sx/2)&(oy+rows*resolution<=y+sy/2)&(oy+(rows+1)*resolution>=y-sy/2)
    result=grid.copy();result[hit]=100
    return result
