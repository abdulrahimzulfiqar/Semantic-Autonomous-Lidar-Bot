"""Pure conservative swept-circle checks against original occupancy cells.

The map is the permitted traversal domain: unknown and nonfree cells block paths.
Continuous segments are sampled <= resolution/4 apart and the check adds half
that spacing to radius, conservatively bounding the gaps between samples.
"""
import math
import numpy as np


def cell_at(grid,resolution,origin,x,y):
    col=math.floor((x-origin[0])/resolution);row=math.floor((y-origin[1])/resolution)
    if not (0<=row<grid.shape[0] and 0<=col<grid.shape[1]):return None
    return int(grid[row,col])


def clearance(grid,resolution,origin,x,y,search):
    h,w=grid.shape
    boundary=min(x-origin[0],y-origin[1],origin[0]+w*resolution-x,origin[1]+h*resolution-y)
    if boundary<=0:return 0.
    c0=max(0,math.floor((x-search-origin[0])/resolution));c1=min(w,math.floor((x+search-origin[0])/resolution)+1)
    r0=max(0,math.floor((y-search-origin[1])/resolution));r1=min(h,math.floor((y+search-origin[1])/resolution)+1)
    rows,cols=np.nonzero(grid[r0:r1,c0:c1]!=0)
    lower=min(search,boundary)
    if len(rows):
        xmin=origin[0]+(cols+c0)*resolution;ymin=origin[1]+(rows+r0)*resolution
        dx=np.maximum(np.maximum(xmin-x,x-(xmin+resolution)),0)
        dy=np.maximum(np.maximum(ymin-y,y-(ymin+resolution)),0)
        lower=min(lower,float(np.min(np.hypot(dx,dy))))
    return lower


def validate_path(grid,resolution,origin,points,radius=.18):
    if resolution<=0 or radius<=0:raise ValueError('Positive resolution and radius required')
    if not points:return {'valid':False,'reason':'empty_path'}
    if not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in points):return {'valid':False,'reason':'nonfinite_path'}
    spacing=resolution/4;required=radius+spacing/2;minimum=math.inf;count=0
    dense=[points[0]]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/spacing))
        dense.extend((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n) for i in range(1,n+1))
    for x,y in dense:
        c=clearance(grid,resolution,origin,x,y,radius+2*resolution)
        minimum=min(minimum,c);count+=1
        if c<required:return {'valid':False,'reason':'swept_footprint_blocked','at':[x,y],'clearance_m':c,'required_m':required}
    return {'valid':True,'samples':count,'radius_m':radius,'segment_sampling_m':spacing,
        'checked_clearance_lower_bound_m':minimum,'clearance_search_cap_m':radius+2*resolution}
