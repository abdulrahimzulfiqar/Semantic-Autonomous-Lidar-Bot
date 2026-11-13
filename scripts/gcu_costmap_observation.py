"""Inspect native occupancy grids using the supplied planar transform, without ROS.

These are observed-cell checks, not a certificate for a swept robot footprint.
The caller must match the producing filter's TF policy: Humble KeepoutFilter
uses latest TF, independently of the costmap publication stamp. Keep native
grids and transforms for independent replay.
"""
import math
import numpy as np
from osmag_constraint_mask import ConstraintError


def lattice_phase(resolution,origin,map_resolution,map_origin):
    expected=float(np.float32(map_resolution))
    phase=[(a-b)/expected for a,b in zip(origin,map_origin)]
    error=max(abs(v-round(v)) for v in phase)
    return {'aligned':resolution==expected and error<=1e-6,'maximum_phase_error_cells':error,'resolution':resolution}


def inspect(snapshot, data, resolution, origin, map_from_grid, probes, *, full=False):
    data=np.asarray(data)
    if data.ndim!=2 or not data.size or not np.isin(data,range(-1,101)).all():
        raise ConstraintError('invalid_native_occupancy_grid')
    if not all(math.isfinite(v) for v in [resolution,*origin,*map_from_grid]) or resolution<=0:
        raise ConstraintError('invalid_native_geometry')
    # OccupancyGrid resolution is float32; accept exactly that serialization,
    # rather than widening the geometric tolerance arbitrarily.
    if resolution not in [snapshot.resolution,float(np.float32(snapshot.resolution))]:
        raise ConstraintError('unvalidated_native_resolution')
    tx,ty,yaw=map_from_grid;c=math.cos(yaw);s=math.sin(yaw)
    rows,cols=np.mgrid[:data.shape[0],:data.shape[1]]
    gx=origin[0]+(cols+.5)*resolution;gy=origin[1]+(rows+.5)*resolution
    x=tx+c*gx-s*gy;y=ty+s*gx+c*gy
    # Native KeepoutFilter constructs its mask Costmap2D from OccupancyGrid:
    # mask resolution is float32 on the wire, even when the source YAML was
    # double precision. Near cell edges, using YAML precision selects a
    # different cell. Match the actual mask representation without epsilon.
    mask_resolution=float(np.float32(snapshot.resolution))
    sr=np.floor((y-snapshot.origin[1])/mask_resolution).astype(int)
    sc=np.floor((x-snapshot.origin[0])/mask_resolution).astype(int)
    inside=(sr>=0)&(sc>=0)&(sr<snapshot.mask.shape[0])&(sc<snapshot.mask.shape[1])
    if not inside.any():raise ConstraintError('no_native_mask_overlap')
    if full:
        if abs(yaw)>1e-10 or math.dist([tx+origin[0],ty+origin[1]],snapshot.origin)>1e-6 or data.shape!=snapshot.mask.shape:
            raise ConstraintError('global_grid_geometry_changed')
    required=np.zeros_like(inside);required[inside]=snapshot.mask[sr[inside],sc[inside]]==100
    missed=required&(data>=0)&(data<100)
    checked=0
    for px,py in probes:
        dx=px-tx;dy=py-ty
        col=math.floor((c*dx+s*dy-origin[0])/resolution)
        row=math.floor((-s*dx+c*dy-origin[1])/resolution)
        if 0<=row<data.shape[0] and 0<=col<data.shape[1]:
            checked+=1
            if data[row,col]<0 or data[row,col]>=99:raise ConstraintError('native_free_probe_blocked')
    if not checked:raise ConstraintError('no_native_free_probe')
    report={'required_cells':int(required.sum()),'missing_cells':int(missed.sum()),'free_probes':checked,
            'map_from_grid':list(map_from_grid),'resolution':resolution,'mask_resolution':mask_resolution}
    if missed.any():raise ConstraintError('native_mask_cells_missing:'+str(report['missing_cells']))
    return report
