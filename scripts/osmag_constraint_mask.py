"""Conservative Phase 06C1 mask snapshots and consumption checks, without ROS.

0 permits occupancy; 100 forbids it. Physical-map cells are never cleared.
This is raw keepout geometry, not an inflated costmap or a motion controller.
"""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np
from shapely.geometry import LineString
from shapely.ops import unary_union
try:
    from shapely import contains_xy
except ImportError:
    from shapely.vectorized import contains as contains_xy
from gcu_plan_validation import validate_path, cell_at


class ConstraintError(ValueError):
    pass


def _lines(geometry):
    if geometry.is_empty:
        return
    if geometry.geom_type in ['LineString', 'LinearRing']:
        yield list(geometry.coords)
    elif hasattr(geometry, 'geoms'):
        for child in geometry.geoms:
            yield from _lines(child)
    else:
        raise ConstraintError('unsupported_barrier_geometry')


def _burn(mask, points, resolution, origin):
    """Block every cell potentially touching a line, using its half diagonal.

    Segment-local bounding boxes bound work; no per-edge whole-map scan.
    """
    margin = resolution/math.sqrt(2)+1e-7
    for a, b in zip(points, points[1:]):
        c0=max(0,math.floor((min(a[0],b[0])-margin-origin[0])/resolution))
        c1=min(mask.shape[1],math.floor((max(a[0],b[0])+margin-origin[0])/resolution)+1)
        r0=max(0,math.floor((min(a[1],b[1])-margin-origin[1])/resolution))
        r1=min(mask.shape[0],math.floor((max(a[1],b[1])+margin-origin[1])/resolution)+1)
        if c0>=c1 or r0>=r1:continue
        rows,cols=np.mgrid[r0:r1,c0:c1]
        x=origin[0]+(cols+.5)*resolution;y=origin[1]+(rows+.5)*resolution
        dx=b[0]-a[0];dy=b[1]-a[1];squared=dx*dx+dy*dy
        t=np.clip(((x-a[0])*dx+(y-a[1])*dy)/squared,0.,1.) if squared else 0.
        distance=np.hypot(x-(a[0]+t*dx),y-(a[1]+t*dy))
        view=mask[r0:r1,c0:c1];view[distance<=margin]=100


@dataclass(frozen=True)
class ConstraintMask:
    mask: np.ndarray
    combined: np.ndarray
    resolution: float
    origin: tuple
    mission_id: str
    route_revision: int
    closure_revision: int
    mask_revision: int
    allowed_areas: tuple
    open_passages: tuple
    disabled_passages: tuple
    physical_sha256: str
    mask_sha256: str
    radius: float
    padding: float

    def report(self):
        return {name:getattr(self,name) for name in self.__dataclass_fields__ if name not in ['mask','combined']} | {
            'shape':list(self.mask.shape),'frame':'map','blocked_cells':int(np.sum(self.mask==100)),
            'scope':'Raw conservative keepout; actual costmap consumption and motion require separate checks.'}


def build_mask(grid,resolution,origin,areas,passage_segments,passage_areas,*,allowed_areas,
               open_passages,disabled_passages,robot_pose,mission_id,route_revision=1,
               closure_revision=0,mask_revision=1,radius=.18,padding=.02):
    grid=np.asarray(grid);origin=tuple(origin)
    if grid.ndim!=2 or not grid.size or not np.isin(grid,[-1,0,100]).all():
        raise ConstraintError('invalid_physical_grid')
    if len(origin)!=2 or not all(math.isfinite(v) for v in (*origin,resolution,radius,padding)) or resolution<=0 or radius<=0 or padding<0:
        raise ConstraintError('invalid_map_geometry')
    if not isinstance(mission_id,str) or not mission_id or any(type(v) is not int for v in [route_revision,closure_revision,mask_revision]) or min(route_revision,mask_revision)<1 or closure_revision<0:
        raise ConstraintError('invalid_mask_identity')
    allowed=tuple(sorted(set(allowed_areas)));opened=tuple(sorted(set(open_passages)));disabled=tuple(sorted(set(disabled_passages)))
    if not allowed or not set(allowed)<=set(areas):raise ConstraintError('unknown_allowed_area')
    if not (set(opened)|set(disabled))<=set(passage_segments):raise ConstraintError('unknown_passage')
    if set(opened)&set(disabled):raise ConstraintError('opened_disabled_passage')
    for name in opened:
        if name not in passage_areas or len(set(passage_areas[name]))!=2 or not set(passage_areas[name])<=set(allowed):
            raise ConstraintError('open_passage_incident_area_mismatch')
    if any(not areas[name].is_valid for name in allowed):raise ConstraintError('invalid_area_geometry')
    # Numerical seam tolerance only; original physical occupancy always wins.
    permitted=unary_union([areas[name].buffer(1e-7) for name in allowed])
    inset=permitted.buffer(-resolution/math.sqrt(2))
    rows,cols=np.mgrid[:grid.shape[0],:grid.shape[1]]
    inside=contains_xy(inset,origin[0]+(cols+.5)*resolution,origin[1]+(rows+.5)*resolution)
    mask=np.where(inside,0,100).astype(np.int8)
    # Union alone is insufficient: adjacent route areas may have an unselected
    # shared boundary. Close EVERY area boundary except named open interfaces.
    boundaries=unary_union([areas[name].boundary for name in allowed])
    if opened:
        openings=unary_union([LineString(passage_segments[name]) for name in opened]).buffer(1e-7)
        boundaries=boundaries.difference(openings)
    for points in _lines(boundaries):_burn(mask,points,resolution,origin)
    # Explicit closures/unselected interfaces override even overlapping openings.
    for name,points in passage_segments.items():
        incident=passage_areas.get(name,())
        if name in disabled or (name not in opened and set(incident)&set(allowed)):
            _burn(mask,points,resolution,origin)
    physical=np.ascontiguousarray(grid,dtype=np.int16)
    combined=np.where((physical==0)&(mask==0),0,np.where(physical<0,-1,100)).astype(np.int16)
    effective=radius+padding
    if not validate_path(physical,resolution,origin,[robot_pose],effective)['valid']:
        raise ConstraintError('robot_footprint_intersects_physical_map')
    if not validate_path(combined,resolution,origin,[robot_pose],effective)['valid']:
        raise ConstraintError('robot_footprint_intersects_constraint')
    metadata=dict(resolution=resolution,origin=origin,mission_id=mission_id,route_revision=route_revision,
                  closure_revision=closure_revision,mask_revision=mask_revision,allowed=allowed,open=opened,disabled=disabled,
                  radius=radius,padding=padding)
    physical_hash=hashlib.sha256(physical.tobytes()).hexdigest()
    mask_hash=hashlib.sha256(mask.tobytes()+json.dumps(metadata,sort_keys=True).encode()+physical_hash.encode()).hexdigest()
    mask.flags.writeable=False;combined.flags.writeable=False
    return ConstraintMask(mask,combined,resolution,origin,mission_id,route_revision,closure_revision,mask_revision,
                          allowed,opened,disabled,physical_hash,mask_hash,radius,padding)


class ConsumptionGate:
    """Check actual map-frame costmap values; publishing a mask is not an ACK.

    This pure checker cannot transform odom grids. A future ROS adapter must
    supply the checked map-frame geometry/TF and a common ROS time domain.
    Native costmaps do not echo mask hashes: readiness proves observed cell
    constraints in each grid's overlap, not native revision metadata ownership.
    """
    def __init__(self,snapshot,published_at,free_probes):
        if not math.isfinite(published_at):raise ConstraintError('invalid_publish_stamp')
        self.snapshot=snapshot;self.published_at=published_at;self.probes=tuple(tuple(p) for p in free_probes)
        self.observations={}
        self.required_cells={}
        for point in self.probes:
            if cell_at(snapshot.combined,snapshot.resolution,snapshot.origin,*point)!=0:
                raise ConstraintError('free_probe_not_permitted')

    def observe(self,source,grid,resolution,origin,stamp,now,frame='map',mask_sha256=None):
        self.observations.pop(source,None)
        if source not in ['global','local']:raise ConstraintError('unknown_costmap_source')
        if mask_sha256 is not None and mask_sha256!=self.snapshot.mask_sha256:raise ConstraintError('obsolete_mask_revision')
        if frame!='map':raise ConstraintError('costmap_transform_required')
        if not all(math.isfinite(v) for v in [stamp,now,resolution,*origin]) or not self.published_at<=stamp<=now or now-stamp>.5:
            raise ConstraintError('costmap_not_fresh_after_mask')
        data=np.asarray(grid)
        if data.ndim!=2 or not data.size or resolution<=0:raise ConstraintError('invalid_costmap')
        if abs(resolution-self.snapshot.resolution)>1e-9:raise ConstraintError('unvalidated_costmap_resolution')
        if source=='global':
            snap=self.snapshot
            if any(a>b+1e-7 for a,b in zip(origin,snap.origin)) or any(
                    a+size*resolution<b+total*snap.resolution-1e-7
                    for a,size,b,total in zip(origin,data.shape[::-1],snap.origin,snap.mask.shape[::-1])):
                raise ConstraintError('global_costmap_does_not_cover_mask')
        snap=self.snapshot
        geometry = (data.shape, resolution, tuple(origin))
        cached_geometry, required, blocked_count = self.required_cells.get(source, (None, None, 0))
        if geometry != cached_geometry:
            # Map cell centers independently along each axis. Cache the
            # projection of this immutable mask, then check every new grid.
            y = origin[1] + (np.arange(data.shape[0]) + .5) * resolution
            x = origin[0] + (np.arange(data.shape[1]) + .5) * resolution
            rows = np.floor((y - snap.origin[1]) / snap.resolution).astype(int)
            cols = np.floor((x - snap.origin[0]) / snap.resolution).astype(int)
            rows_inside = (rows >= 0) & (rows < snap.mask.shape[0])
            cols_inside = (cols >= 0) & (cols < snap.mask.shape[1])
            if not rows_inside.any() or not cols_inside.any():
                raise ConstraintError('costmap_does_not_overlap_mask')
            required = np.zeros(data.shape, dtype=bool)
            required[np.ix_(rows_inside, cols_inside)] = (
                snap.mask[np.ix_(rows[rows_inside], cols[cols_inside])] == 100)
            blocked_count = int(required.sum())
            self.required_cells[source] = (geometry, required, blocked_count)
        if np.any(required&(data>=0)&(data<100)):raise ConstraintError('mask_constraints_not_consumed')
        checked=0
        for point in self.probes:
            value=cell_at(data,resolution,origin,*point)
            if value is not None:
                checked+=1
                if value<0 or value>=99:raise ConstraintError('free_probe_blocked')
        if not checked:raise ConstraintError('no_free_probe_in_costmap')
        self.observations[source]={'stamp':stamp,'observed_at':now,'mask_sha256':snap.mask_sha256,
                                   'checked_blocked_cells':blocked_count,'free_probes':checked}

    def ready(self,now):
        return set(self.observations)=={'global','local'} and all(
            0<=now-o['stamp']<=.5 for o in self.observations.values())
