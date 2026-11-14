"""Offline bounded raster candidate from osmAG polygons, without a source PGM.

No ROS publishers or motion. Raster fidelity against physical CAD occupancy is
a separate admission gate; a topological polygon is not a guaranteed wall model.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
from pyproj import CRS, Transformer
try:
    from shapely import contains_xy
except ImportError:
    from shapely.vectorized import contains as contains_xy
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from osmag_constraint_mask import _burn, _lines


@dataclass(frozen=True)
class RasterWindow:
    grid: np.ndarray
    origin: tuple
    resolution: float
    revision: int
    stamp: float
    source_sha256: str
    sha256: str

    def report(self):
        return dict(frame='map', origin=self.origin, resolution=self.resolution,
                    shape=list(self.grid.shape), revision=self.revision,
                    stamp=self.stamp, source_sha256=self.source_sha256,
                    sha256=self.sha256, grid_bytes=self.grid.nbytes,
                    free_cells=int(np.sum(self.grid == 0)),
                    unknown_cells=int(np.sum(self.grid == -1)),
                    occupied_cells=int(np.sum(self.grid == 100)),
                    edge_policy='One occupied border cell; no outside-window traversal',
                    scope='OSM-only candidate; physical fidelity and native admission separate')


class RollingRaster:
    def __init__(self, areas, passages, *, resolution, anchor=(0., 0.),
                 shape=(128, 128), source_sha256=None):
        if (not math.isfinite(resolution) or resolution <= 0 or len(anchor) != 2
                or not all(math.isfinite(v) for v in anchor)):
            raise ValueError('invalid_grid_geometry')
        if len(shape) != 2 or any(type(v) is not int or v < 3 for v in shape):
            raise ValueError('invalid_window_shape')
        if not areas or any(p.is_empty or not p.is_valid or p.geom_type != 'Polygon'
                            for p in areas.values()):
            raise ValueError('invalid_leaf_geometry')
        if any(len(p) != 2 or not all(math.isfinite(x) for point in p for x in point)
               or math.dist(*p) <= 0 for p in passages.values()):
            raise ValueError('invalid_passage_geometry')
        self.resolution = resolution
        self.anchor = tuple(anchor)
        self.shape = tuple(shape)
        # Full vector geometry remains resident; no full-building grid is made.
        self.free = unary_union(list(areas.values())).buffer(-resolution/math.sqrt(2))
        boundaries = unary_union([p.boundary for p in areas.values()])
        if passages:
            boundaries = boundaries.difference(unary_union(
                [LineString(p) for p in passages.values()]).buffer(1e-7))
        self.barriers = [(points, LineString(points).bounds)
                         for points in _lines(boundaries)]
        geometry = b''.join(n.encode()+p.wkb for n, p in sorted(areas.items()))
        geometry += json.dumps(passages, sort_keys=True).encode()
        self.source_sha256 = source_sha256 or hashlib.sha256(geometry).hexdigest()

    def render(self, center, *, revision, stamp):
        if (len(center) != 2 or not all(math.isfinite(v) for v in center)
                or type(revision) is not int or revision < 1
                or not math.isfinite(stamp) or stamp < 0):
            raise ValueError('invalid_window_identity_or_pose')
        rows, cols = self.shape
        r = self.resolution
        offset = tuple(math.floor((v-a)/r)-size//2
                       for v, a, size in zip(center, self.anchor, (cols, rows)))
        origin = tuple(a+i*r for a, i in zip(self.anchor, offset))
        rr, cc = np.mgrid[:rows, :cols]
        # Use integer global indices so shifting windows samples identical cells.
        xx = self.anchor[0]+(cc+offset[0]+.5)*r
        yy = self.anchor[1]+(rr+offset[1]+.5)*r
        grid = np.where(contains_xy(self.free, xx, yy), 0, -1).astype(np.int8)
        margin = r/math.sqrt(2)+1e-7
        for points, (x0, y0, x1, y1) in self.barriers:
            if (x1+margin >= origin[0] and y1+margin >= origin[1]
                    and x0-margin <= origin[0]+cols*r
                    and y0-margin <= origin[1]+rows*r):
                _burn(grid, points, r, origin)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        meta = dict(origin=origin, resolution=r, shape=self.shape,
                    revision=revision, stamp=stamp, source=self.source_sha256)
        digest = hashlib.sha256(grid.tobytes()+json.dumps(meta, sort_keys=True).encode()).hexdigest()
        grid.flags.writeable = False
        return RasterWindow(grid, origin, r, revision, stamp, self.source_sha256, digest)


def load_osm_geometry(path):
    """Match the accepted local polyconic frame; exclude parent containers."""
    data = path.read_bytes()
    osm = ET.fromstring(data)
    root = osm.find('node')
    crs = CRS.from_proj4(f"+proj=poly +lat_0={root.get('lat')} +lon_0={root.get('lon')} +ellps=WGS84 +units=m +no_defs")
    project = Transformer.from_crs('EPSG:4326', crs, always_xy=True)
    nodes = {n.get('id'): project.transform(float(n.get('lon')), float(n.get('lat')))
             for n in osm.findall('node')}
    areas, parents, passages, incident = {}, {}, {}, {}
    for way in osm.findall('way'):
        tags = {t.get('k'): t.get('v') for t in way.findall('tag')}
        points = [nodes[n.get('ref')] for n in way.findall('nd')]
        if tags.get('osmAG:type') == 'area':
            name = tags['name']
            if name in areas:
                raise ValueError('duplicate_area_name')
            areas[name] = Polygon(points)
            parents[name] = tags.get('osmAG:parent')
        elif tags.get('osmAG:type') == 'passage':
            passages[way.get('id')] = points
            incident[way.get('id')] = (tags['osmAG:from'], tags['osmAG:to'])
    containers = {p for p in parents.values() if p is not None}
    if not containers <= areas.keys():
        raise ValueError('unknown_area_parent')
    leaves = {n: p for n, p in areas.items() if n not in containers}
    if any(not set(pair) <= leaves.keys() for pair in incident.values()):
        raise ValueError('passage_not_between_leaves')
    return leaves, passages, incident, hashlib.sha256(data).hexdigest()
