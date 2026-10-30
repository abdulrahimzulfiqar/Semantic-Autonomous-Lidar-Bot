"""Reconstruct labelled graph regions inside occupancy walls, preserving holes.

Optional alternative to the legacy Voronoi-face polygon merger. Coordinates in
GeoJSON are raster pixels, not geographic coordinates. OSM coordinates use the
same local WGS84 polyconic projection as the C++ exporter.
"""
from __future__ import annotations

from collections import deque
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
from pyproj import CRS, Transformer
from shapely import union_all
from shapely.geometry import LineString, MultiPolygon, mapping, box
from shapely.ops import linemerge


def grow_labels(free: np.ndarray, seeds: np.ndarray) -> np.ndarray:
    """Deterministic four-neighbour geodesic flood; unseeded components stay 0.

    No diagonal corner cutting, filling enclosed white islands, or wall crossing.
    Label 0 means unassigned; positive integers are labels, not occupancy values.
    """
    if free.shape != seeds.shape:
        raise ValueError('Seed and occupancy dimensions differ')
    result = np.where(free, seeds, 0).astype(np.int32)
    height, width = result.shape
    queue = deque(map(tuple, np.argwhere(result > 0)))
    while queue:
        y, x = queue.popleft()
        label = result[y, x]
        for ny, nx in ((y - 1, x), (y, x - 1), (y, x + 1), (y + 1, x)):
            if 0 <= ny < height and 0 <= nx < width and free[ny, nx] and result[ny, nx] == 0:
                result[ny, nx] = label
                queue.append((ny, nx))
    return result


def regrow_without_small_basins(free, seeds, labels, minimum_pixels):
    """Drop tiny seeds, not walkable cells: neighbours reclaim their basins.

    Otherwise filtering a small doorway fragment creates an artificial barrier
    between a room and its corridor. Unseeded isolated components remain excluded.
    """
    if minimum_pixels <= 0:
        return labels
    accepted = np.zeros(labels.shape, bool)
    removed = False
    for label in np.unique(labels):
        if label == 0:
            continue
        count, cc, stats, _ = cv2.connectedComponentsWithStats((labels == label).astype(np.uint8), connectivity=4)
        for component in range(1, count):
            if stats[component, cv2.CC_STAT_AREA] >= minimum_pixels:
                accepted |= cc == component
            else:
                removed = True
    if not removed:
        return labels
    return grow_labels(free, np.where(accepted, seeds, 0))


def polygonal(geometry):
    if geometry.geom_type == 'Polygon':
        return [geometry] if geometry.area > 0 else []
    if hasattr(geometry, 'geoms'):
        return [p for g in geometry.geoms for p in polygonal(g)]
    return []


def mask_geometry(mask):
    """Exact union of raster cells, coalesced into runs before polygon union.

    Pixel (x,y) occupies [x-.5,x+.5] × [y-.5,y+.5]. Using contour pixel
    centres instead would shrink regions and leave passages outside both rooms.
    """
    active, rectangles = {}, []
    for y in range(mask.shape[0] + 1):
        if y < mask.shape[0]:
            changes = np.flatnonzero(np.diff(np.pad(mask[y].astype(np.int8), (1, 1))))
            runs = set(zip(map(int, changes[::2]), map(int, changes[1::2])))
        else:
            runs = set()
        for run in sorted(set(active) - runs):
            first_y = active.pop(run)
            rectangles.append(box(run[0] - .5, first_y - .5, run[1] - .5, y - .5))
        for run in sorted(runs - set(active)):
            active[run] = y
    return union_all(rectangles) if rectangles else MultiPolygon([])


def interfaces(labels):
    """Shared free-cell borders; retain actual polylines, never connect centroids."""
    edges = {}
    for vertical in (True, False):
        a, b = (labels[:, :-1], labels[:, 1:]) if vertical else (labels[:-1, :], labels[1:, :])
        for y, x in np.argwhere((a > 0) & (b > 0) & (a != b)):
            key = tuple(sorted((int(a[y, x]), int(b[y, x]))))
            if vertical:
                line = [(x + .5, y - .5), (x + .5, y + .5)]
            else:
                line = [(x - .5, y + .5), (x + .5, y + .5)]
            edges.setdefault(key, []).append(LineString(line))
    for pair, segments in sorted(edges.items()):
        merged = union_all(segments)
        if merged.geom_type != 'LineString':
            merged = linemerge(merged)
        for line in ([merged] if merged.geom_type == 'LineString' else merged.geoms):
            if line.length > 0:
                yield pair, line


def export_osm(path, regions, passages, config):
    root_cfg = config['root_node']
    resolution = config['map_preprocessing']['resolution']
    local = CRS.from_proj4(f"+proj=poly +lat_0={root_cfg['latitude']} +lon_0={root_cfg['longitude']} +ellps=WGS84 +units=m +no_defs")
    transform = Transformer.from_crs(local, 'EPSG:4326', always_xy=True)
    osm = ET.Element('osm', version='0.6', generator='CAD occupancy-seeded reconstruction')
    reference = ET.SubElement(osm, 'node', id='-1', visible='true',
                              lat=str(root_cfg['latitude']), lon=str(root_cfg['longitude']))
    ET.SubElement(reference, 'tag', k='name', v='root')
    nodes = {(float(root_cfg['pixel_x']), float(root_cfg['pixel_y'])): '-1'}
    next_node, next_way, next_relation = -2, -1, -1

    def tag(element, key, value):
        ET.SubElement(element, 'tag', k=key, v=str(value))

    def node(point):
        nonlocal next_node
        key = tuple(map(float, point))
        if key not in nodes:
            longitude, latitude = transform.transform((key[0] - root_cfg['pixel_x']) * resolution,
                                                      (root_cfg['pixel_y'] - key[1]) * resolution)
            nodes[key] = str(next_node)
            ET.SubElement(osm, 'node', id=str(next_node), visible='true', lat=f'{latitude:.11f}', lon=f'{longitude:.11f}')
            next_node -= 1
        return nodes[key]

    def way(coords):
        nonlocal next_way
        result = ET.SubElement(osm, 'way', id=str(next_way), visible='true')
        next_way -= 1
        for point in coords:
            ET.SubElement(result, 'nd', ref=node(point))
        return result

    level = config.get('level', '1')
    for index, region in sorted(regions.items()):
        parts = polygonal(region['geometry'])
        if len(parts) == 1 and not parts[0].interiors:
            area = way(parts[0].exterior.coords)
        else:
            area = ET.SubElement(osm, 'relation', id=str(next_relation), visible='true')
            next_relation -= 1
            tag(area, 'type', 'multipolygon')
            for part in parts:
                outer = way(part.exterior.coords)
                ET.SubElement(area, 'member', type='way', ref=outer.attrib['id'], role='outer')
                for hole in part.interiors:
                    inner = way(hole.coords)
                    ET.SubElement(area, 'member', type='way', ref=inner.attrib['id'], role='inner')
        tag(area, 'name', f'room_{index}')
        tag(area, 'osmAG:type', 'area')
        tag(area, 'osmAG:areaType', 'room')
        tag(area, 'level', level)
        tag(area, 'height', config.get('height_per_level', 3.2))
        tag(area, 'source', 'labelled_voronoi_occupancy_reconstruction')
    for index, (pair, line) in enumerate(passages, 1):
        passage = way(line.coords)
        for key, value in [('name', f'p_{index}'), ('osmAG:type', 'passage'), ('osmAG:from', f'room_{pair[0]}'),
                           ('osmAG:to', f'room_{pair[1]}'), ('level', level), ('height', config.get('height_per_level', 3.2)), ('source', 'shared_region_interface')]:
            tag(passage, key, value)
    osm[:] = sorted(osm, key=lambda e: {"node": 0, "way": 1, "relation": 2}[e.tag])
    ET.indent(osm)
    ET.ElementTree(osm).write(path, encoding='utf-8', xml_declaration=True)


def reconstruct(png_path, graph_dir, config, case):
    graph_dir = Path(graph_dir)
    occupancy = np.asarray(Image.open(png_path).convert('L'))
    envelope = np.asarray(Image.open(graph_dir / 'building_envelope.png').convert('L'))
    free = (occupancy > 127) & (envelope > 127)
    with (graph_dir / 'stage_01_room_labels.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    room_ids = sorted({int(row['room_id']) for row in rows if int(row['room_id']) >= 0})
    if not room_ids:
        raise ValueError('No labelled finite graph edges to reconstruct')
    seeds = np.zeros(free.shape, np.int32)
    seed_names = {room: index for index, room in enumerate(room_ids, 1)}
    # Canonical row order makes collisions and BFS tie breaking repeatable.
    for row in sorted(rows, key=lambda r: tuple(float(r[k]) for k in ('room_id', 'x1', 'y1', 'x2', 'y2'))):
        room = int(row['room_id'])
        if room < 0:
            continue
        start = tuple(int(round(float(row[k]))) for k in ('x1', 'y1'))
        end = tuple(int(round(float(row[k]))) for k in ('x2', 'y2'))
        cv2.line(seeds, start, end, seed_names[room], 1)
    labels = grow_labels(free, seeds)
    min_area = config.get('polygon_processing', {}).get('small_room_filter', {})
    minimum = float(min_area.get('min_area', 0)) if min_area.get('enabled', False) else 0
    resolution = float(config['map_preprocessing']['resolution'])
    labels = regrow_without_small_basins(free, seeds, labels, minimum / resolution ** 2)
    regions, canonical = {}, np.zeros_like(labels)
    # Separate disconnected components rather than bridge them with a polygon.
    components = []
    for seed in sorted(np.unique(labels)):
        if seed == 0:
            continue
        count, cc, stats, _ = cv2.connectedComponentsWithStats((labels == seed).astype(np.uint8), connectivity=4)
        for component in range(1, count):
            mask = cc == component
            pixels = int(stats[component, cv2.CC_STAT_AREA])
            if pixels * resolution ** 2 < minimum:
                continue
            geometry = mask_geometry(mask)
            if geometry.is_empty or not geometry.is_valid:
                raise ValueError('Invalid reconstructed region')
            components.append((int(stats[component, cv2.CC_STAT_TOP]), int(stats[component, cv2.CC_STAT_LEFT]),
                               seed, mask, geometry, pixels))
    for index, (_, _, seed, mask, geometry, pixels) in enumerate(sorted(components, key=lambda x: x[:3]), 1):
        canonical[mask] = index
        regions[index] = {'geometry': geometry, 'seed_label': int(seed), 'pixel_area_m2': pixels * resolution ** 2}
    passages = list(interfaces(canonical))
    prefix = graph_dir / f'{case}_reconstructed'
    features = [{'type': 'Feature', 'properties': {'name': f'room_{index}', 'source_room_id': room_ids[region['seed_label'] - 1],
                 'area_m2': region['geometry'].area * resolution ** 2}, 'geometry': mapping(region['geometry'])}
                for index, region in sorted(regions.items())]
    geojson = Path(str(prefix) + '_regions.geojson')
    geojson.write_text(json.dumps({'type': 'FeatureCollection', 'coordinate_system': 'image_pixels_y_down', 'features': features}) + '\n')
    osm_path = Path(str(prefix) + '_osmAG.osm')
    export_osm(osm_path, regions, passages, config)
    preview = np.repeat(occupancy[:, :, None], 3, axis=2)
    for index in regions:
        preview[canonical == index] = ((index * 67) % 190 + 35, (index * 103) % 190 + 35, (index * 137) % 190 + 35)
    preview_path = Path(str(prefix) + '_roomGraph.png')
    Image.fromarray(preview).save(preview_path)
    np.save(str(prefix) + '_labels.npy', canonical)
    areas_path = Path(str(prefix) + '_room_areas.csv')
    with areas_path.open('w') as stream:
        csv.writer(stream).writerows((f'room_{i}', r['geometry'].area * resolution ** 2) for i, r in sorted(regions.items()))
    report = {'regions': len(regions), 'passages': len(passages), 'free_pixels': int(free.sum()),
              'unseeded_or_filtered_free_pixels': int((free & (canonical == 0)).sum()),
              'multipolygon_areas': sum(len(polygonal(r['geometry'])) != 1 or any(p.interiors for p in polygonal(r['geometry'])) for r in regions.values()),
              'notes': ['Polygons are exact unions of raster cells; shared interfaces lie on both area boundaries.',
                        'Unseeded components are excluded, not semantically classified as forbidden.',
                        'Passages are inferred shared label interfaces, not certified physical doorway widths.',
                        'Consumers must support OSM multipolygon area relations to preserve holes.']}
    Path(str(prefix) + '_report.json').write_text(json.dumps(report, indent=2) + '\n')
    return {'osm': osm_path, 'room_graph': preview_path, 'reconstructed_regions': geojson,
            'reconstructed_areas': areas_path, 'reconstruction_labels': Path(str(prefix) + '_labels.npy'), 'reconstruction_report': Path(str(prefix) + '_report.json')}
