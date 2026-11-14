#!/usr/bin/env python3
"""Motion-free GCU window fidelity audit; never admits a mismatching raster."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
import yaml

from osmag_rolling_raster import RollingRaster, load_osm_geometry
from gcu_plan_validation import validate_path


def audit(assets, output):
    output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load((assets/'gcu_ground_floor.yaml').read_text())
    resolution, anchor = config['resolution'], tuple(config['origin'][:2])
    if abs(config['origin'][2]) > 1e-10:
        raise ValueError('rotated_reference_unsupported')
    begin = time.perf_counter()
    areas, passages, incident, source = load_osm_geometry(assets/'gcu_ground_floor.osm')
    raster = RollingRaster(areas, passages, resolution=resolution, anchor=anchor,
                           shape=(128,128), source_sha256=source)
    preprocess = time.perf_counter()-begin
    # This full raster is evaluation-only. The renderer never reads it.
    image = np.flipud(np.asarray(Image.open(assets/config['image'])))
    reference = np.where(image==254,0,np.where(image==0,100,-1))
    windows = []
    for revision, center in enumerate([(-.5,-12.960313240266368),(3.,-12.),(6.,-10.)],1):
        begin = time.perf_counter()
        s = raster.render(center,revision=revision,stamp=float(revision))
        elapsed = time.perf_counter()-begin
        rr,cc = np.mgrid[:s.grid.shape[0],:s.grid.shape[1]]
        cols = cc+round((s.origin[0]-anchor[0])/resolution)
        rows = rr+round((s.origin[1]-anchor[1])/resolution)
        valid = (cols>=0)&(cols<reference.shape[1])&(rows>=0)&(rows<reference.shape[0])
        expected = np.full(s.grid.shape,-1,dtype=np.int16)
        expected[valid] = reference[rows[valid],cols[valid]]
        unsafe = (s.grid==0)&(expected!=0)
        conservative = (s.grid!=0)&(expected==0)
        record = s.report() | dict(center=center,render_s=elapsed,
            false_free_vs_reference_cells=int(unsafe.sum()),
            conservative_nonfree_vs_reference_cells=int(conservative.sum()),
            physical_fidelity_passed=not unsafe.any())
        windows.append(record)
        np.savez_compressed(output/f'window_{revision:02d}.npz',candidate=s.grid,
                            reference=expected,false_free=unsafe,origin=s.origin,
                            resolution=s.resolution)
    report = dict(scope='Offline OSM-only candidate; no ROS, native costmaps or motion',
        source_sha256=source,reference_sha256=hashlib.sha256((assets/config['image']).read_bytes()).hexdigest(),
        preprocess_s=preprocess,leaf_count=len(areas),passage_count=len(incident),
        fixed_shape=[128,128],windows=windows,
        fidelity_gate_passed=all(w['physical_fidelity_passed'] for w in windows),
        native_motion_admitted=False,
        remaining=['Physical fidelity gaps, if present, must be resolved before native planning',
                   'Native raster/constraint revision and proxy-goal integration',
                   'Bounded cross-window motion, resource comparison and user review'],
        retained_full_map_users=['Existing graph preprocessing and AMCL; evaluation reference in this offline audit'])
    # Cover every reference cell with an interior sample (exclude the deliberate
    # occupied window border when measuring coverage). This is still evaluation
    # work, not a full-grid dependency in the renderer.
    coverage = np.zeros(reference.shape,dtype=bool)
    unsafe_global = np.zeros(reference.shape,dtype=bool)
    durations = []
    failing_tiles = []
    tile_count = 0
    for row in range(0,reference.shape[0],126):
        for col in range(0,reference.shape[1],126):
            tile_count += 1
            center = (anchor[0]+(col+63.5)*resolution,
                      anchor[1]+(row+63.5)*resolution)
            begin = time.perf_counter()
            s = raster.render(center,revision=tile_count,stamp=float(tile_count))
            durations.append(time.perf_counter()-begin)
            rr,cc = np.mgrid[:128,:128]
            rows = rr+round((s.origin[1]-anchor[1])/resolution)
            cols = cc+round((s.origin[0]-anchor[0])/resolution)
            valid = (rows>=0)&(rows<reference.shape[0])&(cols>=0)&(cols<reference.shape[1])
            valid &= (rr>0)&(rr<127)&(cc>0)&(cc<127)
            coverage[rows[valid],cols[valid]] = True
            bad = (s.grid[valid]==0)&(reference[rows[valid],cols[valid]]!=0)
            unsafe_global[rows[valid][bad],cols[valid][bad]] = True
            if bad.any():failing_tiles.append(dict(center=center,false_free_cells=int(bad.sum())))
    report['full_floor_evaluation'] = dict(tile_count=tile_count,
        covered_reference_cells=int(coverage.sum()),reference_cells=reference.size,
        all_reference_cells_covered=bool(coverage.all()),
        unique_false_free_cells=int(unsafe_global.sum()),failing_tiles=failing_tiles,
        render_p50_s=float(np.median(durations)),render_p95_s=float(np.percentile(durations,95)),
        timing_scope='Local offline Python run, not a VM/Pi or online 10 Hz benchmark')
    report['fidelity_gate_passed'] &= bool(coverage.all() and not unsafe_global.any())
    route_checks = []
    for name in ['adjacent_sector_masked','corridor_to_room_masked','room_to_room_masked','doorway']:
        fixture = json.loads((assets/'phase6_fixtures'/f'{name}.json').read_text())
        path = (fixture['route_contract'] if 'route_contract' in fixture else fixture['route'])['path']
        failures = []
        checked = 0
        for a,b in zip(path,path[1:]):
            count = max(1,int(np.ceil(np.linalg.norm(np.subtract(b,a))/.25)))
            points = np.linspace(a,b,count+1)
            for first,second in zip(points,points[1:]):
                checked += 1
                center = (first+second)/2
                s = raster.render(center,revision=checked,stamp=float(checked))
                if not validate_path(s.grid,s.resolution,s.origin,[first,second],.20)['valid']:
                    failures.append(dict(first=first.tolist(),second=second.tolist()))
        route_checks.append(dict(fixture=name,padded_radius_m=.20,
            checked_short_segments=checked,passed=not failures,failures=failures))
    report['route_clearance_checks'] = route_checks
    report['offline_geometry_gate_passed'] = report['fidelity_gate_passed'] and all(c['passed'] for c in route_checks)
    report['source_files_sha256'] = {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__),Path(__file__).with_name('osmag_rolling_raster.py'),
                  Path(__file__).with_name('osmag_constraint_mask.py')]}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,default=Path('simulation/gcu_ground_floor'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result=audit(a.assets,a.output)
    print(json.dumps({k:result[k] for k in ['fidelity_gate_passed','native_motion_admitted','windows']},indent=2))
