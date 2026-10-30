#!/usr/bin/env python3
"""Local CAD regression: run with ./venv/bin/python area_graph_segment/test/test_gcu_pipeline.py.

The private DXF is not distributed. Regenerate gcu_final_clean.dxf using the
GCU recipe in ../commands_simplified before running this test.
"""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]


class GcuPipelineRegression(unittest.TestCase):
    @unittest.skipUnless((ROOT / 'gcu_final_clean.dxf').is_file(), 'Private GCU CAD input unavailable')
    def test_gcu_exports_valid_graph(self):
        (ROOT / 'runs').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='gcu-regression-', dir=ROOT / 'runs') as output:
            result = subprocess.run(
                [sys.executable, 'run_pipeline.py', '--config', 'config/repro_gcu.yaml',
                 '--case', 'gcu', '--dxf', 'gcu_final_clean.dxf', '--output-root', output,
                 '--set', 'geometry_reconstruction.enabled=false', '--set', 'area_graph.alpha.fixed_value=88'],
                cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                timeout=180,
            )
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertNotIn('has no twin', result.stdout)
            manifest = json.loads((Path(output) / 'gcu/run_manifest.json').read_text())
            self.assertTrue(all(c['success'] for c in manifest['commands']))
            with Image.open(ROOT / manifest['outputs']['png']) as source:
                expected_size = source.size
            for key in ('voronoi_skeleton', 'after_dead_end_removal'):
                diagnostic = ROOT / manifest['outputs'][key]
                self.assertTrue(diagnostic.is_file(), str(diagnostic))
                with Image.open(diagnostic) as image:
                    self.assertEqual(image.size, expected_size)
                    image.verify()
            osm = ET.parse(ROOT / manifest['outputs']['osm']).getroot()
            nodes = {n.attrib['id'] for n in osm.findall('node')}
            self.assertEqual(len(nodes), len(osm.findall('node')))
            areas, passages = set(), []
            for way in osm.findall('way'):
                tags = {t.attrib['k']: t.attrib['v'] for t in way.findall('tag')}
                refs = [n.attrib['ref'] for n in way.findall('nd')]
                self.assertTrue(set(refs) <= nodes)
                if tags.get('osmAG:type') == 'area':
                    self.assertGreaterEqual(len(refs), 4)
                    self.assertEqual(refs[0], refs[-1])
                    areas.add(tags['name'])
                elif tags.get('osmAG:type') == 'passage':
                    passages.append(tags)
            self.assertGreater(len(areas), 1)
            self.assertGreater(len(passages), 0)
            for passage in passages:
                self.assertIn(passage['osmAG:from'], areas)
                self.assertIn(passage['osmAG:to'], areas)
                self.assertNotEqual(passage['osmAG:from'], passage['osmAG:to'])
            with (Path(output) / 'gcu/area_graph/room_areas.csv').open() as stream:
                rows = list(csv.reader(stream))
            self.assertEqual({row[0] for row in rows}, areas)
            self.assertTrue(all(math.isfinite(float(row[1])) and float(row[1]) > 0 for row in rows))

    @unittest.skipUnless((ROOT / 'gcu_final_clean.dxf').is_file(), 'Private GCU CAD input unavailable')
    def test_reconstructed_rooms_and_central_exclusion(self):
        import cv2
        import numpy as np
        from shapely.geometry import shape, Point
        (ROOT / 'runs').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='gcu-reconstructed-regression-', dir=ROOT / 'runs') as output:
            case = 'gcu_reconstruction_regression'
            result = subprocess.run(
                [sys.executable, 'run_pipeline.py', '--config', 'config/repro_gcu.yaml',
                 '--case', case, '--dxf', 'gcu_final_clean.dxf', '--output-root', output,
                 '--set', 'area_graph.alpha.fixed_value=120', '--set', 'geometry_reconstruction.enabled=true'],
                cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=180)
            self.assertEqual(result.returncode, 0, result.stdout)
            manifest = json.loads((Path(output) / case / 'run_manifest.json').read_text())
            paths = {k: ROOT / v for k, v in manifest['outputs'].items() if v is not None}
            labels = np.load(paths['reconstruction_labels'])
            source = np.asarray(Image.open(paths['png']).convert('L'))
            self.assertFalse(np.any((labels > 0) & (source <= 127)))
            corridor = labels[400, 370]
            self.assertGreater(corridor, 0)
            side = [labels[y, x] for x, y in ((130,290), (130,390), (130,510), (870,290), (870,390), (870,510))]
            self.assertEqual(len(set(side)), 6)
            self.assertTrue(all(label > 0 and label != corridor for label in side))
            # A white but isolated enclosure must remain excluded without a manual polygon.
            _, connected = cv2.connectedComponents((source > 127).astype(np.uint8), connectivity=4)
            self.assertTrue(np.all(labels[connected == connected[400, 500]] == 0))
            self.assertTrue(all(labels[y, x] == corridor for x, y in ((630,400), (500,160), (500,700))))
            features = json.loads(paths['reconstructed_regions'].read_text())['features']
            by_name = {f['properties']['name']: shape(f['geometry']) for f in features}
            geometries = list(by_name.values())
            self.assertTrue(all(g.is_valid and g.area > 0 and not g.covers(Point(500,400)) for g in geometries))
            self.assertTrue(all(a.intersection(b).area < 1e-6 for i, a in enumerate(geometries) for b in geometries[i+1:]))
            osm = ET.parse(paths['osm']).getroot()
            nodes = {n.attrib['id'] for n in osm.findall('node')}
            ways = {w.attrib['id'] for w in osm.findall('way')}
            from pyproj import CRS, Transformer
            effective = __import__('yaml').safe_load((Path(output) / case / 'effective_config.yaml').read_text())
            reference = effective['root_node']
            resolution = effective['map_preprocessing']['resolution']
            local = CRS.from_proj4(f"+proj=poly +lat_0={reference['latitude']} +lon_0={reference['longitude']} +ellps=WGS84 +units=m +no_defs")
            transform = Transformer.from_crs('EPSG:4326', local, always_xy=True)
            points = {}
            for node in osm.findall('node'):
                east, north = transform.transform(float(node.attrib['lon']), float(node.attrib['lat']))
                points[node.attrib['id']] = (reference['pixel_x'] + east / resolution, reference['pixel_y'] - north / resolution)
            areas, edges = set(), []
            for element in osm:
                tags = {t.attrib['k']:t.attrib['v'] for t in element.findall('tag')}
                self.assertTrue(all(n.attrib['ref'] in nodes for n in element.findall('nd')))
                self.assertTrue(all(m.attrib['ref'] in ways for m in element.findall('member')))
                if tags.get('osmAG:type') == 'area': areas.add(tags['name'])
                elif tags.get('osmAG:type') == 'passage':
                    from shapely.geometry import LineString
                    pair = (tags['osmAG:from'], tags['osmAG:to'])
                    edges.append(pair)
                    interface = LineString([points[n.attrib['ref']] for n in element.findall('nd')])
                    self.assertTrue(all(by_name[name].boundary.buffer(.001).covers(interface) for name in pair))
            self.assertEqual(len(areas), len(geometries))
            self.assertTrue(all(a in areas and b in areas and a != b for a, b in edges))
            reached = {next(iter(areas))}
            while True:
                expanded = reached | {a for a,b in edges if b in reached} | {b for a,b in edges if a in reached}
                if expanded == reached: break
                reached = expanded
            self.assertEqual(reached, areas, 'Exported passage graph is disconnected')
            repeated_cmd = list(result.args)
            repeated_output = Path(output) / 'repeat'
            repeated_cmd[repeated_cmd.index('--output-root') + 1] = str(repeated_output)
            repeated = subprocess.run(repeated_cmd, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, timeout=180)
            self.assertEqual(repeated.returncode, 0, repeated.stdout)
            repeated_manifest = json.loads((repeated_output / case / 'run_manifest.json').read_text())
            repeated_paths = repeated_manifest['outputs']
            np.testing.assert_array_equal(labels, np.load(ROOT / repeated_paths['reconstruction_labels']))
            self.assertEqual(paths['osm'].read_bytes(), (ROOT / repeated_paths['osm']).read_bytes(),
                             'Repeated execution produced different OSM geometry/topology')



if __name__ == '__main__':
    unittest.main()
