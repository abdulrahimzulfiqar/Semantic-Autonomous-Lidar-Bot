"""Run with the CAD pipeline venv: python -m unittest discover -s tests -p test_gcu_simulation.py."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from shapely.geometry import Polygon, Point, shape
from shapely import union_all

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('prepare_gcu_simulation',ROOT/'scripts/prepare_gcu_simulation.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
MANIFEST=ROOT/'docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/runs/local-cad/gcu_config_parameters_testing_12/run_manifest.json'


class GeometryTests(unittest.TestCase):
    def test_partition_preserves_courtyard(self):
        polygon=Polygon([(0,0),(20,0),(20,20),(0,20)], [[(5,5),(5,15),(15,15),(15,5)]])
        parts=module.simple_parts(polygon)
        self.assertGreater(len(parts),1)
        self.assertTrue(all(p.is_valid and not p.interiors for p in parts))
        self.assertEqual(union_all(parts).symmetric_difference(polygon).area,0)
        self.assertFalse(any(p.covers(Point(10,10)) for p in parts))

    def test_wall_boxes_cover_only_wall_pixels(self):
        wall=np.array([[0,1,1,0],[0,1,1,1],[0,0,0,1]],bool)
        restored=np.zeros_like(wall)
        for x0,y0,x1,y1 in module.rectangles(wall):
            restored[int(y0+.5):int(y1+.5),int(x0+.5):int(x1+.5)]=True
        np.testing.assert_array_equal(wall,restored)

    def test_passage_endpoints_become_ring_nodes(self):
        polygon=Polygon([(0,0),(10,0),(10,10),(0,10)])
        result=module.node_ring(polygon,{(3,0),(7,0)})
        ring=list(result.exterior.coords)
        self.assertTrue(any({a,b}=={(3,0),(7,0)} for a,b in zip(ring,ring[1:])))
        self.assertEqual(result.symmetric_difference(polygon).area,0)

    @unittest.skipUnless(MANIFEST.exists(),'Local GCU run unavailable')
    def test_gcu_artifact_contract(self):
        with tempfile.TemporaryDirectory() as folder:
            directory=Path(folder);report=module.prepare(MANIFEST,directory)
            osm=ET.parse(directory/'gcu_ground_floor.osm').getroot()
            self.assertFalse(osm.findall('relation'))
            areas={};adj={};passages=[]
            for way in osm.findall('way'):
                tags={t.attrib['k']:t.attrib['v'] for t in way.findall('tag')}
                if tags.get('osmAG:type')=='area':
                    areas[tags['name']]=tags
                    if tags['osmAG:areaType']!='structure':self.assertEqual(tags['osmAG:parent'],'gcu_gf_floor')
                elif tags.get('osmAG:type')=='passage':
                    self.assertEqual(len(way.findall('nd')),2)
                    a,b=tags['osmAG:from'],tags['osmAG:to'];passages.append((a,b));adj.setdefault(a,set()).add(b);adj.setdefault(b,set()).add(a)
            leaf=set(areas)-{'gcu_gf_floor'};seen={next(iter(leaf))}
            while True:
                other=seen|set().union(*(adj.get(name,set()) for name in seen))
                if other==seen:break
                seen=other
            self.assertEqual(seen,leaf)
            geometries=[shape(f['geometry']) for f in json.loads((directory/'regions_pixels.geojson').read_text())['features'] if f['properties']['type']!='structure']
            self.assertFalse(any(p.covers(Point(500,400)) for p in geometries))
            self.assertTrue(all(a.intersection(b).area<1e-6 for i,a in enumerate(geometries) for b in geometries[i+1:]))
            sdf=ET.parse(directory/'gcu_ground_floor.sdf').getroot()
            self.assertEqual(len(sdf.findall('.//model[@name="cad_walls"]/link/collision')),report['wall_boxes'])
            config=__import__('yaml').safe_load((directory/'gcu_ground_floor.yaml').read_text())
            self.assertEqual(config['resolution'],report['resolution'])
            image=np.asarray(Image.open(directory/'gcu_ground_floor.pgm'))
            x,y=report['spawn']['pixel'];self.assertEqual(image[y,x],254)
            self.assertEqual(image[400,500],205)
            self.assertFalse(report['runtime_tested'])


if __name__=='__main__':unittest.main()
