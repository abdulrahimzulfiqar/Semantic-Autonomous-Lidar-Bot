"""Run with ./venv/bin/python area_graph_segment/test/test_reconstruction.py."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import Point, Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from reconstruct_regions import grow_labels, regrow_without_small_basins, mask_geometry, export_osm, interfaces


class ReconstructionTests(unittest.TestCase):
    def test_walls_and_unseeded_courtyard_are_not_crossed(self):
        free = np.ones((30, 30), dtype=bool)
        free[8:22, 8] = free[8:22, 21] = False
        free[8, 8:22] = free[21, 8:22] = False
        seeds = np.zeros(free.shape, np.int32)
        seeds[4, 4], seeds[25, 25] = 1, 2
        labels = grow_labels(free, seeds)
        self.assertEqual(labels[15, 15], 0)
        self.assertTrue(np.all(labels[~free] == 0))
        self.assertEqual(labels[4, 4], 1)
        self.assertEqual(labels[25, 25], 2)
        np.testing.assert_array_equal(labels, grow_labels(free, seeds))

    def test_filtering_tiny_basin_does_not_make_a_wall(self):
        free = np.ones((1, 11), bool)
        seeds = np.zeros((1, 11), np.int32)
        seeds[0, 0], seeds[0, 5], seeds[0, 10] = 1, 2, 3
        labels = grow_labels(free, seeds)
        # Force a tiny middle label between two large surviving basins.
        labels[0, :5], labels[0, 5], labels[0, 6:] = 1, 2, 3
        result = regrow_without_small_basins(free, seeds, labels, 3)
        self.assertNotIn(2, np.unique(result))
        self.assertTrue(np.all(result > 0))
        self.assertEqual(len(list(interfaces(result))), 1)

    def test_diagonal_contact_is_not_an_opening(self):
        free = np.eye(3, dtype=bool)
        seeds = np.zeros((3, 3), np.int32)
        seeds[0, 0] = 1
        self.assertEqual(grow_labels(free, seeds)[1, 1], 0)

    def test_contours_preserve_inner_boundary(self):
        mask = np.ones((30, 30), dtype=np.uint8)
        mask[8:22, 8:22] = 0
        polygon = mask_geometry(mask)
        self.assertTrue(polygon.is_valid)
        self.assertEqual(len(polygon.interiors), 1)
        self.assertEqual(polygon.area, 30 * 30 - 14 * 14)
        self.assertFalse(polygon.covers(Point(15, 15)))
        self.assertTrue(polygon.covers(Point(4, 4)))

    def test_hole_export_uses_relation_and_resolvable_members(self):
        polygon = Polygon([(0,0), (30,0), (30,30), (0,30)], [[(8,8), (8,22), (22,22), (22,8)]])
        config = {'root_node': {'latitude': 31, 'longitude': 121, 'pixel_x': 0, 'pixel_y': 0},
                  'map_preprocessing': {'resolution': 1}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'map.osm'
            export_osm(path, {1: {'geometry': polygon}}, [], config)
            osm = ET.parse(path).getroot()
            self.assertEqual(osm.find('node').find('tag').attrib, {'k': 'name', 'v': 'root'})
            relation = osm.find('relation')
            self.assertIsNotNone(relation)
            self.assertEqual({m.attrib['role'] for m in relation.findall('member')}, {'outer', 'inner'})
            ways = {w.attrib['id'] for w in osm.findall('way')}
            self.assertTrue(all(m.attrib['ref'] in ways for m in relation.findall('member')))
            node_ids = {n.attrib['id'] for n in osm.findall('node')}
            self.assertTrue(all(n.attrib['ref'] in node_ids for w in osm.findall('way') for n in w.findall('nd')))
            # Area semantics belong to the relation; outer way must not claim a filled room.
            self.assertFalse(any(t.attrib.get('k') == 'osmAG:type' for w in osm.findall('way') for t in w.findall('tag')))

    def test_shared_interface_stays_on_free_label_border(self):
        labels = np.ones((10, 10), np.int32)
        labels[:, 5:] = 2
        passage = list(interfaces(labels))
        self.assertEqual(len(passage), 1)
        pair, line = passage[0]
        self.assertEqual(pair, (1, 2))
        self.assertTrue(all(x == 4.5 for x, y in line.coords))
        for label in pair:
            self.assertTrue(mask_geometry(labels == label).boundary.covers(line))


if __name__ == '__main__':
    unittest.main()
