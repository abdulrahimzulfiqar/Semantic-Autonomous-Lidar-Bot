"""Footprint audit geometry and unsupported-geometry checks (no ROS required)."""
import importlib.util
import tempfile
from pathlib import Path
import unittest
import numpy as np
spec=importlib.util.spec_from_file_location('audit',Path(__file__).resolve().parents[1]/'scripts/audit_gcu_footprint.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class FootprintTests(unittest.TestCase):
    def audit(self,xml):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'robot.urdf';p.write_text(xml);return module.audit(p)

    def test_offset_shape_exceeds_radius(self):
        r=self.audit('<robot><link name="base_footprint"><collision><origin xyz="0.1 0 0"/><geometry><box size="0.2 0.1 0.1"/></geometry></collision></link></robot>')
        self.assertFalse(r['radius_covers_nominal_collision_bounds'])
        np.testing.assert_allclose(r['maximum_xy_m'],[.2,.05])

    def test_fixed_frame_rotation(self):
        r=self.audit('<robot><link name="base_footprint"/><link name="child"><collision><geometry><box size="0.2 0.1 0.1"/></geometry></collision></link><joint name="fixed" type="fixed"><parent link="base_footprint"/><child link="child"/><origin xyz="0 0.2 0" rpy="0 0 1.5707963267948966"/></joint></robot>')
        np.testing.assert_allclose(r['minimum_xy_m'],[-.05,.1]);np.testing.assert_allclose(r['maximum_xy_m'],[.05,.3])

    def test_mesh_collision_is_not_silently_ignored(self):
        with self.assertRaises(ValueError):
            self.audit('<robot><link name="base_footprint"><collision><geometry><mesh filename="unavailable.stl"/></geometry></collision></link></robot>')


if __name__=='__main__':unittest.main()
