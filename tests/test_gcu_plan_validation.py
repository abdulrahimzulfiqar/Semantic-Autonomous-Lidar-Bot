"""Tests for swept-circle route validation against occupied and unknown cells."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
spec=importlib.util.spec_from_file_location('plan_validation',Path(__file__).resolve().parents[1]/'scripts/gcu_plan_validation.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class RouteValidationTests(unittest.TestCase):
    def test_clear_full_segment(self):
        r=module.validate_path(np.zeros((30,30),dtype=int),.1,(0,0),[(.5,.5),(2.5,.5)])
        self.assertTrue(r['valid']);self.assertGreater(r['samples'],20)

    def test_clear_endpoints_do_not_allow_crossing_wall(self):
        grid=np.zeros((30,30),dtype=int);grid[:,15]=100
        self.assertFalse(module.validate_path(grid,.1,(0,0),[(.5,.5),(2.5,.5)])['valid'])

    def test_unknown_is_blocking(self):
        grid=np.zeros((30,30),dtype=int);grid[:,15]=-1
        self.assertFalse(module.validate_path(grid,.1,(0,0),[(.5,.5),(2.5,.5)])['valid'])

    def test_center_free_but_footprint_hits_wall(self):
        grid=np.zeros((30,30),dtype=int);grid[:,15]=100
        self.assertFalse(module.validate_path(grid,.1,(0,0),[(1.4,1.)])['valid'])

    def test_outside_empty_and_nonfinite_are_rejected(self):
        grid=np.zeros((30,30),dtype=int)
        for points in [[(-.1,1.)],[],[(float('nan'),1.)]]:
            self.assertFalse(module.validate_path(grid,.1,(0,0),points)['valid'])


if __name__=='__main__':unittest.main()
