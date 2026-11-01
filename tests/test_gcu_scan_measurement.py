"""Synthetic first-hit checks for the read-only scan diagnostic (no ROS needed)."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('gcu_scan', Path(__file__).resolve().parents[1]/'scripts/check_gcu_stationary.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RaycastTests(unittest.TestCase):
    def test_first_wall_and_unbounded_direction(self):
        grid = np.zeros((10, 12), dtype=int)
        grid[:, 7] = 100
        ranges = module.raycast(grid, 1, (0, 0), (2.5, 4.5), [0, np.pi], 10)
        self.assertAlmostEqual(ranges[0], 4.5)
        self.assertTrue(np.isposinf(ranges[1]))

    def test_unknown_space_is_not_physical_wall(self):
        grid = np.full((8, 8), -1)
        grid[6, :] = 100
        ranges = module.raycast(grid, .5, (-2, -2), (-.75, -.75), [np.pi/2], 4)
        self.assertAlmostEqual(ranges[0], 1.75)


if __name__ == '__main__':
    unittest.main()
