import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_physical_obstacle import overlay_box
from gcu_plan_validation import validate_path


class PhysicalObstacleTest(unittest.TestCase):
    def test_evaluation_overlay_blocks_reference_without_changing_navigation_map(self):
        grid=np.zeros((50,50),dtype=np.int16);path=[[.5,2.5],[4.5,2.5]]
        before=grid.copy();evaluation=overlay_box(grid,.1,(0.,0.),{'center':[2.5,2.5],'size':[.5,.5,.6]})
        self.assertTrue(validate_path(grid,.1,(0.,0.),path,.18)['valid'])
        self.assertFalse(validate_path(evaluation,.1,(0.,0.),path,.18)['valid'])
        self.assertTrue(np.array_equal(grid,before))

    def test_invalid_geometry_rejected(self):
        with self.assertRaisesRegex(ValueError,'invalid_obstacle'):
            overlay_box(np.zeros((10,10)),.1,(0.,0.),{'center':[0.,0.],'size':[float('nan'),.5,.6]})
