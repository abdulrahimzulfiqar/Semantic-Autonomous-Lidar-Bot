"""Regress physical-only shortcuts against the real logical execution mask."""
import copy,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_paper_planner import PaperPlanner
from osmag_execution_route import constrain_route
from osmag_constraint_mask import build_mask
from gcu_plan_validation import validate_path


class ExecutionRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner=PaperPlanner(Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor')

    def test_mask_corner_is_replanned_without_changing_passages(self):
        p=self.planner;route=p.plan([-.5,19.9],[3.9,22.3])
        incidence={n:(v['osmAG:from'],v['osmAG:to']) for n,v in p.passages.items()}
        mask=build_mask(p.grid,p.resolution,p.origin,p.leaves,p.passage_segments,incidence,allowed_areas=route['area_ids'],open_passages=route['passage_ids'],disabled_passages=[],robot_pose=route['start'],mission_id='regression')
        self.assertFalse(validate_path(mask.combined,p.resolution,p.origin,route['path'],.20)['valid'])
        improved=constrain_route(p,route,.02)
        self.assertEqual(improved['passage_ids'],route['passage_ids'])
        self.assertTrue(validate_path(mask.combined,p.resolution,p.origin,improved['path'],.20)['valid'])
        self.assertEqual(route['path'][0],improved['path'][0]);self.assertEqual(route['path'][-1],improved['path'][-1])

    def test_blocked_selected_interface_is_rejected_without_old_route_fallback(self):
        p=self.planner;route=p.plan([-.5,19.9],[3.9,22.3]);blocked=copy.copy(p);blocked.grid=p.grid.copy()
        portal=p.passage_segments[route['passage_ids'][0]];x=sum(v[0] for v in portal)/len(portal)
        col=int((x-p.origin[0])/p.resolution);blocked.grid[:,col]=100
        with self.assertRaisesRegex(ValueError,'constraint_'):
            constrain_route(blocked,route,.02)
