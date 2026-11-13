import sys,unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_user_mission import parse_goal,require_recoverable
from osmag_mission_contract import MissionError


class UserMissionTest(unittest.TestCase):
    def test_planar_map_goal(self):
        self.assertEqual(parse_goal('map',[1.,2.],[0.,0.,0.,1.]),[1.,2.,0.])

    def test_invalid_frame_nonfinite_and_orientation_rejected(self):
        for frame,point,q in [('odom',[1.,2.],[0.,0.,0.,1.]),('map',[float('nan'),2.],[0.,0.,0.,1.]),('map',[1.,2.],[1.,0.,0.,0.])]:
            with self.assertRaises(MissionError):parse_goal(frame,point,q)

    def test_recovery_requires_rest_and_resolved_actions(self):
        m=SimpleNamespace(state='cancelled',stop_confirmed=False,_unresolved=lambda:[])
        with self.assertRaises(MissionError):require_recoverable(m,{})
        m.stop_confirmed=True
        with self.assertRaises(MissionError):require_recoverable(m,{'old':'handle'})
        require_recoverable(m,{})
