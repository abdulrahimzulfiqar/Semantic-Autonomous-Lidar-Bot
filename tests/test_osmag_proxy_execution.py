from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_mission_state import MissionState
from osmag_mission_contract import MissionError
from test_osmag_mission_contract import contract,FixturePlanner
from osmag_window_route import prefix_to_target


class ProxyExecutionTests(unittest.TestCase):
    def setUp(self):
        self.c=contract(FixturePlanner(three=True),goal=(3.,1.))
        self.m=MissionState();self.token=self.m.start(self.c,0.)[0]['token']
        self.m.accepted(self.token,'a',True,.01)

    def test_proxy_success_does_not_advance_or_finish_original_segment(self):
        self.m.result('a','succeeded',.02,proxy=True)
        self.assertEqual(self.m.state,'waiting_proxy');self.assertFalse(self.m.final_action_succeeded)
        self.assertEqual(self.m.confirmed,0);self.assertEqual(self.m.index,0)
        self.assertEqual(self.m.advance((-.1,1.),0,.03),[])

    def test_new_mission_preflight_ignores_previous_mission_segment_index(self):
        from dataclasses import replace
        from gcu_rolling_window_manager import RollingWindowManager
        manager=RollingWindowManager.__new__(RollingWindowManager)
        manager.contract=replace(self.c,mission_id='new',route_revision=2)
        manager.adapter=SimpleNamespace(model=SimpleNamespace(contract=self.c,index=1))
        self.assertEqual(manager.preflight_segments(),list(manager.contract.segments))
        manager.adapter.model.contract=manager.contract
        self.assertEqual(manager.preflight_segments(),list(manager.contract.segments)[1:])

    def test_refresh_reuses_original_segment_and_supersedes_old_handle(self):
        effect=self.m.repeat_current(.02)[0]
        self.assertEqual(effect['segment'].index,0);self.assertTrue(self.m.attempts[self.token].superseded)
        self.m.result('a','aborted',.03,proxy=True)
        self.m.accepted(effect['token'],'b',True,.04)
        self.assertEqual(self.m.state,'executing');self.assertEqual(self.m.index,0)

    def test_original_handoff_is_suppressed_while_native_target_is_proxy(self):
        self.assertEqual(self.m.advance((-.1,1.),0,.02,allow_handoff=False),[])
        self.assertEqual(self.m.index,0)
        self.assertEqual(self.m.advance((-.1,1.),0,.03)[0]['segment'].index,1)

    def test_cancel_blocks_proxy_refresh_and_cancels_late_acceptance(self):
        pending=self.m.repeat_current(.02)[0]['token'];self.m.cancel(.03)
        with self.assertRaises(MissionError):self.m.repeat_current(.04)
        effects=self.m.accepted(pending,'late',True,.05)
        self.assertTrue(any(e['uuid']=='late' and e['kind']=='cancel' for e in effects))

    def test_reference_prefix_keeps_corner_instead_of_direct_shortcut(self):
        path=prefix_to_target([(0.,0.),(2.,0.),(2.,2.)],(.5,.1),(2.,2.))
        self.assertIn((2.,0.),path);self.assertEqual(path[0],(.5,.1))

    def test_off_reference_original_target_is_rejected(self):
        with self.assertRaises(ValueError):prefix_to_target([(0.,0.),(2.,0.)],(.5,0.),(1.,1.))

    def test_contact_burst_does_not_trigger_unbounded_window_pose_queries(self):
        from gcu_rolling_window_manager import RollingWindowManager
        calls=[]
        manager=RollingWindowManager.__new__(RollingWindowManager)
        manager.busy=False;manager.frozen=False;manager.last_tick_wall=-float('inf');manager.committed=None
        manager.adapter=SimpleNamespace(enabled=True,model=SimpleNamespace(state='executing'),
            pose=lambda:(calls.append(True) or ([0.,0.,0.],1.)))
        with patch('gcu_rolling_window_manager.time.monotonic',return_value=10.):
            for _ in range(1000):manager.tick()
        self.assertEqual(len(calls),1)
        with patch('gcu_rolling_window_manager.time.monotonic',return_value=10.1):manager.tick()
        self.assertEqual(len(calls),2)


if __name__=='__main__':unittest.main()
