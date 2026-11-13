import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_segmented_executor import simulation_guard,coherent_pose_time,fresh_scan_at,require_completed_previous
from types import SimpleNamespace
from osmag_mission_contract import MissionError


class ExecutionGuardTest(unittest.TestCase):
    def test_consecutive_start_requires_rest_and_no_old_action_handles(self):
        model=SimpleNamespace(contract=object(),state='succeeded',stop_confirmed=True,_unresolved=lambda:[])
        require_completed_previous(model,{})
        for changes,handles in [({'stop_confirmed':False},{}),({'state':'cancelling'},{}),({}, {'old':object()})]:
            candidate=SimpleNamespace(**(vars(model)|changes))
            with self.assertRaisesRegex(MissionError,'previous_mission'):require_completed_previous(candidate,handles)
    def test_slow_simulation_does_not_make_a_young_scan_stale(self):
        self.assertTrue(fresh_scan_at(20.075,[(20.,5759.2189)]))
        self.assertFalse(fresh_scan_at(20.301,[(20.,5759.2189)]))

    def test_future_scan_does_not_replace_a_usable_past_scan(self):
        self.assertTrue(fresh_scan_at(20.075,[(20.,5759.),(20.1,5761.)]))
        self.assertFalse(fresh_scan_at(20.075,[(20.1,5761.)]))

    def test_future_tf_uses_existing_history_at_current_clock(self):
        self.assertEqual(coherent_pose_time(23.31,23.38),23.31)
        self.assertEqual(coherent_pose_time(23.31,23.32),23.31)

    def test_old_tf_is_never_relabelled_as_current(self):
        self.assertAlmostEqual(coherent_pose_time(23.31,23.2),23.2)
        with self.assertRaisesRegex(MissionError,'stale_robot_tf'):coherent_pose_time(23.31,23.15)
        with self.assertRaisesRegex(MissionError,'invalid_tf_clock'):coherent_pose_time(float('nan'),23.3)

    def test_explicit_execute_and_isolated_simulator_are_all_required(self):
        valid={'ROS_DOMAIN_ID':'195','ROS_LOCALHOST_ONLY':'1','IGN_PARTITION':'gcu_phase6b2b_test'}
        simulation_guard(True,valid)
        for key in valid:
            broken=valid.copy();broken.pop(key)
            with self.assertRaises(MissionError):simulation_guard(True,broken)
        with self.assertRaises(MissionError):simulation_guard(False,valid)

    def test_mock_namespace_or_unisolated_domain_cannot_enable_motion(self):
        for domain,partition in [('77','gcu_phase6b2b_test'),('195','gcu_06b2a_1'),('195','gcu_phase6b1_mock')]:
            with self.assertRaises(MissionError):simulation_guard(True,{'ROS_DOMAIN_ID':domain,'ROS_LOCALHOST_ONLY':'1','IGN_PARTITION':partition})
