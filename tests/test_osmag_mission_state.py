import dataclasses
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_mission_state import MissionState
from osmag_mission_contract import MissionError
from test_osmag_mission_contract import contract, FixturePlanner


class MissionStateTests(unittest.TestCase):
    def setUp(self):
        self.c = contract(FixturePlanner(three=True), goal=(3., 1.))
        self.m = MissionState()
        self.first = self.m.start(self.c, 0.)[0]['token']

    def accept_first(self):
        self.m.accepted(self.first, 'a', True, .01)

    def next(self):
        self.accept_first()
        return self.m.advance((-.1, 1.), 0, .02)[0]['token']

    def test_busy_click_does_not_replace_active_mission(self):
        with self.assertRaisesRegex(MissionError, 'busy'):
            self.m.start(self.c, .1)
        self.assertEqual(self.m.pending, self.first)

    def test_proximity_dispatches_next_without_waiting_for_action_result(self):
        second = self.next()
        self.assertEqual(self.m.index, 1); self.assertEqual(self.m.confirmed, 0)
        self.assertFalse(self.m.attempts[self.first].terminal)
        self.assertEqual(self.m.pending, second)

    def test_two_unconfirmed_portals_cannot_be_skipped(self):
        second = self.next(); self.m.accepted(second, 'b', True, .03)
        self.assertEqual(self.m.advance((2., 1), 0, .04), [])
        self.assertEqual(self.m.index, 1)
        effects = self.m.advance((2., 1), 1, .05)
        self.assertEqual(effects[0]['segment'].index, 2)

    def test_old_abort_before_new_acceptance_is_supersession_not_failure(self):
        second = self.next()
        self.assertEqual(self.m.result('a', 'aborted', .03), [])
        self.assertEqual(self.m.pending, second)
        self.m.accepted(second, 'b', True, .04)
        self.assertEqual(self.m.state, 'executing')

    def test_late_old_success_cannot_complete_new_segment(self):
        second = self.next(); self.m.accepted(second, 'b', True, .03)
        self.m.result('a', 'succeeded', .04)
        self.assertEqual(self.m.active, second); self.assertEqual(self.m.state, 'executing')

    def test_cancel_during_pending_send_cancels_late_handle_and_requires_rest(self):
        self.assertEqual(self.m.cancel(.01), [])
        self.assertFalse(self.m.confirm_stopped(True, .02))
        self.assertEqual(self.m.accepted(self.first, 'a', True, .03)[0]['kind'], 'cancel')
        self.m.result('a', 'cancelled', .04)
        self.assertFalse(self.m.confirm_stopped(False, .05))
        self.assertTrue(self.m.confirm_stopped(True, .06))
        self.assertEqual(self.m.state, 'cancelled')

    def test_cancel_during_handoff_cancels_both_handles(self):
        second = self.next()
        first_cancel = self.m.cancel(.03)
        late_cancel = self.m.accepted(second, 'b', True, .04)
        self.assertEqual([e['uuid'] for e in first_cancel+late_cancel], ['a', 'b'])
        self.assertEqual(self.m.advance((2, 1), 1, .05), [])

    def test_rejected_preemption_stops_previous_goal(self):
        second = self.next()
        effects = self.m.accepted(second, '', False, .03)
        self.assertEqual(effects[0]['uuid'], 'a'); self.assertEqual(self.m.state, 'cancelling')
        self.assertEqual(self.m.reason, 'navigation_action_rejected')

    def test_ack_timeout_does_not_claim_stopped_or_allow_new_mission(self):
        self.m.check_deadlines(16.)
        self.assertEqual(self.m.state, 'failed'); self.assertFalse(self.m.stop_confirmed)
        with self.assertRaisesRegex(MissionError, 'busy'):
            self.m.start(self.c, 16.)
        self.assertEqual(self.m.accepted(self.first, 'late', True, 16.1)[0]['uuid'], 'late')

    def test_final_action_success_still_needs_crossings_pose_and_rest(self):
        second = self.next(); self.m.accepted(second, 'b', True, .03)
        self.m.result('a', 'cancelled', .04)
        last = self.m.advance((2., 1.), 1, .05)[0]['token']
        self.m.accepted(last, 'final', True, .06); self.m.result('b', 'cancelled', .07)
        self.m.result('final', 'succeeded', .08)
        self.assertEqual(self.m.state, 'verifying')
        self.assertFalse(self.m.confirm_final((3., 1., .7), True, .09))
        self.m.advance((3., 1.), 2, .10)
        self.assertFalse(self.m.confirm_final((3., 1., .7), False, .11))
        self.assertFalse(self.m.confirm_final((3., 1., 1.7), True, .12))
        self.assertFalse(self.m.confirm_final((3.3, 1., .7), True, .13))
        self.assertTrue(self.m.confirm_final((3., 1., .7), True, .14))

    def test_same_sector_uses_final_action_only(self):
        c = contract(start=(-1., 0.), goal=(-.5, .5))
        m = MissionState(); token = m.start(c, 0)[0]['token']
        m.accepted(token, 'final', True, .01); m.result('final', 'succeeded', .02)
        self.assertTrue(m.confirm_final((-.5, .5, .7), True, .03))

    def test_obsolete_generation_handle_is_cancelled_without_overwriting_current(self):
        self.m.cancel(.01); self.m.accepted(self.first, '', False, .02)
        self.m.confirm_stopped(True, .03)
        current = self.m.start(dataclasses.replace(self.c, mission_id='next'), .04)[0]['token']
        effects = self.m.accepted(self.first, 'obsolete', True, .05)
        self.assertEqual(effects[0]['uuid'], 'obsolete'); self.assertEqual(self.m.pending, current)

    def test_stale_start_and_nonmonotonic_event_clock_reject(self):
        with self.assertRaisesRegex(MissionError, 'invalid_event_clock'):
            self.m.accepted(self.first, 'a', True, -.1)
        m = MissionState()
        with self.assertRaisesRegex(MissionError, 'not_fresh'):
            m.start(self.c, 1.)

    def test_invalid_or_decreasing_crossing_progress_rejects(self):
        self.accept_first()
        with self.assertRaisesRegex(MissionError, 'invalid_crossing_progress'):
            self.m.advance((0., 1.), 2, .02)


if __name__ == '__main__':
    unittest.main()
