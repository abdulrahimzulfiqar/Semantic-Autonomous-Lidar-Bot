"""Transport-free Phase 06A action state model, exercised with mock events.

Returned intents are data, not ROS calls. No runtime execution, TF watchdog,
costmap constraint readiness or physical rest measurement is implemented here.
Those are mandatory later adapter gates before using this model for motion.
"""
from dataclasses import dataclass
import math
from osmag_mission_contract import MissionError


@dataclass
class Attempt:
    token: str
    segment: int
    sent_at: float
    uuid: str | None = None
    terminal: bool = False
    superseded: bool = False
    cancel_sent: bool = False


class MissionState:
    def __init__(self):
        self.state = 'idle'; self.contract = None; self.generation = 0
        self.attempts = {}; self.active = None; self.pending = None
        self.index = -1; self.confirmed = 0; self.reason = None
        self.final_action_succeeded = False; self.stop_confirmed = False
        self.clock = -math.inf; self.intermediate_succeeded = False

    def _time(self, now):
        if not math.isfinite(now) or now < self.clock:
            raise MissionError('invalid_event_clock')
        self.clock = now

    def _unresolved(self):
        return [a for a in self.attempts.values() if not a.terminal]

    def _send(self, index, now):
        if self.active is not None:
            # The old terminal result can arrive before the new acceptance ACK.
            self.attempts[self.active].superseded = True
        token = f'{self.generation}:{len(self.attempts)}:{index}'
        self.attempts[token] = Attempt(token, index, now)
        self.pending = token; self.index = index; self.state = 'dispatching'
        self.intermediate_succeeded = False
        return [{'kind': 'send', 'token': token, 'mission_id': self.contract.mission_id,
                 'route_revision': self.contract.route_revision, 'segment': self.contract.segments[index]}]

    def start(self, contract, now):
        self._time(now)
        if self.state not in ['idle', 'succeeded', 'cancelled', 'failed'] or self._unresolved():
            raise MissionError('busy_cancel_before_new_mission')
        if now < contract.created_at or now-contract.pose_stamp > contract.settings.maximum_pose_age_s:
            raise MissionError('mission_start_pose_not_fresh')
        self.contract = contract; self.generation += 1; self.attempts = {}
        self.active = None; self.pending = None; self.confirmed = 0; self.reason = None
        self.final_action_succeeded = False; self.stop_confirmed = False
        return self._send(0, now)

    def _cancel_intents(self):
        effects = []
        for attempt in self._unresolved():
            if attempt.uuid is not None and not attempt.cancel_sent:
                effects.append({'kind': 'cancel', 'uuid': attempt.uuid, 'token': attempt.token})
                attempt.cancel_sent = True
        return effects

    def cancel(self, now, reason='user_cancel'):
        self._time(now)
        if self.contract is None or self.state in ['succeeded', 'cancelled']:
            return []
        self.state = 'cancelling'; self.reason = reason
        return self._cancel_intents()

    def accepted(self, token, uuid, accepted, now):
        self._time(now)
        if accepted and (not isinstance(uuid, str) or not uuid):
            raise MissionError('missing_action_uuid')
        attempt = self.attempts.get(token)
        if attempt is None:
            # A handle from an obsolete generation must never start driving.
            return [{'kind': 'cancel', 'uuid': uuid, 'token': token}] if accepted else []
        if attempt.uuid is not None or attempt.terminal:
            if accepted and attempt.uuid != uuid:
                return self.cancel(now, 'duplicate_action_handle')+[{'kind': 'cancel', 'uuid': uuid, 'token': token}]
            return []
        if not accepted:
            attempt.terminal = True
            if token == self.pending:
                self.pending = None
            if self.state in ['cancelling', 'failed']:
                return []
            return self.cancel(now, 'navigation_action_rejected')
        attempt.uuid = uuid
        if token == self.pending:
            self.pending = None
        if self.state in ['cancelling', 'failed']:
            return self._cancel_intents()
        if self.active is not None:
            self.attempts[self.active].superseded = True
        self.active = token; self.state = 'executing'
        return []

    def result(self, uuid, outcome, now):
        self._time(now)
        if outcome not in ['succeeded', 'cancelled', 'aborted']:
            raise MissionError('unknown_action_result')
        attempt = next((a for a in self.attempts.values() if a.uuid == uuid), None)
        if attempt is None or attempt.terminal:
            return []
        attempt.terminal = True
        if attempt.superseded or self.state in ['cancelling', 'failed']:
            return []
        if self.active != attempt.token:
            return []
        self.active = None
        if outcome != 'succeeded':
            return self.cancel(now, 'navigation_action_'+outcome)
        if attempt.segment == len(self.contract.segments)-1:
            self.final_action_succeeded = True; self.state = 'verifying'
        else:
            self.intermediate_succeeded = True; self.state = 'waiting_handoff'
        return []

    def advance(self, point, confirmed_crossings, now):
        """Use fresh pose + CrossingMonitor count from the future runtime adapter.

        Proximity can dispatch ONE goal beyond an unconfirmed crossing; a count
        is never incremented by a successful action or reaching its point.
        """
        self._time(now)
        if self.contract is None:
            raise MissionError('no_mission')
        if not all(math.isfinite(v) for v in point) or len(point) != 2:
            raise MissionError('invalid_progress_pose')
        if type(confirmed_crossings) is not int or not self.confirmed <= confirmed_crossings <= min(
                self.confirmed+1, len(self.contract.passage_ids)):
            raise MissionError('invalid_crossing_progress')
        self.confirmed = confirmed_crossings
        if self.state not in ['executing', 'waiting_handoff'] or self.pending is not None:
            return []
        if self.index == len(self.contract.segments)-1:
            return []
        near = math.dist(point, self.contract.segments[self.index].point) <= self.contract.settings.reach_threshold_m
        if (near or self.intermediate_succeeded) and self.confirmed >= self.index:
            return self._send(self.index+1, now)
        return []

    def check_deadlines(self, now):
        self._time(now)
        if self.contract is None:
            return []
        overdue = [a for a in self._unresolved() if a.uuid is None and
                   now-a.sent_at > self.contract.settings.action_ack_timeout_s]
        if overdue:
            effects = self.cancel(now, 'action_ack_timeout')
            self.state = 'failed'  # Stop is UNKNOWN; unresolved requests still block a new mission.
            return effects
        return []

    def confirm_stopped(self, rest_confirmed, now):
        self._time(now)
        if self.state not in ['cancelling', 'failed'] or self._unresolved() or not rest_confirmed:
            return False
        self.stop_confirmed = True
        self.state = 'cancelled' if self.reason == 'user_cancel' else 'failed'
        return True

    def confirm_final(self, pose, rest_confirmed, now):
        self._time(now)
        if self.state != 'verifying' or self._unresolved() or not rest_confirmed:
            return False
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise MissionError('invalid_final_pose')
        if self.confirmed != len(self.contract.passage_ids):
            return False
        goal = self.contract.goal
        heading = abs(math.atan2(math.sin(pose[2]-goal[2]), math.cos(pose[2]-goal[2])))
        if math.dist(pose[:2], goal[:2]) > .20 or heading > math.radians(10):
            return False
        self.stop_confirmed = True; self.state = 'succeeded'
        return True

    def report(self):
        return {'state': self.state, 'mission_id': self.contract.mission_id if self.contract else None,
                'last_dispatched_segment': self.index, 'confirmed_crossings': self.confirmed,
                'reason': self.reason, 'stop_confirmed': self.stop_confirmed,
                'unresolved_requests': len(self._unresolved()), 'scope': 'pure_mock_model_no_motion'}
