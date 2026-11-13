"""Pure Phase 06A mission geometry. No ROS, action clients or motion outputs."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

from shapely.geometry import LineString, Point
from osmag_route_refinement import RouteCorridor


class MissionError(ValueError):
    pass


@dataclass(frozen=True)
class MissionSettings:
    reach_threshold_m: float = .30
    crossing_hysteresis_m: float = .06
    confirmation_samples: int = 3
    maximum_pose_age_s: float = .5
    maximum_observation_gap_s: float = .15
    maximum_pose_step_m: float = .25
    action_ack_timeout_s: float = 15.

    def __post_init__(self):
        values = (self.reach_threshold_m, self.crossing_hysteresis_m,
                  self.maximum_pose_age_s, self.maximum_observation_gap_s,
                  self.maximum_pose_step_m, self.action_ack_timeout_s)
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise MissionError('invalid_mission_settings')
        if type(self.confirmation_samples) is not int or self.confirmation_samples < 2:
            raise MissionError('invalid_confirmation_samples')


@dataclass(frozen=True)
class SegmentGoal:
    index: int
    point: tuple
    yaw: float
    passage_id: str | None
    source_area: str
    destination_area: str


@dataclass(frozen=True)
class MissionContract:
    mission_id: str
    route_revision: int
    closure_revision: int
    map_sha256: str
    settings_sha256: str
    route_sha256: str
    policy: str
    start: tuple
    goal: tuple
    pose_stamp: float
    created_at: float
    area_ids: tuple
    passage_ids: tuple
    disabled_passages: tuple
    path: tuple
    segments: tuple
    settings: MissionSettings

    def report(self):
        return asdict(self)


def sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def make_contract(planner, start, goal, yaw, *, mission_id, now, pose_stamp,
                  route_revision=1, closure_revision=0, policy='refined_crossing',
                  settings=None, disabled_passages=()):
    """Plan afresh from a timestamped pose; never consume a display-topic cache.

    Timestamps must share a clock domain supplied by the caller. Real TF/mask
    readiness and transport ownership are later adapter responsibilities.
    """
    settings = settings or MissionSettings()
    if not isinstance(mission_id, str) or not mission_id or type(route_revision) is not int or route_revision < 1:
        raise MissionError('invalid_mission_identity')
    if type(closure_revision) is not int or closure_revision < 0:
        raise MissionError('invalid_closure_revision')
    if not all(math.isfinite(v) for v in [yaw, now, pose_stamp]):
        raise MissionError('nonfinite_mission_input')
    if not 0 <= now-pose_stamp <= settings.maximum_pose_age_s:
        raise MissionError('start_pose_not_fresh')
    if policy not in ['refined_crossing', 'paper_center']:
        raise MissionError('unsupported_segment_policy')
    route = planner.plan(start, goal, disabled_passages)
    return _contract_from_route(planner, route, yaw, mission_id=mission_id, now=now,
                               pose_stamp=pose_stamp, route_revision=route_revision,
                               closure_revision=closure_revision, policy=policy, settings=settings)


def _contract_from_route(planner, route, yaw, *, mission_id, now, pose_stamp,
                        route_revision, closure_revision, policy, settings):
    """Internal geometry builder; route must come from a fresh planner query."""
    areas = route['area_ids']; passages = route['passage_ids']
    disabled = route['disabled_passages']
    corridor = RouteCorridor(planner.leaves, planner.passage_segments, areas, passages, disabled)
    path = route['path']
    check = corridor.validate(path)
    if not check['valid'] or check['passage_ids'] != passages:
        raise MissionError('invalid_mission_corridor')
    stored = route['refinement']['topology']['crossings']
    if len(stored) != len(check['crossings']) or any(
            a['passage_id'] != b['passage_id'] or math.dist(a['point'], b['point']) > 1e-6
            for a, b in zip(stored, check['crossings'])):
        raise MissionError('crossing_provenance_mismatch')
    points = [list(c['point']) for c in check['crossings']]
    if policy == 'paper_center':
        points = []
        for ident in passages:
            portal = LineString(planner.passage_segments[ident])
            points.append(list(portal.interpolate(.5, normalized=True).coords)[0])
        path = [route['start']]
        for area, target in zip(areas, points+[route['goal']]):
            local = planner.grid_path(area, path[-1], target)
            if local is None:
                raise MissionError('paper_center_local_connection_unvalidated')
            path.extend(local[1:])
        if not corridor.validate(path)['valid']:
            raise MissionError('paper_center_corridor_unvalidated')
    from gcu_plan_validation import validate_path
    physical = lambda p: validate_path(planner.grid, planner.resolution, planner.origin, p, planner.radius)
    if not physical(path)['valid']:
        raise MissionError('mission_path_footprint_blocked')
    segments = []
    for i, (ident, point) in enumerate(zip(passages, points)):
        tags = planner.passages[ident]
        if {tags['osmAG:from'], tags['osmAG:to']} != {areas[i], areas[i+1]}:
            raise MissionError('passage_incident_area_mismatch')
        if Point(point).distance(LineString(planner.passage_segments[ident])) > 1e-6:
            raise MissionError('crossing_outside_interface')
        if not physical([point])['valid']:
            raise MissionError('segment_goal_footprint_blocked')
        delta = [b-a for a, b in zip(point, route['goal'])]
        heading = math.atan2(delta[1], delta[0]) if math.hypot(*delta) > 1e-10 else yaw
        segments.append(SegmentGoal(i, tuple(point), heading, ident, areas[i], areas[i+1]))
    if passages and planner.leaves[areas[-1]].boundary.distance(Point(route['goal'])) < settings.crossing_hysteresis_m:
        raise MissionError('final_goal_inside_crossing_hysteresis')
    segments.append(SegmentGoal(len(passages), tuple(route['goal']), yaw, None, areas[-1], areas[-1]))
    geometry = dict(path=path, areas=areas, passages=passages, segments=[asdict(s) for s in segments])
    return MissionContract(mission_id, route_revision, closure_revision, route['map_sha256'],
                           sha(asdict(settings)), sha(geometry), policy, tuple(route['start']),
                           tuple(route['goal'])+(yaw,), pose_stamp, now, tuple(areas), tuple(passages),
                           tuple(disabled), tuple(tuple(p) for p in path), tuple(segments), settings)


class CrossingMonitor:
    """Ordered observed crossings with boundary hysteresis and fresh samples.

    A pending crossing may jitter back across its portal before confirmation.
    Crossing a different portal or leaving the active incident-area pair fails.
    This checks observations, not physical motion or costmap enforcement.
    """
    def __init__(self, contract, areas, passage_segments):
        self.contract = contract; self.areas = areas; self.portals = passage_segments
        self.confirmed = 0; self.side = 0; self.stable_samples = 0
        self.previous = contract.start; self.stamp = contract.pose_stamp
        self.pending_point = None; self.events = []; self.pairs = {}

    def _pair(self, index, side):
        key = index, side
        if key not in self.pairs:
            names = list(self.contract.area_ids[index:index+2])
            if side:
                names.reverse()
            self.pairs[key] = RouteCorridor(self.areas, self.portals, names,
                                           [self.contract.passage_ids[index]], self.contract.disabled_passages)
        return self.pairs[key]

    def observe(self, point, stamp):
        point = tuple(point)
        cfg = self.contract.settings
        if len(point) != 2 or not all(math.isfinite(v) for v in (*point, stamp)):
            raise MissionError('nonfinite_pose_observation')
        if stamp <= self.stamp:
            raise MissionError('nonincreasing_pose_stamp')
        if stamp-self.stamp > cfg.maximum_observation_gap_s+1e-9:
            raise MissionError('pose_observation_gap')
        if math.dist(point, self.previous) > cfg.maximum_pose_step_m+1e-9:
            raise MissionError('pose_observation_jump')
        i = self.confirmed
        event = None
        if i == len(self.contract.passage_ids):
            corridor = RouteCorridor(self.areas, self.portals, [self.contract.area_ids[-1]], [],
                                     self.contract.disabled_passages)
            check = corridor.segment(self.previous, point, 0)
            if not check['valid']:
                raise MissionError('unexpected_area_transition')
        else:
            check = self._pair(i, self.side).segment(self.previous, point, 0)
            if not check['valid']:
                raise MissionError('unexpected_area_transition:'+check.get('reason', 'unknown'))
            new_side = self.side if check['state'] == 0 else 1-self.side
            if new_side != self.side:
                self.stable_samples = 0
                self.pending_point = check['crossings'][0]['point'] if new_side else None
            self.side = new_side
            destination = self.areas[self.contract.area_ids[i+1]]
            deep = self.side == 1 and destination.contains(Point(point)) and (
                destination.boundary.distance(Point(point)) >= cfg.crossing_hysteresis_m-1e-9)
            self.stable_samples = self.stable_samples+1 if deep else 0
            if self.stable_samples >= cfg.confirmation_samples:
                event = {'index': i, 'passage_id': self.contract.passage_ids[i],
                         'source_area': self.contract.area_ids[i], 'destination_area': self.contract.area_ids[i+1],
                         'crossing_point': self.pending_point, 'confirmed_at': stamp}
                self.events.append(event); self.confirmed += 1
                self.side = 0; self.stable_samples = 0; self.pending_point = None
        self.previous = point; self.stamp = stamp
        return event
