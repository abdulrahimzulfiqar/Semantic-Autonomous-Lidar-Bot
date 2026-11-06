"""Visibility refinement inside an ordered, permitted area/passage corridor.

This geometric refinement is our extension, not an author-runtime component.
It preserves the selected transition sequence; it does not optimize over every
possible passage sequence or claim a continuous-space global optimum.
"""
import math
from shapely.geometry import LineString, Point
from shapely.ops import unary_union


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def coordinates(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type in ['Point', 'LineString', 'LinearRing']:
        return list(geometry.coords)
    return [point for part in geometry.geoms for point in coordinates(part)]


def remove_collinear(path):
    result = []
    for point in path:
        point = list(point)
        if result and math.dist(result[-1], point) < 1e-10:
            continue
        while len(result) >= 2:
            a, b = result[-2:]
            u = (b[0]-a[0], b[1]-a[1]); v = (point[0]-b[0], point[1]-b[1])
            if abs(u[0]*v[1]-u[1]*v[0]) > 1e-10 or u[0]*v[0]+u[1]*v[1] < 0:
                break
            result.pop()
        result.append(point)
    return result


class RouteCorridor:
    tolerance = 1e-7  # Serialization-scale tolerance, never occupancy clearance.

    def __init__(self, areas, passage_segments, area_ids, passage_ids, disabled=()):
        if len(area_ids) != len(passage_ids)+1 or not area_ids:
            raise ValueError('invalid_route_corridor')
        if set(passage_ids) & set(disabled):
            raise ValueError('disabled_selected_passage')
        self.area_ids = list(area_ids); self.passage_ids = list(passage_ids)
        self.polygons = [areas[name].buffer(self.tolerance) for name in area_ids]
        self.portals = [LineString(passage_segments[name]) for name in passage_ids]
        self.closed = [LineString(passage_segments[name]) for name in disabled]
        self.union = unary_union(self.polygons)

    def segment(self, a, b, state, expected=None):
        """Check exact polygon coverage and ordered permitted portal crossings."""
        if math.dist(a, b) < 1e-10:
            return {'valid': expected is None or state == expected, 'state': state, 'crossings': []}
        line = LineString([a, b])
        if not self.union.covers(line):
            return {'valid': False, 'reason': 'outside_route_corridor'}
        if any(line.intersects(portal) for portal in self.closed):
            return {'valid': False, 'reason': 'closed_passage_intersection'}
        cuts = [0., 1.]
        for portal in self.portals:
            cuts.extend(line.project(Point(p), normalized=True) for p in coordinates(line.intersection(portal)))
        cuts = sorted(set(round(max(0., min(1., t)), 12) for t in cuts))
        crossings = []
        for left, right in zip(cuts, cuts[1:]):
            if right-left < 1e-12:
                continue
            begin = line.interpolate(left, normalized=True); end = line.interpolate(right, normalized=True)
            piece = LineString([begin, end])
            if self.polygons[state].covers(piece):
                continue
            if (state+1 >= len(self.polygons) or not self.polygons[state+1].covers(piece)
                    or begin.distance(self.portals[state]) > self.tolerance):
                return {'valid': False, 'reason': 'transition_sequence_changed'}
            crossings.append({'passage_id': self.passage_ids[state], 'point': list(begin.coords)[0]})
            state += 1
        if expected is not None and state != expected:
            return {'valid': False, 'reason': 'transition_progress_changed'}
        return {'valid': True, 'state': state, 'crossings': crossings}

    def validate(self, path):
        if not path or not self.polygons[0].covers(Point(path[0])):
            return {'valid': False, 'reason': 'invalid_corridor_start'}
        state = 0; states = [0]; crossings = []
        for a, b in zip(path, path[1:]):
            check = self.segment(a, b, state)
            if not check['valid']:
                return check
            state = check['state']; states.append(state); crossings.extend(check['crossings'])
        if state != len(self.polygons)-1 or not self.polygons[-1].covers(Point(path[-1])):
            return {'valid': False, 'reason': 'incomplete_transition_sequence'}
        return {'valid': True, 'vertex_states': states, 'crossings': crossings,
                'passage_ids': [item['passage_id'] for item in crossings]}


def refine_route(path, corridor, physical_check, maximum_attempts=512):
    """Greedy farthest-visible string pulling with an explicit work budget.

    A failed or exhausted refinement retains the validated reference geometry.
    Every accepted shortcut passes BOTH the corridor/closure contract and the
    original swept-footprint checker. No path is fabricated on search failure.
    """
    compact = remove_collinear(path)
    topology = corridor.validate(compact)
    if not topology['valid']:
        raise ValueError('reference_corridor_invalid:'+topology.get('reason', 'unknown'))
    states = topology['vertex_states']; result = [compact[0]]; at = 0; attempts = 0
    while at < len(compact)-1:
        selected = at+1
        for candidate in range(len(compact)-1, at+1, -1):
            if attempts >= maximum_attempts:
                break
            attempts += 1
            allowed = corridor.segment(compact[at], compact[candidate], states[at], states[candidate])
            if allowed['valid'] and physical_check([compact[at], compact[candidate]])['valid']:
                selected = candidate
                break
        result.append(compact[selected]); at = selected
    check = corridor.validate(result)
    physical = physical_check(result)
    if not check['valid'] or not physical['valid'] or path_length(result) > path_length(path)+1e-7:
        # The caller will independently validate the reference too.
        return path, {'applied': False, 'reason': 'refinement_validation_failed',
                      'attempts': attempts, 'topology': topology}
    return result, {'applied': path_length(result) < path_length(path)-1e-7,
                    'method': 'topology_preserving_visibility_string_pulling',
                    'attempts': attempts, 'budget_exhausted': attempts >= maximum_attempts,
                    'reference_poses': len(path), 'refined_poses': len(result),
                    'topology': {k: v for k, v in check.items() if k != 'vertex_states'},
                    'scope': 'Shortening within the selected passage sequence; no global continuous-space optimality claim'}
