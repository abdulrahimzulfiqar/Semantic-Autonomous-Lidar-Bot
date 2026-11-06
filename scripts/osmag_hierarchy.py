"""Paper-based recursive compact/lift reference, not author runtime source.

Leaf graphs use validated base edges. Parent graphs summarize CHILD COMPACT
GRAPHS recursively, and query frontiers propagate with multi-source Dijkstra at
each level. Hierarchy failures fall back to the validated base passage graph.
"""
import heapq
import itertools
import math


def shortest(graph, source, allowed=None, target=None, heuristic=None):
    distances = {source: 0.}; previous = {}; counter = itertools.count()
    queue = [(0., next(counter), 0., source)]
    while queue:
        _, _, cost, node = heapq.heappop(queue)
        if cost != distances[node]:
            continue
        if node == target:
            break
        for neighbor, weight in graph.get(node, {}).items():
            if allowed is not None and neighbor not in allowed:
                continue
            candidate = cost + weight
            if candidate < distances.get(neighbor, math.inf) - 1e-12:
                distances[neighbor] = candidate; previous[neighbor] = node
                estimate = heuristic(neighbor) if heuristic else 0.
                heapq.heappush(queue, (candidate+estimate, next(counter), candidate, neighbor))
    return distances, previous


def trace(previous, source, target):
    result = [target]
    while result[-1] != source:
        if result[-1] not in previous:
            return []
        result.append(previous[result[-1]])
    return result[::-1]


def join(first, second):
    return first + (second[1:] if first and second and first[-1] == second[0] else second)


class HierarchyUnavailable(ValueError):
    """A compact query cannot supply a validated passage trace."""


class HierarchyGraph:
    def __init__(self, points, graph, owners, parents):
        self.points = points; self.graph = graph; self.owners = owners; self.parents = parents
        self.cache = {}; self.compact_cache = {}
        for area in parents:
            self.ancestors(area)
        # Normal-query summaries belong to preprocessing, not the first query.
        for area in sorted(parents, key=lambda a: len(self.ancestors(a)), reverse=True):
            self.compact(area, frozenset())

    def ancestors(self, area):
        result = []
        while area is not None:
            if area in result:
                raise ValueError('cyclic_hierarchy')
            result.append(area); area = self.parents.get(area)
        return result

    def lca(self, first, second):
        other = set(self.ancestors(second))
        return next((a for a in self.ancestors(first) if a in other), None)

    def nodes(self, area, disabled):
        return {n for n, owner in self.owners.items()
                if n not in disabled and area in self.ancestors(owner)}

    def boundary(self, area, disabled):
        inside = self.nodes(area, disabled)
        return sorted(n for n in inside if any(m not in inside and m not in disabled for m in self.graph[n]))

    def child(self, leaf, ancestor):
        chain = self.ancestors(leaf)
        return leaf if leaf == ancestor else chain[chain.index(ancestor)-1]

    @staticmethod
    def expand(walk, paths):
        if not walk:
            return []
        result = [walk[0]]
        for a, b in zip(walk, walk[1:]):
            result = join(result, paths[a, b])
        return result

    def within(self, area, source, disabled):
        key = (area, source, disabled)
        if key not in self.cache:
            graph, paths = self.compact(area, disabled)
            distances, previous = shortest(graph, source)
            expanded = {n: self.expand(trace(previous, source, n), paths) for n in distances}
            self.cache[key] = distances, expanded
        return self.cache[key]

    def compact(self, area, disabled):
        disabled = frozenset(disabled); key = (area, disabled)
        if key in self.compact_cache:
            return self.compact_cache[key]
        children = [a for a, parent in self.parents.items() if parent == area]
        graph = {}; paths = {}; inside = self.nodes(area, disabled)
        def edge(a, b, cost, path):
            if cost < graph.setdefault(a, {}).get(b, math.inf):
                graph[a][b] = cost; graph.setdefault(b, {})[a] = cost
                paths[a, b] = path; paths[b, a] = path[::-1]
        if not children:
            for a in inside:
                graph.setdefault(a, {})
                for b, weight in self.graph[a].items():
                    if b in inside:
                        edge(a, b, weight, [a, b])
        else:
            for child in children:
                self.compact(child, disabled)
                boundary = self.boundary(child, disabled)
                for i, a in enumerate(boundary):
                    graph.setdefault(a, {})
                    distances, expanded = self.within(child, a, disabled)
                    for b in boundary[i+1:]:
                        if b in distances:
                            edge(a, b, distances[b], expanded[b])
            for a in inside:
                for b, weight in self.graph[a].items():
                    if b in inside and self.child(self.owners[a], area) != self.child(self.owners[b], area):
                        edge(a, b, weight, [a, b])
        self.compact_cache[key] = graph, paths
        return graph, paths

    def lift(self, area, seeds, disabled):
        """Propagate all endpoint frontier costs through one cached level."""
        graph, paths = self.compact(area, disabled)
        counter = itertools.count(); queue = []; distances = {}; previous = {}; roots = {}
        for node, (cost, _) in seeds.items():
            if node in graph:
                distances[node] = cost; roots[node] = node
                heapq.heappush(queue, (cost, next(counter), node))
        while queue:
            cost, _, node = heapq.heappop(queue)
            if cost != distances[node]:
                continue
            for neighbor, weight in graph[node].items():
                candidate = cost+weight
                if candidate < distances.get(neighbor, math.inf)-1e-12:
                    distances[neighbor] = candidate; previous[neighbor] = node; roots[neighbor] = roots[node]
                    heapq.heappush(queue, (candidate, next(counter), neighbor))
        frontier = {}
        for node in self.boundary(area, disabled):
            if node in distances:
                root = roots[node]
                walk = trace(previous, root, node)
                frontier[node] = (distances[node], join(seeds[root][1], self.expand(walk, paths)))
        return frontier

    def _frontier(self, leaf, ancestor, attachments, disabled):
        seeds = {n: (cost, [n]) for n, cost in attachments.items() if n not in disabled}
        levels = []; area = leaf
        while area != ancestor:
            seeds = self.lift(area, seeds, disabled); levels.append(area)
            if not seeds:
                raise HierarchyUnavailable('empty_attach_or_lift_frontier')
            area = self.parents.get(area)
            if area is None:
                raise HierarchyUnavailable('missing_parent_lift')
        return seeds, levels

    def _hierarchical_route(self, first, last, start, goal, start_xy, goal_xy, disabled):
        ancestor = self.lca(first, last)
        if ancestor is None:
            raise HierarchyUnavailable('disconnected_hierarchy')
        original, stored = self.compact(ancestor, disabled)
        graph = {n: dict(edges) for n, edges in original.items()}; paths = dict(stored)
        levels = {}
        for name, area, attachments in [('@start', first, start), ('@goal', last, goal)]:
            frontier, levels[name] = self._frontier(area, ancestor, attachments, disabled)
            graph[name] = {}
            for node, (cost, path) in frontier.items():
                if node not in graph:
                    raise HierarchyUnavailable('frontier_absent_from_common_parent')
                graph[name][node] = cost; graph[node][name] = cost
                paths[name, node] = path; paths[node, name] = path[::-1]
        def heuristic(node):
            point = start_xy if node == '@start' else goal_xy if node == '@goal' else self.points[node]
            return math.dist(point, goal_xy)
        distances, previous = shortest(graph, '@start', target='@goal', heuristic=heuristic)
        if '@goal' not in distances:
            raise HierarchyUnavailable('no_compact_route')
        walk = trace(previous, '@start', '@goal'); expanded = []
        for a, b in zip(walk, walk[1:]):
            expanded = join(expanded, paths[a, b])
        if not expanded or any(n in disabled for n in expanded):
            raise HierarchyUnavailable('invalid_expanded_trace')
        return {'cost_m': distances['@goal'], 'nodes': expanded, 'lca': ancestor,
                'compact_search_nodes': len(graph), 'base_nodes': len(self.points),
                'lift_levels': levels, 'fallback_used': False,
                'method': 'paper_based_recursive_compact_multisource_lift_astar_expand'}

    def _flat_route(self, first, last, start, goal, start_xy, goal_xy, disabled, reason):
        # Only existing validated edges and raster-validated endpoint attachments.
        graph = {a: {b: w for b, w in edges.items() if b not in disabled}
                 for a, edges in self.graph.items() if a not in disabled}
        for name, attachments in [('@start', start), ('@goal', goal)]:
            graph[name] = {}
            for node, cost in attachments.items():
                if node in graph:
                    graph[name][node] = cost; graph[node][name] = cost
        def heuristic(node):
            return math.dist(start_xy if node == '@start' else goal_xy if node == '@goal' else self.points[node], goal_xy)
        distances, previous = shortest(graph, '@start', target='@goal', heuristic=heuristic)
        if '@goal' not in distances:
            raise ValueError('no_feasible_graph_route')
        return {'cost_m': distances['@goal'], 'nodes': trace(previous, '@start', '@goal')[1:-1],
                'lca': self.lca(first, last), 'compact_search_nodes': len(graph), 'base_nodes': len(self.points),
                'fallback_used': True, 'fallback_reason': reason,
                'method': 'validated_global_passage_astar_fallback'}

    def route(self, first, last, start, goal, start_xy, goal_xy, disabled=()):
        disabled = frozenset(disabled)
        try:
            return self._hierarchical_route(first, last, start, goal, start_xy, goal_xy, disabled)
        except (HierarchyUnavailable, KeyError) as error:
            return self._flat_route(first, last, start, goal, start_xy, goal_xy, disabled, str(error))
