import sys
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_hierarchy import HierarchyGraph,HierarchyUnavailable
from unittest.mock import patch


def fixture(alternate=False):
    names=['a1','a2','b1','b2','c1','c2','d1','d2']
    points={n:(i,0) for i,n in enumerate(names)}
    graph={n:{} for n in names}
    for a,b in zip(names,names[1:]):graph[a][b]=graph[b][a]=1.
    if alternate:graph['a2']['c2']=graph['c2']['a2']=7.
    owners={n:n[0] for n in names}
    parents={'a':'west','b':'west','c':'east','d':'east','west':'floor','east':'floor','floor':None}
    return HierarchyGraph(points,graph,owners,parents)


class HierarchyTests(unittest.TestCase):
    def test_lift_compact_and_expand_preserves_full_trace_and_cost(self):
        h=fixture();r=h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0))
        self.assertEqual(r['lca'],'floor');self.assertEqual(r['cost_m'],7.)
        self.assertEqual(r['nodes'],list(h.points));self.assertEqual(r['compact_search_nodes'],4)

    def test_closure_invalidates_cached_internal_trace(self):
        h=fixture(True);h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0))
        r=h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0),{'b2'})
        self.assertNotIn('b2',r['nodes']);self.assertEqual(r['cost_m'],10.)

    def test_disconnected_route_is_rejected_without_direct_fallback(self):
        h=fixture()
        with self.assertRaisesRegex(ValueError,'no_feasible_graph_route'):
            h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0),{'b2'})

    def test_query_virtual_nodes_do_not_modify_cached_base_graph(self):
        h=fixture();before={n:dict(v) for n,v in h.graph.items()}
        for _ in range(2):h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0))
        self.assertEqual(before,h.graph);self.assertNotIn('@start',h.points)

    def test_cycle_is_not_accepted_as_hierarchy(self):
        with self.assertRaisesRegex(ValueError,'cyclic_hierarchy'):
            HierarchyGraph({}, {}, {}, {'a':'b','b':'a'})

    def test_frontiers_lift_through_multiple_cached_parent_levels(self):
        h=fixture()
        h.parents.update({'west':'west_floor','east':'east_floor',
                          'west_floor':'building','east_floor':'building','floor':None,'building':None})
        h=HierarchyGraph(h.points,h.graph,h.owners,h.parents)
        r=h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0))
        self.assertEqual(r['cost_m'],7.)
        self.assertEqual(r['lift_levels']['@start'],['a','west','west_floor'])
        self.assertFalse(r['fallback_used'])
        self.assertEqual(r['nodes'],list(h.points))

    def test_global_graph_fallback_preserves_cost_and_closure(self):
        h=fixture(True)
        with patch.object(h,'_hierarchical_route',side_effect=HierarchyUnavailable('test unavailable summary')):
            r=h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0),{'b2'})
        self.assertTrue(r['fallback_used']);self.assertEqual(r['cost_m'],10.)
        self.assertNotIn('b2',r['nodes'])

    def test_global_fallback_never_invents_edge_for_disconnected_query(self):
        h=fixture()
        with patch.object(h,'_hierarchical_route',side_effect=HierarchyUnavailable('test unavailable summary')):
            with self.assertRaisesRegex(ValueError,'no_feasible_graph_route'):
                h.route('a','d',{'a1':0.},{'d2':0.},(0,0),(7,0),{'b2'})


if __name__=='__main__':unittest.main()
