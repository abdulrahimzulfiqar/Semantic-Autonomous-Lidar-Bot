import math
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_paper_planner import PaperPlanner,RouteError,length
from shapely.geometry import LineString


class GcuPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p=PaperPlanner(Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor')
        cls.start=(-.05864395131342248,-12.960313240266368)

    def test_coordinate_serialization_agrees_at_noncollinear_landmarks(self):
        r=self.p.coordinate_report();self.assertTrue(r['passed']);self.assertLess(r['maximum_error_m'],1e-5)

    def test_multisector_route_cost_matches_independent_sparse_dijkstra(self):
        goal=(-10.,-7.5);p=self.p;r=p.plan(self.start,goal)
        first=p.containing_area(self.start,'start');last=p.containing_area(goal,'goal')
        names=list(p.points)+['@start','@goal'];indices={n:i for i,n in enumerate(names)}
        matrix=np.zeros((len(names),len(names)))
        for a,edges in p.graph.items():
            for b,cost in edges.items():matrix[indices[a],indices[b]]=cost
        for virtual,area,point in [('@start',first,self.start),('@goal',last,goal)]:
            for n in p.residents[area]:
                path=p.grid_path(area,point,p.points[n])
                if path is not None:matrix[indices[virtual],indices[n]]=matrix[indices[n],indices[virtual]]=length(path)
        cost=dijkstra(csr_matrix(matrix),directed=False,indices=indices['@start'])[indices['@goal']]
        self.assertAlmostEqual(r['graph_cost_m'],cost,places=7)
        self.assertTrue(r['validation']['valid']);self.assertGreaterEqual(len(r['passage_ids']),2)

    def test_goal_rejection_and_no_synthetic_start(self):
        for goal,reason in [((0,0),'unknown_goal'),((200,200),'outside_goal'),((math.nan,0),'nonfinite_goal')]:
            with self.subTest(goal=goal),self.assertRaisesRegex(RouteError,reason):self.p.plan(self.start,goal)
        with self.assertRaisesRegex(RouteError,'outside_start'):self.p.plan((200,200),(-10,-7.5))

    def test_disabling_all_passages_prevents_cross_sector_route(self):
        with self.assertRaisesRegex(RouteError,'no_feasible_graph_route'):
            self.p.plan(self.start,(-10,-7.5),self.p.passages)

    def test_same_sector_uses_no_unnecessary_passage(self):
        route=self.p.plan(self.start,(-3.,-10.5));self.assertEqual(route['passage_ids'],[])
        self.assertTrue(route['validation']['valid'])

    def test_open_space_queries_refine_to_clear_direct_lower_bound(self):
        for goal in [(1.4413560486865775,-12.960313240266368),(5.,-8.),(.5,-16.),(-3.,-10.5)]:
            with self.subTest(goal=goal):
                r=self.p.plan(self.start,goal)
                self.assertAlmostEqual(r['refined_length_m'],math.dist(self.start,goal),places=7)
                self.assertAlmostEqual(length(r['graph_path']),r['graph_cost_m'],places=7)
                self.assertLessEqual(r['refined_length_m'],r['graph_cost_m']+1e-7)
                self.assertEqual(r['refinement']['topology']['passage_ids'],r['passage_ids'])

    def test_closure_survives_refinement_although_pgm_direct_line_is_clear(self):
        goal=(1.4413560486865775,-12.960313240266368)
        r=self.p.plan(self.start,goal,['-93'])
        self.assertNotIn('-93',r['passage_ids']);self.assertGreater(r['refined_length_m'],1.5)
        barrier=LineString(self.p.passage_segments['-93'])
        self.assertTrue(all(not LineString([a,b]).intersects(barrier) for a,b in zip(r['path'],r['path'][1:])))
        self.assertEqual(r['refinement']['topology']['passage_ids'],r['passage_ids'])

    def test_duplicate_interface_pair_filter_is_applied(self):
        self.assertEqual(len(self.p.redundant_interface_pairs),16)
        for row in self.p.redundant_interface_pairs:
            a,b=row['states'];self.assertNotIn(b,self.p.graph[a])


if __name__=='__main__':unittest.main()
