"""Closure revisions cannot reuse stale routes or replan with driving handles."""
import json,sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_closure import parse_closure,resolve_closure,require_closure_contract
from osmag_mission_contract import MissionError
from osmag_paper_planner import RouteError


class ClosureTest(unittest.TestCase):
    def test_valid_and_replayed_revision(self):
        raw=json.dumps({'revision':2,'disabled_passages':['p']})
        self.assertEqual(parse_closure(raw,1,{'p':1})['revision'],2)
        with self.assertRaisesRegex(MissionError,'obsolete'):parse_closure(raw,2,{'p':1})

    def test_unknown_and_duplicate_closures_rejected(self):
        for values in [['unknown'],['p','p']]:
            with self.assertRaisesRegex(MissionError,'invalid_closed'):parse_closure(json.dumps({'revision':1,'disabled_passages':values}),0,{'p':1})

    def test_old_contract_and_mismatched_disabled_set_block_dispatch(self):
        state={'revision':1,'disabled_passages':['p']}
        for c in [SimpleNamespace(closure_revision=0,disabled_passages=['p']),SimpleNamespace(closure_revision=1,disabled_passages=[])]:
            with self.assertRaisesRegex(MissionError,'obsolete_contract'):require_closure_contract(c,state)
        require_closure_contract(SimpleNamespace(closure_revision=1,disabled_passages=('p',)),state)

    def test_replan_requires_verified_rest_and_no_handles(self):
        m=SimpleNamespace(stop_confirmed=False,_unresolved=lambda:[],state='failed',reason='passage_closed')
        with self.assertRaisesRegex(MissionError,'verified_stop'):resolve_closure(None,m,{},[],{},1,1)
        m.stop_confirmed=True
        with self.assertRaisesRegex(MissionError,'verified_stop'):resolve_closure(None,m,{'old':'handle'},[],{},1,1)

    def test_only_graph_disconnect_is_explicit_no_route(self):
        old=SimpleNamespace(goal=(2.,0.,0.),mission_id='m',route_revision=1)
        m=SimpleNamespace(stop_confirmed=True,_unresolved=lambda:[],state='failed',reason='passage_closed',contract=old)
        closure={'revision':1,'disabled_passages':['p']}
        with patch('gcu_closure.make_contract',side_effect=RouteError('no_feasible_graph_route')):
            self.assertEqual(resolve_closure(None,m,{},[0.,0.,0.],closure,1,1)['status'],'no_route')
        with patch('gcu_closure.make_contract',side_effect=RouteError('occupied_start')):
            with self.assertRaises(RouteError):resolve_closure(None,m,{},[0.,0.,0.],closure,1,1)
