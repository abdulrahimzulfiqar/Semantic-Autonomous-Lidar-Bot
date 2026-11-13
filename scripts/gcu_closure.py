"""Versioned logical closures and stopped-only replanning; no ROS dependency."""
import json
from osmag_mission_contract import MissionError,make_contract
from osmag_paper_planner import RouteError


def parse_closure(payload,current_revision,known):
    value=json.loads(payload)
    if not isinstance(value,dict) or set(value)!={'revision','disabled_passages'}:raise MissionError('invalid_closure_schema')
    revision=value['revision'];disabled=value['disabled_passages']
    if type(revision) is not int or revision<=current_revision:raise MissionError('obsolete_closure_revision')
    if not isinstance(disabled,list) or not disabled or any(not isinstance(p,str) for p in disabled) or len(set(disabled))!=len(disabled) or not set(disabled)<=set(known):raise MissionError('invalid_closed_passages')
    return {'revision':revision,'disabled_passages':sorted(disabled)}


def resolve_closure(planner,model,handles,pose,closure,now,pose_stamp):
    if not model.stop_confirmed or model._unresolved() or handles or model.state!='failed' or model.reason!='passage_closed':raise MissionError('closure_requires_verified_stop')
    old=model.contract
    try:
        revised=make_contract(planner,pose[:2],old.goal[:2],old.goal[2],mission_id=old.mission_id,now=now,pose_stamp=pose_stamp,
            route_revision=old.route_revision+1,closure_revision=closure['revision'],disabled_passages=closure['disabled_passages'])
    except RouteError as error:
        if str(error) not in ['no_feasible_graph_route','no_feasible_local_route']:raise
        return {'status':'no_route','reason':str(error),'closure_revision':closure['revision'],'disabled_passages':closure['disabled_passages'],'start':pose[:2],'goal':list(old.goal)}
    return {'status':'replan_requires_mask_validation','closure_revision':closure['revision'],
        'disabled_passages':closure['disabled_passages'],'contract':revised.report()}


def require_closure_contract(contract,closure):
    if contract.closure_revision!=closure['revision'] or set(contract.disabled_passages)!=set(closure['disabled_passages']):
        raise MissionError('obsolete_contract_closure_state')
