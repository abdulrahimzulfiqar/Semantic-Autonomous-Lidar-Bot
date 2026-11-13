"""Matched fixed-mask comparator: one metric goal, no passage-goal dispatch."""
from dataclasses import replace,asdict
from osmag_mission_contract import sha


def metric_contract(contract):
    segments=(replace(contract.segments[-1],index=0),)
    digest=sha({'path':contract.path,'areas':contract.area_ids,'passages':contract.passage_ids,'segments':[asdict(s) for s in segments]})
    return replace(contract,segments=segments,policy='metric_final_goal',route_sha256=digest)
