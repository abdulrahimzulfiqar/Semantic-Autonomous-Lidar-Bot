"""Bounded fresh input acquisition before any mission/action has been issued."""
import time


def begin_when_fresh(adapter,contract,readiness,step,*,timeout_s=5.,wall=time.monotonic):
    from osmag_mission_contract import MissionError
    began=wall();attempts=0
    while True:
        attempts+=1
        try:
            adapter.begin(contract,readiness)
            return dict(attempts=attempts,waited_wall_s=wall()-began,
                scope='Fresh TF, scan and costmap acquisition before mission start; runtime guards unchanged')
        except MissionError as error:
            # begin() can also fail after model.start(). Retrying that case
            # could duplicate an action, so only a pristine idle model qualifies.
            model=adapter.model
            if (str(error) not in {'stale_robot_tf','stale_scan','constraint_costmaps_not_ready'}
                    or model.state!='idle' or model.contract is not None
                    or model.attempts or adapter.handles):raise
            if wall()-began>=timeout_s:raise
            step()
