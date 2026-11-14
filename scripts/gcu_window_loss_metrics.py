"""Independent truth/action scoring for a deliberately frozen raster source."""
import math
from gcu_navigation_metrics import rest_window,velocities,coverage


def score_window_loss(report):
    request=report['window_loss_request'];events=report['events'];truth=report['truth']
    def require(value,reason):
        if not value:raise ValueError(reason)
    faults=[e for e in events if e['event']=='fault']
    require(len(faults)==1 and faults[0]['reason']=='watchdog:constraint_costmaps_not_ready','wrong_window_fault')
    require(.45<=faults[0]['sim']-request['sim']<=.8,'source_deadline_not_observed')
    require(not any(e['event']=='send' and e['wall']>request['wall'] for e in events),'dispatch_after_source_freeze')
    require(any(e['event']=='cancel_request' and e['wall']>=faults[0]['wall'] for e in events),'missing_native_cancel')
    require(any(e['event']=='result' and e['outcome']=='cancelled' and e['wall']>=faults[0]['wall'] for e in events),'native_cancel_not_confirmed')
    state=report['state'];require(state['state']=='failed' and state['stop_confirmed'] and not state['unresolved_requests'],'executor_not_settled')
    before=[p for p in truth if p['sim_time']<=request['sim']]
    require(before and math.dist(before[-1]['truth'][:2],truth[0]['truth'][:2])>=.4,'no_meaningful_motion')
    require(any(v['v']>.02 and request['sim']-.3<=v['sim_time']<=request['sim'] for v in velocities(truth)),'source_loss_while_not_moving')
    stopped=rest_window(truth,earliest=request['sim'])
    require(stopped is not None and 0<=stopped['start_sim']-request['sim']<=1.,'window_loss_stop_over_one_second')
    require(truth[-1]['sim_time']>=stopped['confirmed_sim']+1.,'missing_stop_hold')
    require(all(abs(v['v'])+abs(v['w'])<=1e-5 for v in report['commands'] if v['sim_time']>=stopped['confirmed_sim']),'command_after_rest')
    cov=coverage(truth,report['localization']);require(cov['matched_fraction']>=.9 and cov['max_truth_gap_sim_s']<=.15,'truth_coverage_failed')
    return dict(passed=True,stop_delay_sim_s=stopped['start_sim']-request['sim'],rest=stopped,coverage=cov)
