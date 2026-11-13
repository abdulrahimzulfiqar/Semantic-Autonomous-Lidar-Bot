"""Independent cancellation evidence checks; no ROS or motion feedback."""
import math
from gcu_navigation_metrics import rest_window,velocities,coverage


def score_cancellation(report,*,reason='user_cancel',final_state='cancelled',band=(.30,.45),request_key='cancel_request'):
    def require(ok,reason):
        if not ok:raise ValueError(reason)
    request=report[request_key];events=report['events'];truth=report['truth']
    require(band[0]<request['segment_distance_m']<=band[1],'cancel_not_before_near_handoff')
    point=report['contract']['segments'][0]['point']
    require(abs(math.dist(request['pose'][:2],point)-request['segment_distance_m'])<1e-6,'cancel_trigger_geometry_changed')
    sends=[e for e in events if e['event']=='send']
    require(len(sends)==1 and sends[0]['wall']<request['wall'],'segment_advanced_after_cancel')
    accepted=[e for e in events if e['event']=='accepted' and e['accepted']]
    require(len(accepted)==1 and accepted[0]['token']==sends[0]['token'] and accepted[0]['wall']<request['wall'],'cancel_did_not_target_active_goal')
    ident=accepted[0]['uuid']
    faults=[e for e in events if e['event']=='fault']
    require(len(faults)==1 and faults[0]['reason']==reason and faults[0]['wall']>=request['wall'],'cancel_topic_not_received_or_other_fault')
    require(any(e['event']=='cancel_request' and e['uuid']==ident and e['wall']>=request['wall'] for e in events),'missing_native_cancel_request')
    results=[e for e in events if e['event']=='result' and e['uuid']==ident]
    require(len(results)==1 and results[0]['outcome']=='cancelled' and results[0]['wall']>=request['wall'],'native_cancel_not_confirmed')
    state=report['state']
    require(state['state']==final_state and state['stop_confirmed'] and state['unresolved_requests']==0,'executor_not_cancelled_and_settled')
    before=[v for v in truth if v['sim_time']<=request['sim']]
    require(before and math.dist(before[-1]['truth'][:2],truth[0]['truth'][:2])>=.25,'cancel_before_meaningful_motion')
    require(any(v['v']>.02 and request['sim']-.3<=v['sim_time']<=request['sim'] for v in velocities(truth)),'cancel_did_not_interrupt_motion')
    stopped=rest_window(truth,earliest=request['sim'])
    require(stopped is not None and 0<=stopped['start_sim']-request['sim']<=1.,'cancel_stop_exceeds_one_sim_second')
    require(truth[-1]['sim_time']>=stopped['confirmed_sim']+1.,'missing_post_cancel_hold')
    require(all(v['v']<=.01 and v['w']<=.02 for v in velocities(truth) if v['sim_time']>=stopped['confirmed_sim']),'motion_resumed_after_cancel')
    require(all(abs(v['v'])+abs(v['w'])<=1e-5 for v in report['commands'] if v['sim_time']>=stopped['confirmed_sim']),'nonzero_command_after_cancel_rest')
    cov=coverage(truth,report['localization'])
    require(cov['matched_fraction']>=.9 and cov['max_truth_gap_sim_s']<=.15,'cancel_truth_coverage_failed')
    return {'cancel_stop_delay_sim_s':stopped['start_sim']-request['sim'],'truth_rest':stopped,
        'post_rest_hold_sim_s':truth[-1]['sim_time']-stopped['confirmed_sim'],'coverage':cov,
        'native_cancel_confirmed':True,'no_later_segment':True}
