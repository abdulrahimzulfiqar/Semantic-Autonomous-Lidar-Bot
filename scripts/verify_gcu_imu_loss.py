#!/usr/bin/env python3
"""Independently score loss of the executor's required IMU stream."""
import argparse,json
from pathlib import Path
from gcu_cancel_metrics import score_cancellation
from verify_gcu_cancel import score_cancel_run
from verify_gcu_segmented_crossing import verify


def score_imu_loss(report,arrays,planner):
    metrics=score_cancel_run(report,arrays,planner,lambda r:score_cancellation(r,reason='watchdog:stale_or_unowned_imu',final_state='failed',band=(.60,.75),request_key='imu_loss_request'))
    rows=report['imu_forwarding'];accepted=report['required_imu_samples'];request=report['imu_loss_request']
    assert report['raw_imu_publishers']==report['required_imu_publishers']==1
    assert sum(v['forwarded'] for v in rows)>=100 and sum(not v['forwarded'] for v in rows)>=100
    assert all(not v['forwarded'] for v in rows if v['received_sim']>request['sim']+.05)
    forwarded={v['stamp'] for v in rows if v['forwarded']}
    assert accepted and all(v['sim_time'] in forwarded and v['frame']=='imu_link' for v in accepted)
    fault=next(e for e in report['events'] if e['event']=='fault')
    assert fault['sim']-accepted[-1]['sim_time']>.15-1e-9
    assert max(v['received_sim'] for v in rows if not v['forwarded'])>=metrics['truth_rest']['confirmed_sim']+1.
    metrics['imu_loss_stop_delay_sim_s']=metrics.pop('cancel_stop_delay_sim_s')
    return dict(metrics,raw_imu_continued=True,executor_input_stale_verified=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--trial',default='01');a=p.parse_args()
    result=verify(a.evidence,a.trial,'imu_loss',mission_scorer=score_imu_loss)
    (a.evidence/'historical_verification_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
