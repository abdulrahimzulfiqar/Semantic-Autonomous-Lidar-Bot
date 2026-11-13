#!/usr/bin/env python3
"""Independently replay two legs and verify continuous recorded session truth."""
import argparse,hashlib,io,json,math,tarfile
from pathlib import Path
import numpy as np
from osmag_paper_planner import PaperPlanner
from gcu_navigation_metrics import coverage,velocities
from verify_gcu_segmented_crossing import score_mission


def verify(folder,trial='01'):
    root=Path(__file__).resolve().parents[1];manifest=json.loads((folder/'manifest.json').read_text())
    for name,digest in manifest['artifacts_sha256'].items():assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest,name
    with tarfile.open(folder/('attempt'+trial+'.tar.gz')) as archive:
        prefix='results/consecutive_'+trial+'/'
        report=json.load(archive.extractfile(prefix+'report.json'));run=json.load(archive.extractfile(prefix+'run.json'))
        runtime=json.load(archive.extractfile('crossing_manifest.json'))['files_sha256']
        for name,digest in manifest['source_sha256'].items():
            member=Path(name).name
            assert hashlib.sha256(archive.extractfile(member).read()).hexdigest()==digest,name
            assert runtime[member]==digest,name
        result={'verification_passed':False,'trial_passed':bool(report['passed'] and run['passed'])}
        assert run['passed']==bool(report['passed'] and run['checker_exit']==0 and run['process_exits']==[0]*5 and not run['audit']['critical'] and run['truth_shutdown']['passed'] and run['simulator_shutdown']['passed'])
        if not result['trial_passed']:
            return dict(result,verification_passed=True,error=report.get('error'),scope='Failed consecutive attempt retained; not an accepted session')
        assert len(run['process_pids'])==len(set(run['process_pids']))==5
        assert len(report['missions'])==2 and all(report['lifecycle_shutdown'].values())
        planner=PaperPlanner(root/'simulation/gcu_ground_floor');scores=[]
        for index,mission in enumerate(report['missions'],1):
            arrays=np.load(io.BytesIO(archive.extractfile(prefix+f'constraint_{index}.npz').read()))
            scores.append(score_mission(mission,arrays,planner))
            sends=[e for e in mission['events'] if e['event']=='send']
            assert {int(e['token'].split(':')[0]) for e in sends}=={index}
            assert mission['state']['state']=='succeeded' and mission['state']['stop_confirmed'] and mission['state']['unresolved_requests']==0
            assert mission['snapshot']['mask_revision']==mission['contract']['route_revision']==index
        first,second=report['missions']
        assert first['contract']['mission_id']!=second['contract']['mission_id']
        assert second['reset_under_rest'] and second['dispatch_sim']>first['truth_rest']['confirmed_sim']
        assert math.dist(second['contract']['start'],first['truth'][-1]['truth'][:2])<.25
        assert second['truth'][0]['sim_time']>first['truth'][-1]['sim_time']
        # The full-session stream includes the intermission, not just two clips.
        truth=report['truth'];cov=coverage(truth,report['localization'])
        assert cov['max_truth_gap_sim_s']<=.15 and cov['matched_fraction']>=.9
        assert all(0<b['sim_time']-a['sim_time']<=.15 and math.dist(a['truth'][:2],b['truth'][:2])<=.25 for a,b in zip(truth,truth[1:]))
        assert max(v['v'] for v in velocities(truth))<=.20
        if report['fixture'].get('fuse_imu_yaw_rate'):
            imu=report['imu_samples'];times=np.asarray([v['sim_time'] for v in imu]);rates=np.asarray([v['yaw_rate'] for v in imu])
            assert len(imu)>200 and np.isfinite(rates).all() and all(v['frame']=='imu_link' for v in imu)
            assert np.all(np.diff(times)>0)
            active_gaps=[]
            for mission in report['missions']:
                begin=mission['dispatch_sim'];end=mission['truth_rest']['confirmed_sim']
                active=times[(times>=begin)&(times<=end)]
                assert len(active)>2 and active[0]-begin<=.15 and end-active[-1]<=.15
                maximum=float(np.max(np.diff(active)));assert maximum<=.15
                active_gaps.append(maximum)
            t0=max(truth[0]['sim_time'],times[0]);t1=min(truth[-1]['sim_time'],times[-1])
            selected=np.r_[t0,times[(times>t0)&(times<t1)],t1];w=np.interp(selected,times,rates)
            gyro_delta=float(np.sum((w[1:]+w[:-1])*.5*np.diff(selected)))
            truth_times=[v['sim_time'] for v in truth];yaw=np.unwrap([v['truth'][2] for v in truth])
            true_delta=float(np.interp(t1,truth_times,yaw)-np.interp(t0,truth_times,yaw))
            result['imu_evaluation']={'samples':len(imu),'integrated_yaw_rad':gyro_delta,'truth_delta_rad':true_delta,
                'integrated_error_deg':math.degrees(abs(gyro_delta-true_delta)),
                'active_window_maximum_gaps_s':active_gaps,'full_log_maximum_gap_s':float(np.max(np.diff(times))),
                'scope':'Evaluation only; truth never feeds EKF or mission execution'}
        assert not report['contacts'] and not json.loads((folder/'cleanup.json').read_text())['remaining_test_processes']
        simulator_log=archive.extractfile(prefix+'simulator.log').read().decode()
        assert simulator_log.count('process started with pid')>0
        result.update(verification_passed=True,missions=scores,continuous_session_coverage=cov,
            mission_generations=[1,2],native_processes=len(run['process_pids']),same_session=True)
        return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--trial',default='01');a=p.parse_args()
    result=verify(a.evidence,a.trial);(a.evidence/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
