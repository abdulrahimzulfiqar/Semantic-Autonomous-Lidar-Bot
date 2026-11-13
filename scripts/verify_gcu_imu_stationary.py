#!/usr/bin/env python3
"""Replay the no-motion IMU availability proof from captured native artifacts."""
import argparse,ast,hashlib,json,math,tarfile
import xml.etree.ElementTree as ET
from pathlib import Path


def verify(folder):
    manifest=json.loads((folder/'manifest.json').read_text())
    for name,digest in manifest['artifacts_sha256'].items():assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest,name
    with tarfile.open(folder/'stationary06.tar.gz') as archive:
        report=json.load(archive.extractfile('results/stationary_06/report.json'));run=json.load(archive.extractfile('results/stationary_06/run.json'))
        runtime=json.load(archive.extractfile('crossing_manifest.json'))['files_sha256']
        for name,digest in manifest['source_sha256'].items():
            member=Path(name).name
            assert hashlib.sha256(archive.extractfile(member).read()).hexdigest()==runtime[member]==digest
        assert report['passed'] and report['diagnostic_completed'] and run['passed']
        assert run['process_exits']==[0]*5 and not run['audit']['critical'] and run['simulator_shutdown']['passed']
        assert not report['events'] and not any(abs(v['v'])+abs(v['w'])>1e-8 for v in report['commands'])
        assert report['stationary']['displacement_m']<.01 and report['stationary']['mask_ready_at_end']
        samples=report['imu_diagnostic']['samples'];assert report['imu_diagnostic']['publishers']==1 and report['imu_diagnostic']['fresh']
        gaps=[b['sim_time']-a['sim_time'] for a,b in zip(samples,samples[1:])];duration=samples[-1]['sim_time']-samples[0]['sim_time'];hz=(len(samples)-1)/duration
        assert duration>=5 and 80<=hz<=120 and all(0<d<=.05 for d in gaps)
        maximum=max(abs(v['yaw_rate']) for v in samples)
        assert maximum<.02 and all(math.isfinite(v['yaw_rate']) and v['frame']=='imu_link' for v in samples)
        world=ET.fromstring(archive.extractfile('assets/contact_v2/world.sdf').read())
        assert len([p for p in world.findall('.//plugin') if p.get('name')=='ignition::gazebo::systems::Imu'])==1
        tree=ast.parse(archive.extractfile('gcu_amcl.launch.py').read());params=None
        for call in ast.walk(tree):
            if not isinstance(call,ast.Call):continue
            fields={v.arg:v.value for v in call.keywords}
            if 'name' in fields and isinstance(fields['name'],ast.Constant) and fields['name'].value=='gcu_alignment_ekf':
                params={}
                for item in fields['parameters'].elts:params.update(ast.literal_eval(item))
        assert params['imu0']=='/imu_broadcaster/imu' and params['imu0_config']==[False]*11+[True]+[False]*3
        assert not params['odom0_config'][11] and params['odom0_config'][6:8]==[True,True]
        return {'verification_passed':True,'scope':'Stationary IMU delivery/fusion configuration only; no navigation claim',
            'samples':len(samples),'rate_hz':hz,'duration_sim_s':duration,'maximum_abs_yaw_rate':maximum,'nonzero_commands':0}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args();r=verify(a.evidence)
    (a.evidence/'stationary_verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
