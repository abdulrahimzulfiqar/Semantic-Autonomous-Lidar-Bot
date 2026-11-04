#!/usr/bin/env python3
"""Verify the archived original Phase 04 matrix from hashes and recorded samples.

Run with the CAD pipeline Python environment (numpy, Pillow, PyYAML, OpenCV).
Recomputes localization, rest, sampled footprint and fixture connectivity checks;
updates independent_verification.json and source_verification.json. Preserves
failed/externally interrupted attempts and the open consecutive-heading issue.
"""
from pathlib import Path
import json,gzip,hashlib,sys,math
import numpy as np
from PIL import Image
import yaml,cv2
root=Path(__file__).resolve().parents[1];ev=root/'simulation/gcu_ground_floor/phase4_diagnostics/autonomous_acceptance'
sys.path.insert(0,str(root/'scripts'))
from gcu_navigation_metrics import rest_window,coverage
from gcu_plan_validation import validate_path
from compact_gcu_evidence import EvidenceFiles, verify_archive
files=EvidenceFiles(ev)
history=verify_archive(ev) if (ev/'development_history_manifest.json').exists() else None
x=json.loads((ev/'results.json').read_text());assert x['acceptance_passed'] and len(x['acceptance'])==16 and x['nominal_passes']==9
hashes=json.loads((ev/'frozen_inputs.json').read_text());verified=[]
for key,sha in hashes.items():
 p=root/'scripts'/key if '/' not in key and key.endswith('.py') else root/'simulation/gcu_ground_floor'/key.removeprefix('assets/')
 if p.is_file():
  assert hashlib.sha256(p.read_bytes()).hexdigest()==sha,(key,'source mismatch');verified.append(str(p.relative_to(root)))
urdf=root/'simulation/gcu_ground_floor/phase4_diagnostics/robot_collision_model.urdf'
assert hashlib.sha256(urdf.read_bytes()).hexdigest()==hashes['assets/contact_v2/robot.urdf']
(ev/'source_verification.json').write_text(json.dumps({'matched_repository_inputs':verified,'mismatches':[],'executed_urdf_matches_audit':True},indent=2)+'\n')
allrows=x['acceptance']+x['development']+x.get('interrupted_trials',[])
for suite in x['interrupted_suites'].values():allrows+=suite['trials']
checked=0
for row in allrows:
 for r in [row,*row.get('additional_reports',{}).values()]:
  if 'full_report' in r:
   data=files.read(r['full_report']);assert hashlib.sha256(data).hexdigest()==r['full_report_sha256'];json.loads(gzip.decompress(data));checked+=1
cfg=yaml.safe_load((root/'simulation/gcu_ground_floor/gcu_ground_floor.yaml').read_text());res=cfg['resolution'];origin=cfg['origin'][:2]
def grid(folder):
 c=yaml.safe_load((folder/'map.yaml').read_text()) if (folder/'map.yaml').exists() else cfg
 name=c['image'];im=np.flipud(np.asarray(Image.open(folder/name)))
 return np.where(im==254,0,np.where(im==0,100,-1))
basegrid=grid(root/'simulation/gcu_ground_floor');metrics=[]
for row in x['acceptance']:
 assert row['passed'] and row['runner']['passed'] and all(v==0 for v in row['runner']['launch_exits'])
 raw=json.loads(gzip.decompress((ev/row['full_report']).read_bytes()));assert len(raw['contact_publishers'])==13 and all(v==1 for v in raw['contact_publishers'].values())
 if raw['positive_control']:
  assert raw['contact_event_count']>0;continue
 assert raw['contact_event_count']==0 and not raw['contacts']
 assert raw['velocity_publishers']==[{'name':'controller_server','namespace':'/'}]
 pairs=raw['localization'];pe=[math.dist(v['truth'][:2],v['estimated'][:2]) for v in pairs]
 ye=[abs(math.degrees(math.atan2(math.sin(v['truth'][2]-v['estimated'][2]),math.cos(v['truth'][2]-v['estimated'][2])))) for v in pairs]
 assert abs(float(np.percentile(pe,95))-row['localization_p95_position_m'])<1e-9
 assert abs(float(np.percentile(ye,95))-row['localization_p95_heading_deg'])<1e-9
 cov=coverage(raw['truth'],pairs);assert cov==row['sampling_coverage'] and cov['matched_fraction']>=.9 and cov['max_truth_gap_sim_s']<=.15
 assert rest_window([v for v in raw['truth'] if v['sim_time']>=raw['rest_confirmed_sim']-1.3]) is not None
 g=grid(ev/'fixture_maps/detour') if row['name'].startswith('obstacle_detour') else grid(ev/'fixture_maps/blocked') if row['name'].startswith('blocked_passage') else basegrid
 swept=validate_path(g,res,origin,[v['truth'][:2] for v in raw['truth']]);assert swept['valid']
 if raw['expected']=='success':
  assert raw['action_status']==4 and math.dist(raw['final_truth'][:2],raw['goal'][:2])<=.2 and row['final_heading_error_deg']<=10
 elif raw['expected']=='cancelled':
  assert raw['action_status']==5 and raw['cancel_ack_count']>0
  delay=rest_window(raw['truth'],earliest=raw['cancel_request_sim'])['start_sim']-raw['cancel_request_sim'];assert abs(delay-row['cancel_stop_delay_sim_s'])<1e-9 and delay<=1
 else:
  assert not any(abs(v['v'])+abs(v['w'])>1e-5 for v in raw['commands'])
  if raw['via_click_bridge']:assert raw['action_status'] is None and raw['bridge_outcome']=='rejected' and not raw['display_paths'][-1]['points']
  else:assert raw['action_status']==6
 metrics.append(row['name'])
contract=json.loads((ev/'contract.json').read_text());fixture_results={}
for name,case in [('detour','obstacle_detour'),('blocked','blocked_passage')]:
 folder=ev/'fixture_maps'/name;g=grid(folder);p=folder/'map.pgm'
 assert hashlib.sha256(p.read_bytes()).hexdigest()==hashes['assets/navigation_fixtures/'+name+'/map.pgm']
 free=(g==0).astype(np.uint8);dist=cv2.distanceTransform(free,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*res
 mask=(dist>.18+res/math.sqrt(2)).astype(np.uint8);_,labels=cv2.connectedComponents(mask,connectivity=4)
 goal=next(c['goal'] for c in contract['cases'] if c['name']==case)
 def label(p):return int(labels[int((p[1]-origin[1])/res),int((p[0]-origin[0])/res)])
 a,b=label(contract['spawn']),label(goal);assert a>0 and b>0
 connected=a==b;assert connected==(name=='detour')
 fixture_results[name]={'spawn_label':a,'goal_label':b,'connected':connected,'conservative_radius_m':.18,'pgm_sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
review=json.loads((ev/'teardown_review.json').read_text())
for case in review['cases'].values():
 p=ev/case['full_log'];assert hashlib.sha256(p.read_bytes()).hexdigest()==case['full_log_sha256']
 lines=gzip.decompress(p.read_bytes()).decode().splitlines()
 assert 'SIGINT' in lines[case['signal_line']-1] and 'exit code -11' in lines[case['crash_line']-1] and case['signal_line']<case['crash_line']
result={'crash_free_shutdown':review['crash_free_shutdown'],'acceptance_cases_verified':16,'raw_report_hashes_verified':checked,'metric_and_sweep_cases_verified':metrics,'fixture_connectivity':fixture_results,'source_files_verified':len(verified),'qualified_scope':'original selected-route matrix; consecutive-goal heading and bridge shutdown issues remain open'}
(ev/'independent_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
files.close()
if history:print('Historical archive members verified:',len(history['members']))
