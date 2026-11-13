#!/usr/bin/env python3
"""Replay captured native-cell mismatches with stamped/latest TF and wire resolution.

Usage: python3 replay_gcu_costmap_timing.py ARCHIVE results/doorway_04
This diagnoses observed grids; it does not score a successful robot mission.
"""
import tarfile,json,io,math,numpy as np,sys
archive=sys.argv[1];prefix=sys.argv[2]
with tarfile.open(archive) as t:
 report=json.load(t.extractfile(prefix+'/report.json'));snap=report['snapshot'];out=[]
 for i,item in enumerate(report.get('native_grid_failures',[]),1):
  raw=np.load(io.BytesIO(t.extractfile(prefix+f'/native_grid_failure_{i}.npz').read()));data=raw['native'];mask=raw['mask'];rows,cols=np.mgrid[:data.shape[0],:data.shape[1]]
  gx=item['origin'][0]+(cols+.5)*item['resolution'];gy=item['origin'][1]+(rows+.5)*item['resolution']
  result={'capture':i,'source':item['source'],'stamp':item['stamp'],'observed_at':item['observed_at'],'transforms':{}}
  variants={'stamped':item['map_from_grid_at_stamp']}
  if item.get('map_from_grid_latest'):variants['latest']=item['map_from_grid_latest']['transform']
  for label,(tx,ty,yaw) in variants.items():
   x=tx+math.cos(yaw)*gx-math.sin(yaw)*gy;y=ty+math.sin(yaw)*gx+math.cos(yaw)*gy
   for encoding,resolution in [('original',snap['resolution']),('wire',float(np.float32(snap['resolution'])))]:
    r=np.floor((y-snap['origin'][1])/resolution).astype(int);c=np.floor((x-snap['origin'][0])/resolution).astype(int)
    inside=(r>=0)&(c>=0)&(r<mask.shape[0])&(c<mask.shape[1]);required=np.zeros_like(inside);required[inside]=mask[r[inside],c[inside]]==100;missing=required&(data>=0)&(data<100)
    rr,cc=np.where(missing);values,counts=np.unique(data[missing],return_counts=True)
    result['transforms'][label+'_'+encoding]={'required':int(required.sum()),'missing':int(missing.sum()),'native_values':dict(zip(values.tolist(),counts.tolist())),'cells':[{'native':[int(a),int(b)],'mask':[int(r[a,b]),int(c[a,b])],'value':int(data[a,b])} for a,b in zip(rr[:30],cc[:30])]}
  out.append(result)
 print(json.dumps({'report_passed':report['passed'],'error':report.get('error'),'captures':out},indent=2))
