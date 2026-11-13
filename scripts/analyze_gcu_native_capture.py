#!/usr/bin/env python3
"""Offline native-grid connectivity diagnostic; not a NavFn/footprint acceptance test."""
import argparse,io,json,math,tarfile
import cv2
import numpy as np


def connectivity(data,resolution,origin,start,goal):
    data=np.asarray(data);cells=[]
    for point in [start,goal]:
        col=math.floor((point[0]-origin[0])/resolution);row=math.floor((point[1]-origin[1])/resolution)
        if not (0<=row<data.shape[0] and 0<=col<data.shape[1]):return {'connected':False,'reason':'endpoint_outside_grid'}
        cells.append((row,col))
    start_cell,goal_cell=cells;free=(data<253).astype(np.uint8)
    # NavFn clears its robot start cell; surrounding blocked cells still matter.
    free[start_cell]=1;free[0,:]=free[-1,:]=free[:,0]=free[:,-1]=0
    _,labels=cv2.connectedComponents(free,connectivity=4)
    row,col=goal_cell
    return {'connected':bool(labels[start_cell] and labels[start_cell]==labels[goal_cell]),
        'start_cell':list(start_cell),'goal_cell':list(goal_cell),'start_cost':int(data[start_cell]),'goal_cost':int(data[goal_cell]),
        'goal_patch':data[max(0,row-3):row+4,max(0,col-3):col+4].tolist(),
        'scope':'Four-connected raw-cost graph with unknown disallowed; not an exact NavFn or swept-footprint replay'}


def analyze(archive_path,trial):
    prefix='results/consecutive_'+trial+'/'
    with tarfile.open(archive_path) as archive:
        session=json.load(archive.extractfile(prefix+'report.json'))
        reports=session.get('missions',[])+([session['failed_mission']] if 'failed_mission' in session else [])
        output=[]
        for report in reports:
            for capture in report.get('native_captures',[]):
                meta=capture['grids'].get('global_raw')
                if not meta or 'pose' not in capture:continue
                raw=np.load(io.BytesIO(archive.extractfile(prefix+meta['file']).read()))['data']
                result=connectivity(raw,meta['resolution'],meta['origin'],capture['pose'][:2],capture['segment'])
                output.append(dict(label=capture['label'],stamp=meta['stamp'],pose=capture['pose'],segment=capture['segment'],**result))
        return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('archive');p.add_argument('--trial',default='03');a=p.parse_args()
    print(json.dumps(analyze(a.archive,a.trial),indent=2))
