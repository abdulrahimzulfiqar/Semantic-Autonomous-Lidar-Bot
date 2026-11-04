#!/usr/bin/env python3
"""Render measured acceptance paths/trajectories; run with the CAD Python venv."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',required=True,type=Path)
    p.add_argument('--assets',type=Path,default=Path('simulation/gcu_ground_floor'));p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    results=json.loads((a.evidence/'results.json').read_text())
    if not results['acceptance_passed']:raise ValueError('Acceptance did not pass; do not render a completed-checkpoint figure')
    cfg=yaml.safe_load((a.assets/'gcu_ground_floor.yaml').read_text());im=np.flipud(np.asarray(Image.open(a.assets/cfg['image'])))
    x,y=cfg['origin'][:2];r=cfg['resolution'];extent=(x,x+im.shape[1]*r,y,y+im.shape[0]*r)
    fig,axes=plt.subplots(1,4,figsize=(15,5),layout='constrained')
    for ax,(prefix,title) in zip(axes,[('corridor_','Corridor'),('turn_','Turn'),('room_entry_','Room entry'),('obstacle_detour_','LiDAR obstacle detour')]):
        selected=[v for v in results['acceptance'] if v['name'].startswith(prefix)]
        ax.imshow(im,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=254,interpolation='nearest')
        allpoints=[]
        for i,row in enumerate(selected):
            report=json.loads(gzip.decompress((a.evidence/row['full_report']).read_bytes()))
            truth=np.array([v['truth'][:2] for v in report['truth']]);allpoints.extend(truth.tolist())
            if i==0:
                plan=np.array(report['preflight']['path']);ax.plot(plan[:,0],plan[:,1],color='#1675c4',ls='--',lw=1,label='Initial metric plan')
            ax.plot(truth[:,0],truth[:,1],color=['#137b43','#df6b14','#8f44ad'][i%3],lw=1.3,label='Measured trial '+str(i+1))
            goal=report['goal'];ax.plot(goal[0],goal[1],'rx',ms=8,mew=1.7)
        if prefix=='obstacle_detour_':
            ax.add_patch(Rectangle((.95-.45/2,-12.960313240266368-.65/2),.45,.65,facecolor='#4f4f4f',edgecolor='red',label='Added obstacle'))
        pts=np.asarray(allpoints);pad=1.0;ax.set_xlim(pts[:,0].min()-pad,pts[:,0].max()+pad);ax.set_ylim(pts[:,1].min()-pad,pts[:,1].max()+pad)
        ax.set_aspect('equal');ax.set_title(title+f" ({len(selected)} trials)",fontsize=11);ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)')
        ax.legend(fontsize=6.5,loc='best')
    fig.suptitle('GCU autonomous metric navigation: planned paths and measured physical trajectories',fontsize=13)
    fig.savefig(a.output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
