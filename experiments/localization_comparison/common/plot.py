#!/usr/bin/env python3
"""Evaluator-only scientific figure from frozen paired captures."""
import argparse
import io
from pathlib import Path
import json
import tarfile
import os
import tempfile
import numpy as np
os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'edgeosmag_mpl_cache'))
os.environ.setdefault('XDG_CACHE_HOME',str(Path(tempfile.gettempdir())/'edgeosmag_font_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    fig,axes=plt.subplots(2,2,figsize=(12,7));colors=['#222222','#2563eb','#d97706']
    for col,case in enumerate(['clean','clutter']):
        with tarfile.open(a.evidence/(case+'_tracking')/'attempt01.tar.gz') as t:
            prefix='results/paired_capture_'+case+'_01/'
            reference=np.load(io.BytesIO(t.extractfile(prefix+'evaluation.npz').read()))
            candidate=json.load(t.extractfile(prefix+'structural_estimate.json'))
            arrays=[reference['truth'],reference['amcl'],np.array([v['pose'] for v in candidate['poses']])]
            stamps=reference['stamps']-reference['stamps'][0]
            for data,color,label,style in zip(arrays,colors,['Truth','AMCL + EKF','Structural 2D + same odometry'],['-','--',':']):
                axes[0,col].plot(data[:,0],data[:,1],style,color=color,label=label,lw=1.5)
                if label!='Truth':axes[1,col].plot(stamps,100*np.linalg.norm(data[:,:2]-arrays[0][:,:2],axis=1),color=color,label=label,lw=1.2)
            axes[0,col].set(title=f'{case.title()} — {len(stamps)} matched scans',xlabel='Map x (m)',ylabel='Map y (m)',aspect='equal')
            axes[0,col].set_box_aspect(.5)
            axes[0,col].set_aspect('equal',adjustable='datalim')
            axes[1,col].set(xlabel='Elapsed simulated time (s)',ylabel='Absolute position error (cm)')
            for row in range(2):axes[row,col].grid(alpha=.2);axes[row,col].legend(fontsize=8)
    fig.suptitle('Preliminary GCU shared-input localization replay',fontsize=15)
    fig.text(.5,.015,'Fixed declared start; no truth-fitted alignment. Shadow replay on AMCL-controlled trajectories; no hardware or closed-loop candidate claim.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.04,1,.95]);fig.savefig(a.output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
