import sys,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path.cwd()/'scripts'))
from osmag_paper_planner import PaperPlanner
from gcu_plan_validation import validate_path
folder=Path('simulation/gcu_ground_floor/phase6_diagnostics/06d_nominal_enclosed_room')
r=json.loads((folder/'nominal_01_report.json').read_text())
p=PaperPlanner(Path('simulation/gcu_ground_floor'))
old=p.plan(r['contract']['start'],r['contract']['goal'][:2])
new=np.asarray(r['contract']['path']);prior=np.asarray(old['path'])
truth=np.asarray([row['truth'][:2] for row in r['truth']])
with np.load(__import__('io').BytesIO(__import__('tarfile').open(folder/'attempt01.tar.gz').extractfile('results/nominal_01/constraint.npz').read())) as a:
 grid=a['physical'];mask=a['mask'];combined=a['combined']
assert not validate_path(combined,p.resolution,p.origin,old['path'],.20)['valid']
shade=np.ones(grid.shape);shade[(mask>0)&(grid==0)]=.65;shade[grid!=0]=.15
x,y=p.origin[:2];h,w=grid.shape;extent=[x,x+w*p.resolution,y,y+h*p.resolution]
fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
for ax in axes:
 ax.imshow(shade,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1,interpolation='nearest')
 ax.plot(prior[:,0],prior[:,1],'--',color='#c33136',lw=2,label='Physical-only reference (rejected by padded mask)')
 ax.plot(new[:,0],new[:,1],color='#2263ad',lw=2,label='Padded-mask reference')
 ax.plot(truth[:,0],truth[:,1],color='#16854b',lw=1.5,label='Executed robot position (evaluation only)')
 ax.set_aspect('equal');ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)')
axes[0].set_xlim(-1.2,4.7);axes[0].set_ylim(19.2,22.9);axes[0].set_title('Corridor → room: same two passages')
axes[1].set_xlim(2.6,4.4);axes[1].set_ylim(20.1,21.9);axes[1].set_title('Turn detail: gray cells are logical keepout')
handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',fontsize=8)
fig.savefig(folder/'execution_mask_route.png',dpi=160)
