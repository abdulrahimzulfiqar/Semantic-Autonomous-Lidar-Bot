"""Reproduce the closure geometry and independently measured velocity figure."""
import io,json,sys,tarfile
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(root/'scripts'))
from gcu_navigation_metrics import velocities
from osmag_paper_planner import PaperPlanner
p=Path(__file__).parent
with tarfile.open(p/'attempt01.tar.gz') as t:
 r=json.loads(t.extractfile('results/closure_01/report.json').read())
 a=np.load(io.BytesIO(t.extractfile('results/closure_01/closure_constraint.npz').read()))
 grid=a['physical'];mask=a['mask']
planner=PaperPlanner(root/'simulation/gcu_ground_floor')
shade=np.ones(grid.shape);shade[(mask>0)&(grid==0)]=.72;shade[grid!=0]=.12
x,y=planner.origin[:2];h,w=grid.shape
fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
ax=axes[0];ax.imshow(shade,origin='lower',extent=[x,x+w*planner.resolution,y,y+h*planner.resolution],cmap='gray',vmin=0,vmax=1,interpolation='nearest')
reference=np.array(r['contract']['path']);truth=np.array([v['truth'][:2] for v in r['truth']]);portal=np.array(planner.passage_segments['-95'])
ax.plot(reference[:,0],reference[:,1],'--',color='#2768b5',label='Reference before closure')
ax.plot(truth[:,0],truth[:,1],color='#16854b',lw=2,label='Measured robot position')
ax.plot(portal[:,0],portal[:,1],color='#c33136',lw=4,label='Logically closed passage -95')
ax.plot(*truth[-1],marker='x',ms=9,color='black',label='Robot stopped')
ax.set(xlim=(2.5,5.25),ylim=(19.2,24.),xlabel='Map x (m)',ylabel='Map y (m)',title='Closed room exit; gray is logical keepout')
ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper right')
ax=axes[1];v=velocities(r['truth']);ax.plot([a['sim_time'] for a in v],[a['v'] for a in v],color='#2768b5',label='Sampled translation speed')
request=r['closure_request']['sim'];onset=r['cancellation']['truth_rest']['start_sim']
ax.axvline(request,color='#c33136',ls='--',label='Closure request');ax.axvline(onset,color='#16854b',ls='--',label='Verified rest onset')
ax.axhline(.01,color='gray',ls=':',label='Translation rest threshold')
ax.set(xlim=(request-1.5,r['truth'][-1]['sim_time']+.1),ylim=(-.005,.115),xlabel='Simulation time (s)',ylabel='Translation speed (m/s)',title='Rest begins 0.297 simulated seconds after request')
ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.savefig(p/'closure_stop.png',dpi=150)
