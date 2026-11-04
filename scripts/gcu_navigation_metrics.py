"""Independent sampled-truth rest and localization-coverage metrics."""
import math


def velocities(samples,baseline=.09):
    result=[]
    for i,item in enumerate(samples):
        earlier=next((old for old in reversed(samples[:i]) if item['sim_time']-old['sim_time']>=baseline),None)
        if earlier is None:continue
        dt=item['sim_time']-earlier['sim_time']
        a=item['truth'];b=earlier['truth'];angle=math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2]))
        result.append({'sim_time':item['sim_time'],'v':math.hypot(a[0]-b[0],a[1]-b[1])/dt,'w':abs(angle)/dt})
    return result


def rest_window(samples,earliest=None,duration=1.,max_gap=.15):
    """First continuously slow >=1s window, with bounded observation gaps.

    The returned time is the first slow velocity interval, measured over >=90ms.
    Both translation and rotation must remain below the declared limits.
    """
    start=None;previous=None
    for item in velocities(samples):
        t=item['sim_time']
        if earliest is not None and t<earliest:continue
        if item['v']>.01 or item['w']>.02 or (previous is not None and t-previous>max_gap):start=None
        else:
            if start is None:start=t
            if t-start>=duration:return {'start_sim':start,'confirmed_sim':t}
        previous=t
    return None


def coverage(truth,matched):
    gaps=[b['sim_time']-a['sim_time'] for a,b in zip(truth,truth[1:])]
    return {'truth_samples':len(truth),'matched_samples':len(matched),
        'matched_fraction':len(matched)/len(truth) if truth else 0.,'max_truth_gap_sim_s':max(gaps,default=math.inf)}
