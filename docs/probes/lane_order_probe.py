"""Session 18 probe (no ROS, no language model): does flying the clue's lanes
first help (D50 -> D53), and is D47's 8 m clearance enough (D54)?

  PYTHONPATH=src/uav_mapping:src/uav_planning:src/uav_perception:src/uav_eval \
      python3 docs/probes/lane_order_probe.py        (about 5 min, 5 processes)

The "model" is a stand-in with gemini-3.8-flash's habits in the pilot (15 m
circles, 18 m on polygons, 12 m capsules along lines), so these are design
numbers, not results. Seeds 2000+, never the eval's. The clearance line prints
what scenario.CLEAR gives now; with 8 m it was 37 of 400 runs.
"""
import json, numpy as np
from concurrent.futures import ProcessPoolExecutor
from uav_eval import run as cli; from uav_eval.core import arms, scenario as sc, sim
from uav_mapping.core.landmarks import wrong_landmarks
from uav_mapping.core.prior import parse_regions, rasterize
w,g,p=cli.load('worlds','valley')
def answer(clue, world):     # stand-in: the model's pilot habits: 15 m circles, capsules along lines
    R=[]
    for lm in w.landmarks:
        if f'near the {lm.name}' in clue:
            if lm.type=='polyline':
                R+=[dict(shape='capsule',start=list(a),end=list(b),radius=12,weight=1.0,confidence=.95) for a,b in zip(lm.points,lm.points[1:])]
            else: R.append(dict(shape='circle',center=list(map(float,np.mean(lm.points,axis=0))),radius=18 if lm.type=='polygon' else 15,weight=1.0,confidence=.95))
    return json.dumps({'regions':R})
def job(a):
    arm,seed,on=a; return arms.run_arm(arm, sc.draw(w,g,seed), w,g,answer, sim.SimParams(prior_lanes=on))
if __name__=='__main__':
    bad=tot=badr=0
    for s in range(2000,2400):
        x=sc.draw(w,g,s); u=rasterize(g,parse_regions(answer(x.misleading,w)))
        z=[u[max(r-1,0):r+2,max(c-1,0):c+2].max()>0 for c,r in [g.cell_of(*t) for t in x.targets]]
        bad+=sum(z); tot+=len(z); badr+=any(z)
    print('clearance', sc.CLEAR, ': targets whose zone a wrong region touches',bad,'/',tot,'; runs',badr,'/400')
    seeds=range(2000,2100)
    with ProcessPoolExecutor(5) as ex:
        res={}
        for on in (True,False):
            for arm in ('uniform','accurate','misleading'):
                res[arm,on]=list(ex.map(job,[(arm,s,on) for s in seeds]))
    for (arm,on),rs in res.items():
        s=arms.summary(rs); print(arm,'prior lanes' if on else 'default lanes',s['first'],s['all'],'missed',s['targets_missed'],'by leader',s['found_by_leader'],'/',s['targets'])
    for arm in ('accurate','misleading'):
        for what in ('first','all'):
            print(arm,what,'prior vs default lanes',arms.paired({'on':res[arm,True],'off':res[arm,False]},'on','off',what))
