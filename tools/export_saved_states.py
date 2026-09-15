"""Saved Newton state arrays -> canonical rendering JSONL; no hardware or stepping."""
import argparse,pathlib,json,numpy as np
from real2sim.traj.bridge import snapshot_from_newton
from real2sim.contracts import load
p=argparse.ArgumentParser();p.add_argument('--states',required=True);p.add_argument('--bindings',required=True);p.add_argument('--out',required=True);p.add_argument('--stride',type=int,default=1);a=p.parse_args()
d=np.load(a.states,allow_pickle=False);b=load(a.bindings);out=pathlib.Path(a.out)
if out.exists():raise FileExistsError(out)
if a.stride<1 or str(d['length_unit'])!='m' or str(d['quaternion_order'])!='xyzw':raise ValueError('Invalid sampling or units')
if set(b)!={'schema_version','objects','cameras'} or b['schema_version']!='1.0':raise ValueError('Invalid binding manifest')
if d['body_q'].shape!=(len(d['time']),len(d['body_labels']),7) or not np.all(np.diff(d['time'])>0):raise ValueError('Invalid saved Newton states')
class Model:body_label=d['body_labels'].tolist()
class Array:
 def __init__(self,i):self.i=i
 def numpy(self):return d['body_q'][self.i]
class State:
 def __init__(self,i):self.body_q=Array(i)
out.parent.mkdir(parents=True,exist_ok=True)
with out.open('x',encoding='utf-8') as f:
 for i in range(0,len(d['time']),a.stride):f.write(json.dumps(snapshot_from_newton(Model(),State(i),b['objects'],i,float(d['time'][i]),b['cameras']),allow_nan=False)+'\n')
print('Exported',len(range(0,len(d['time']),a.stride)),'snapshots')
