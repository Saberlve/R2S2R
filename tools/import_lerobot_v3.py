"""Read-only LeRobot v3 -> normalized episode NPZ + original video indices.
Run in an existing environment with pyarrow/numpy. Does not modify/copy videos.
"""
import argparse,pathlib,json,hashlib,numpy as np,pyarrow.parquet as pq
p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--out',required=True);p.add_argument('--dof',type=int,required=True);p.add_argument('--joint-unit',choices=['rad'],required=True);p.add_argument('--gripper-closed-one',action='store_true',required=True);a=p.parse_args();src=pathlib.Path(a.source).resolve();out=pathlib.Path(a.out).resolve()
if out.exists() or out.is_relative_to(src):raise ValueError('Output must be new and outside source')
info=json.loads((src/'meta/info.json').read_text())
if not str(info.get('codebase_version','')).startswith('v3'):raise ValueError('This importer supports LeRobot v3 only')
parts={};hashes={}
for f in sorted((src/'data').glob('*/*.parquet')):
 hashes[str(f.relative_to(src))]=hashlib.sha256(f.read_bytes()).hexdigest();t=pq.read_table(f).to_pydict()
 for i,ep in enumerate(t['episode_index']):
  state=np.asarray(t['observation.state'][i]);action=np.asarray(t['action'][i])
  if state.shape!=(a.dof+1,) or action.shape!=(a.dof+1,):raise ValueError('Expected DOF joints plus gripper, not existing EEF data')
  parts.setdefault(ep,[]).append((t['timestamp'][i],state,action,t['frame_index'][i],t['index'][i]))
if not parts:raise ValueError('No episodes')
refs={}
for f in sorted((src/'meta/episodes').glob('*/*.parquet')):
 for row in pq.read_table(f).to_pylist():refs[row['episode_index']]={k:v for k,v in row.items() if k.startswith('videos/') or k in ['episode_index','length','dataset_from_index','dataset_to_index']}
out.mkdir(parents=True)
for ep,rows in parts.items():
 times=np.array([x[0] for x in rows]);states=np.array([x[1] for x in rows]);actions=np.array([x[2] for x in rows])
 if not np.all(np.diff(times)>0) or not np.isfinite(states).all() or not np.isfinite(actions).all():raise ValueError('Invalid timestamps/values')
 if np.min(states[:,-1])<0 or np.max(states[:,-1])>1 or np.min(actions[:,-1])<0 or np.max(actions[:,-1])>1:raise ValueError('Invalid normalized gripper')
 np.savez_compressed(out/f'{ep:03d}.npz',time=times,q=states[:,:a.dof],action_q=actions[:,:a.dof],gripper=states[:,-1],action_gripper=actions[:,-1],frame_index=[x[3] for x in rows],global_index=[x[4] for x in rows])
manifest={'schema_version':'1.0','joint_unit':'rad','time_unit':'s','gripper_convention':'0=open,1=closed','dof':a.dof,'source':str(src),'source_parquet_sha256':hashes,'episode_ids':sorted(parts),'video_indices':refs,'gripper_observation_is_measured':'unknown; audit source acquisition code'}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2));print('Imported',len(parts),'episodes,',sum(map(len,parts.values())),'frames')
