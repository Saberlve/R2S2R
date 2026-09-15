import numpy as np
from ..contracts import pose,rigid

def snapshot_from_newton(model,state,bindings,frame_id,time_s,camera_bindings=None):
 if not isinstance(frame_id,(int,np.integer)) or frame_id<0 or not np.isfinite(time_s):raise ValueError("Invalid frame id/time")
 labels=list(model.body_label)
 if len(set(labels))!=len(labels):raise ValueError('Duplicate Newton body labels')
 q=state.body_q.numpy().copy();objects={}
 for name,b in bindings.items():
  if set(b)!={'body_label','T_body_object'}:raise ValueError('Binding needs exactly body_label and T_body_object')
  if b['body_label'] not in labels:raise ValueError('Missing Newton body '+b['body_label'])
  v=q[labels.index(b['body_label'])];T=pose(v[:3],v[3:7])@rigid(b['T_body_object']);objects[name]=T.tolist()
 cameras={}
 for name,b in (camera_bindings or {}).items():
  if set(b)!={'body_label','T_body_optical'} or b['body_label'] not in labels:raise ValueError('Invalid camera binding')
  v=q[labels.index(b['body_label'])];cameras[name]=(pose(v[:3],v[3:7])@rigid(b['T_body_optical'])).tolist()
 return {'T_world_optical_cameras':cameras,'schema_version':'1.0','frame_id':int(frame_id),'time_s':float(time_s),'T_world_objects':objects}
