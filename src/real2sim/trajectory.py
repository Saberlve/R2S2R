"""Hardware-independent FK using captured calibrated joint origins."""
import numpy as np
from scipy.spatial.transform import Rotation
from .contracts import rigid

def forward(q,calibration):
 q=np.asarray(q,float);origins=np.asarray(calibration['joint_origins_xyz_m_rpy_rad'],float)
 if origins.ndim!=2 or origins.shape[1]!=6 or q.shape!=(len(origins),) or not np.isfinite(origins).all() or not np.isfinite(q).all():raise ValueError('Joint dimensions/units invalid')
 axes=np.asarray(calibration['joint_axes'],float)
 if axes.shape!=(len(origins),3) or not np.isfinite(axes).all() or not np.allclose(np.linalg.norm(axes,axis=1),1):raise ValueError('Need explicit unit revolute joint axes')
 T=np.eye(4)
 for v,a,axis in zip(origins,q,axes):
  J=np.eye(4);J[:3,3]=v[:3];J[:3,:3]=Rotation.from_euler('xyz',v[3:]).as_matrix()@Rotation.from_rotvec(axis*a).as_matrix();T=T@J
 return T@rigid(calibration['T_flange_tcp'])

def convert_episode(data,calibration):
 required={'time','q','action_q','gripper','action_gripper'}
 if not required.issubset(data):raise ValueError('Missing episode arrays: '+str(required-set(data)))
 t=np.asarray(data['time']);n=len(t)
 if n<2 or not np.isfinite(t).all() or not np.all(np.diff(t)>0):raise ValueError('Time must increase within each episode')
 for k in required:
  if len(data[k])!=n or not np.isfinite(data[k]).all():raise ValueError('Invalid '+k)
 for k in ['gripper','action_gripper']:
  if np.asarray(data[k]).shape!=(n,) or np.min(data[k])<0 or np.max(data[k])>1:raise ValueError('Gripper must be N values in [0,1]')
 out=dict(data)
 for key,prefix in [('q',''),('action_q','action_')]:
  Ts=np.array([forward(q,calibration) for q in data[key]])
  out[prefix+'tcp_m']=Ts[:,:3,3];out[prefix+'tcp_quat_xyzw']=Rotation.from_matrix(Ts[:,:3,:3]).as_quat()
 return out
