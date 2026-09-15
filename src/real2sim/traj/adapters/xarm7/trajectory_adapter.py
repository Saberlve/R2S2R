"""Hardware-free coordinate conversion and Sim2Real trajectory export."""
import json,pathlib,numpy as np
from scipy.spatial.transform import Rotation
import os
ROOT=pathlib.Path(os.environ['R2S_CASE_ROOT']).resolve()
def transform(position,quat_xyzw):
 T=np.eye(4);T[:3,3]=position;T[:3,:3]=Rotation.from_quat(quat_xyzw).as_matrix();return T
def matrix_to_pose(T):
 return np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_quat()]
def sim_to_base(T_sim_tcp,T_sim_base):
 return np.linalg.solve(T_sim_base,T_sim_tcp)
def continuous_rotvec(matrices):
 out=[]
 for M in matrices:
  v=Rotation.from_matrix(M).as_rotvec();angle=np.linalg.norm(v)
  if out:
   if angle>1e-10:
    axis=v/angle;v=min((axis*(angle+2*np.pi*k) for k in range(-2,3)),key=lambda c:np.linalg.norm(c-out[-1]))
   elif np.linalg.norm(out[-1])>np.pi:
    axis=out[-1]/np.linalg.norm(out[-1]);v=axis*(2*np.pi*round(np.linalg.norm(out[-1])/(2*np.pi)))
  assert np.max(np.abs(Rotation.from_rotvec(v).as_matrix()-M))<1e-6
  out.append(v)
 return np.asarray(out)

def export_trajectory(path,time,target_q,actual_q,target_tcp_sim,actual_tcp_sim,gripper,T_sim_base,limits):
 time=np.asarray(time);target_q=np.asarray(target_q)
 assert len(time)>1 and np.all(np.diff(time)>0)
 vel=np.gradient(target_q,time,axis=0);acc=np.gradient(vel,time,axis=0)
 lo,hi=limits
 finite=bool(np.isfinite(target_q).all() and np.isfinite(target_tcp_sim).all())
 inlimits=bool(np.all(target_q>=lo) and np.all(target_q<=hi))
 target_base=np.array([sim_to_base(T,T_sim_base) for T in target_tcp_sim])
 actual_base=np.array([sim_to_base(T,T_sim_base) for T in actual_tcp_sim])
 rt=max(np.max(np.abs(T_sim_base@b-a)) for a,b in zip(target_tcp_sim,target_base))
 target_rotvec=continuous_rotvec(target_base[:,:3,:3]);actual_rotvec=continuous_rotvec(actual_base[:,:3,:3])
 # Joint targets preserve the chosen redundant configuration. TCP is for verification.
 rows=[]
 for i,t in enumerate(time):
  rows.append({'timestamp_s':float(t),'tcp_target_sim_m_quat_xyzw':matrix_to_pose(target_tcp_sim[i]).tolist(),'tcp_actual_sim_m_quat_xyzw':matrix_to_pose(actual_tcp_sim[i]).tolist(),'joint_target_rad':target_q[i].tolist(),'joint_actual_sim_rad':actual_q[i].tolist(),'tcp_target_base_mm_rotvec':np.r_[target_base[i,:3,3]*1000,target_rotvec[i]].tolist(),'tcp_actual_sim_base_mm_rotvec':np.r_[actual_base[i,:3,3]*1000,actual_rotvec[i]].tolist(),'gripper_closed_fraction':float(gripper[i]),'gripper_width_mm':float(84*(1-np.clip(gripper[i],0,1)))})
 report={'dry_run':True,'hardware_commands_sent':False,'frame':'robot_base','tcp_offset_mm':[0,0,172],'rotation':'axis_angle_radians','joint_units':'radians','finite':finite,'joint_limits_pass':inlimits,'max_joint_speed_rad_s':np.abs(vel).max(0).tolist(),'max_joint_acceleration_rad_s2':np.abs(acc).max(0).tolist(),'coordinate_roundtrip_max':float(rt),'collision_validation':'must reference corresponding simulation report','real_execution_validated':False,'speed_limit_rad_s':1.,'acceleration_limit_rad_s2':3.,'limit_source':'Conservative offline software gates; not measured hardware limits','speed_check_pass':bool(np.abs(vel).max()<=1.),'acceleration_check_pass':bool(np.abs(acc).max()<=3.),'execution_authorized':False}
 path=pathlib.Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 path.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
 path.with_suffix('.validation.json').write_text(json.dumps(report,indent=2))
 if not finite or not inlimits or rt>1e-9:raise RuntimeError(report)
 return report
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('trajectory');p.add_argument('--execute',action='store_true');a=p.parse_args()
 if a.execute:raise SystemExit('Hardware execution is intentionally unavailable in this offline deliverable.')
 rows=[json.loads(s) for s in pathlib.Path(a.trajectory).read_text().splitlines()]
 assert all(np.isfinite(r['joint_target_rad']).all() for r in rows)
 print(json.dumps({'dry_run':True,'rows':len(rows),'first_timestamp':rows[0]['timestamp_s'],'last_timestamp':rows[-1]['timestamp_s'],'hardware_connected':False}))
