"""Convert an offline JSONL TCP trajectory into continuous Newton IK targets.
Rows: timestamp_s, tcp_base_mm_rotvec [6] OR tcp_sim_m_quat_xyzw [7],
gripper_closed_fraction, optional joint_reference_rad [7]. No hardware imports.
"""
import argparse,json,numpy as np,warp as wp,newton,newton.ik as ik
from scipy.spatial.transform import Rotation,Slerp
from physics_model import ROOT,CalibratedArm,XArm7Info,pose
from trajectory_adapter import transform
p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('output');a=p.parse_args()
rows=[json.loads(x) for x in open(a.input) if x.strip()];times=np.array([x['timestamp_s'] for x in rows]);assert len(times)>1 and np.all(np.diff(times)>0)
s=json.loads((ROOT/'scene_config.json').read_text());B=np.array(s['T_sim_base']);Ts=[]
for row in rows:
 if 'tcp_sim_m_quat_xyzw' in row:
  v=np.array(row['tcp_sim_m_quat_xyzw']);T=transform(v[:3],v[3:])
 else:
  v=np.array(row['tcp_base_mm_rotvec']);T=B@transform(v[:3]*.001,Rotation.from_rotvec(v[3:]).as_quat())
 assert np.isfinite(T).all();Ts.append(T)
Ts=np.array(Ts);t=np.arange(int(np.floor((times[-1]-times[0])*30))+1)/30+times[0]
xyz=np.column_stack([np.interp(t,times,Ts[:,j,3]) for j in range(3)]);rots=Slerp(times,Rotation.from_matrix(Ts[:,:3,:3]))(t)
g=np.interp(t,times,[r['gripper_closed_fraction'] for r in rows]);assert np.all((g>=0)&(g<=1))
wp.init();newton.use_coord_layout_targets=True;base=pose(B)
q0=rows[0].get('joint_reference_rad',np.load(ROOT/'inputs/episodes/000.npz')['q'][0].tolist())
robot=CalibratedArm(base_position=base.position,base_quat_wxyz=base.quat_wxyz,info=XArm7Info(home_q=tuple(q0)));builder=newton.ModelBuilder();robot._load_urdf(builder);robot._write_joint_setup(builder);m=builder.finalize();flange=next(i for i,n in enumerate(m.body_label) if n.endswith('/link7'))
po=ik.IKObjectivePosition(link_index=flange,link_offset=wp.vec3(0,0,.172),target_positions=wp.array([wp.vec3()],dtype=wp.vec3));ro=ik.IKObjectiveRotation(link_index=flange,link_offset_rotation=wp.quat_identity(),target_rotations=wp.array([wp.vec4(0,0,0,1)],dtype=wp.vec4));li=ik.IKObjectiveJointLimit(joint_limit_lower=m.joint_limit_lower,joint_limit_upper=m.joint_limit_upper)
solver=ik.IKSolver(model=m,n_problems=1,objectives=[po,ro,li],lambda_initial=.01,jacobian_mode=ik.IKJacobianType.ANALYTIC);q=wp.array(m.joint_q.numpy().reshape(1,-1),dtype=wp.float32);qs=[];errs=[];state=m.state();outTs=[]
for pos,rot in zip(xyz,rots):
 po.target_positions.assign(np.array([pos],np.float32));ro.target_rotations.assign(np.array([rot.as_quat()],np.float32));solver.step(q,q,iterations=40);v=q.numpy()[0];qs.append(v[:7].copy());state.joint_q.assign(v);newton.eval_fk(m,state.joint_q,state.joint_qd,state);f=state.body_q.numpy()[flange];actual=transform(f[:3],f[3:]);actual[:3,3]+=actual[:3,:3]@np.array([0,0,.172]);target=transform(pos,rot.as_quat());outTs.append(np.linalg.solve(B,target));errs.append([np.linalg.norm(pos-actual[:3,3])*1000,Rotation.from_matrix(rot.as_matrix().T@actual[:3,:3]).magnitude()*180/np.pi])
errs=np.array(errs);assert errs[:,0].max()<=1 and errs[:,1].max()<=.1,errs.max(0)
outTs=np.array(outTs);np.savez_compressed(a.output,time=t-t[0],q=qs,gripper=g,tcp_m=outTs[:,:3,3],tcp_quat_xyzw=Rotation.from_matrix(outTs[:,:3,:3]).as_quat())
print(json.dumps({'frames':len(t),'max_fk_error_mm_deg':errs.max(0).tolist(),'fps':30,'no_hardware':True}))
