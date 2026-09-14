"""Generate a new approach-close-lift-hold trajectory for the estimated rigid bar."""
import json,numpy as np,warp as wp,newton,newton.ik as ik
from scipy.spatial.transform import Rotation,Slerp
from physics_model import ROOT,CalibratedArm,XArm7Info,pose
from trajectory_adapter import transform
import argparse
p=argparse.ArgumentParser();p.add_argument('--offset',type=float,nargs=3,default=[0,0,0]);p.add_argument('--output',default='generated_grasp.npz');options=p.parse_args()
wp.init();newton.use_coord_layout_targets=True
s=json.loads((ROOT/'scene_config.json').read_text());B=np.array(s['T_sim_base']);base=pose(B)
d=np.load(ROOT/'inputs/episodes/000.npz')
robot=CalibratedArm(base_position=base.position,base_quat_wxyz=base.quat_wxyz,info=XArm7Info(home_q=tuple(d['q'][0])))
b=newton.ModelBuilder();robot._load_urdf(b);robot._write_joint_setup(b);m=b.finalize()
flange=next(i for i,n in enumerate(m.body_label) if n.endswith('/link7'))
start=B@transform(d['tcp_m'][0],d['tcp_quat_xyzw'][0])
obj=np.array(s['bar']['position_m'])+np.array(options.offset);table=np.array(s['table_matrix']);normal=table[:3,2]
objR=Rotation.from_quat(s['bar']['quaternion_xyzw']).as_matrix()
# Tool +Z points down; tool X follows long bar axis. Pinch direction is tool Y.
x=objR[:,0];z=-normal;y=np.cross(z,x);R=np.column_stack([x,y,z])
grasp=obj-normal*(s['bar']['size_m'][2]/2-.003)
approach=grasp+normal*.12;lift=grasp+normal*.12
knots=[(0,start[:3,3],start[:3,:3],0),(3,approach,R,0),(5,grasp,R,0),(6.5,grasp,R,1),(9,lift,R,1),(11,lift,R,1)]
times=np.arange(331)/30;Ts=[];grips=[]
for t in times:
 j=min(np.searchsorted([k[0] for k in knots],t,side='right')-1,len(knots)-2);j=max(0,j)
 a,c=knots[j:j+2];u=np.clip((t-a[0])/(c[0]-a[0]),0,1);u=u*u*u*(10-15*u+6*u*u)
 T=np.eye(4);T[:3,3]=(1-u)*a[1]+u*c[1];T[:3,:3]=Slerp([0,1],Rotation.from_matrix([a[2],c[2]]))([u]).as_matrix()[0]
 Ts.append(T);grips.append((1-u)*a[3]+u*c[3])
po=ik.IKObjectivePosition(link_index=flange,link_offset=wp.vec3(0,0,.172),target_positions=wp.array([wp.vec3()],dtype=wp.vec3))
ro=ik.IKObjectiveRotation(link_index=flange,link_offset_rotation=wp.quat_identity(),target_rotations=wp.array([wp.vec4(0,0,0,1)],dtype=wp.vec4))
li=ik.IKObjectiveJointLimit(joint_limit_lower=m.joint_limit_lower,joint_limit_upper=m.joint_limit_upper)
solver=ik.IKSolver(model=m,n_problems=1,objectives=[po,ro,li],lambda_initial=.01,jacobian_mode=ik.IKJacobianType.ANALYTIC)
qik=wp.array(m.joint_q.numpy().reshape(1,-1),dtype=wp.float32);qs=[]
for T in Ts:
 po.target_positions.assign(np.array([T[:3,3]],np.float32));ro.target_rotations.assign(np.array([Rotation.from_matrix(T[:3,:3]).as_quat()],np.float32))
 solver.step(qik,qik,iterations=40);qs.append(qik.numpy()[0,:7].copy())
Tb=np.array([np.linalg.solve(B,T) for T in Ts])
np.savez_compressed(ROOT/'results'/options.output,time=times,q=qs,gripper=grips,tcp_m=Tb[:,:3,3],tcp_quat_xyzw=Rotation.from_matrix(Tb[:,:3,:3]).as_quat())
print('Generated',len(qs),'new waypoint frames; max step rad',np.max(np.abs(np.diff(qs,axis=0))),flush=True)
