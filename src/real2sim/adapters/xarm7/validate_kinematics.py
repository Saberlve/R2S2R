"""FK gate and Newton IK trajectory solve using calibrated URDF; no hardware."""
import argparse,json,sys,numpy as np
from physics_model import ROOT,CalibratedArm,XArm7Info,pose
from trajectory_adapter import transform
import newton,newton.ik as ik,warp as wp
from scipy.spatial.transform import Rotation
args=argparse.ArgumentParser();args.add_argument('--episode',type=int,default=0);args.add_argument('--skip-fk',action='store_true');args=args.parse_args()
wp.init();newton.use_coord_layout_targets=True
s=json.loads((ROOT/'scene_config.json').read_text());B=np.array(s['T_sim_base']);p=pose(B)
data=np.load(ROOT/'inputs/episodes'/f'{args.episode:03d}.npz')
robot=CalibratedArm(base_position=p.position,base_quat_wxyz=p.quat_wxyz,info=XArm7Info(home_q=tuple(data['q'][0])))
b=newton.ModelBuilder();robot._load_urdf(b);robot._write_joint_setup(b);m=b.finalize();st=m.state()
labels=list(m.body_label);flange=next(i for i,n in enumerate(labels) if n.endswith('/link7'));tcp=next(i for i,n in enumerate(labels) if n.endswith('/link_tcp'))
qfull=m.joint_q.numpy().copy()
def Tof(q):
 m.joint_q.assign(q);newton.eval_fk(m,m.joint_q,m.joint_qd,st)
 v=st.body_q.numpy()[flange];T=transform(v[:3],v[3:]);T[:3,3]+=T[:3,:3]@np.array([0,0,.172]);return T
if not args.skip_fk:
 errors=[]
 for file in sorted((ROOT/'inputs/episodes').glob('*.npz')):
  d=np.load(file);ee=[]
  for j,q in enumerate(d['q']):
   qfull[:7]=q;T=Tof(qfull);target=B@transform(d['tcp_m'][j],d['tcp_quat_xyzw'][j])
   ee.append([np.linalg.norm(T[:3,3]-target[:3,3])*1000,Rotation.from_matrix(T[:3,:3].T@target[:3,:3]).magnitude()*180/np.pi])
  errors.append({'episode':int(file.stem),'frames':len(ee),'max_position_mm':float(np.max(ee,axis=0)[0]),'max_rotation_deg':float(np.max(ee,axis=0)[1])})
 report={'episodes':errors,'max_position_mm':max(x['max_position_mm'] for x in errors),'max_rotation_deg':max(x['max_rotation_deg'] for x in errors)}
 (ROOT/'reports/newton_fk.json').write_text(json.dumps(report,indent=2));print('FK',report['max_position_mm'],report['max_rotation_deg'],flush=True)
 assert report['max_position_mm']<1 and report['max_rotation_deg']<.1
posobj=ik.IKObjectivePosition(link_index=flange,link_offset=wp.vec3(0,0,.172),target_positions=wp.array([wp.vec3()],dtype=wp.vec3))
rotobj=ik.IKObjectiveRotation(link_index=flange,link_offset_rotation=wp.quat_identity(),target_rotations=wp.array([wp.vec4(0,0,0,1)],dtype=wp.vec4))
limit=ik.IKObjectiveJointLimit(joint_limit_lower=m.joint_limit_lower,joint_limit_upper=m.joint_limit_upper)
solver=ik.IKSolver(model=m,n_problems=1,objectives=[posobj,rotobj,limit],lambda_initial=.01,jacobian_mode=ik.IKJacobianType.ANALYTIC)
seed=m.joint_q.numpy().copy();seed[:7]=data['q'][0];qik=wp.array(seed.reshape(1,-1),dtype=wp.float32)
targets=[];solutions=[];metrics=[]
for i in range(len(data['q'])):
 T=B@transform(data['tcp_m'][i],data['tcp_quat_xyzw'][i]);quat=Rotation.from_matrix(T[:3,:3]).as_quat()
 posobj.target_positions.assign(np.array([T[:3,3]],np.float32));rotobj.target_rotations.assign(np.array([quat],np.float32))
 solver.step(qik,qik,iterations=40)
 q=qik.numpy()[0];actual=Tof(q);solutions.append(q[:7].copy());targets.append(T)
 metrics.append([np.linalg.norm(actual[:3,3]-T[:3,3])*1000,Rotation.from_matrix(actual[:3,:3].T@T[:3,:3]).magnitude()*180/np.pi])
np.savez_compressed(ROOT/'results'/f'episode{args.episode:03d}_ik.npz',time=data['time'],q=np.array(solutions),tcp_target_sim=np.array(targets),gripper=data['gripper'])
(ROOT/'reports'/f'ik_{args.episode:03d}.json').write_text(json.dumps({'max_position_mm':float(np.max(metrics,0)[0]),'max_rotation_deg':float(np.max(metrics,0)[1]),'p95':np.percentile(metrics,95,axis=0).tolist(),'max_joint_step_rad':np.max(np.abs(np.diff(solutions,axis=0)),axis=0).tolist(),'redundant_q_rmse_rad':np.sqrt(np.mean((np.array(solutions)-data['q'])**2,axis=0)).tolist()},indent=2))
print('IK',np.max(metrics,axis=0),flush=True)


assert np.max(metrics,0)[0]<=1. and np.max(metrics,0)[1]<=.1, 'IK accuracy gate failed'
