"""Newton dynamics replay. Targets never overwrite simulated state after initialization."""
import argparse,json,numpy as np,warp as wp,newton
from scipy.spatial.transform import Rotation
from physics_model import ROOT,build
from trajectory_adapter import transform,export_trajectory
from newton_gen.sim.engine.registry import get_engine_class
p=argparse.ArgumentParser();p.add_argument('--mode',choices=['joint','eef'],default='eef');p.add_argument('--bar',action='store_true');p.add_argument('--frames',type=int,default=330);p.add_argument('--engine',default='mujoco_fast');p.add_argument('--tag',default='');p.add_argument('--episode',type=int,default=0);p.add_argument('--trajectory');p.add_argument('--offset',type=float,nargs=3,default=[0,0,0]);p.add_argument('--friction',type=float);p.add_argument('--size-scale',type=float,default=1.);p.add_argument('--arm-kp',type=float);a=p.parse_args()
wp.init();w,settings=build(a.engine,a.bar,a.episode,a.offset,a.friction,a.size_scale,a.arm_kp);m=w.model;cfg=w.cfg
data=np.load(ROOT/'inputs/episodes'/f'{a.episode:03d}.npz');ik=np.load(ROOT/'results'/f'episode{a.episode:03d}_ik.npz')
if a.trajectory:data=np.load(a.trajectory);ik=data
qtargets=data['q'] if a.mode=='joint' else ik['q']
gripper_commands=data['action_gripper'] if 'action_gripper' in data.files else data['gripper']
gmap=json.loads((ROOT/'inputs/gripper_mapping.json').read_text())
def grip(f):return np.interp(.084*(1-np.clip(f,0,1)),gmap['gap_m'][::-1],gmap['drive_rad'][::-1])
mu=m.shape_material_mu.numpy()
if a.bar:mu[w.object_shape_indices('bar')]=settings['bar']['friction']
m.shape_material_mu.assign(mu)
engine=get_engine_class(a.engine)()
if a.engine=='mujoco':
 original_solver=newton.solvers.SolverMuJoCo
 class CompatibleSolver(original_solver):
  def __init__(self,*args,**kw):
   if kw.pop('strong_friction_shapes',None) is not None:raise RuntimeError('Unsupported tactile strong friction request')
   kw.pop('strong_friction_stiffness_scale',None)
   super().__init__(*args,**kw)
 newton.solvers.SolverMuJoCo=CompatibleSolver
 try:engine.build(w,cfg)
 finally:newton.solvers.SolverMuJoCo=original_solver
else:engine.build(w,cfg)
s0=m.state();s1=m.state();control=m.control()
initial=m.joint_q.numpy();initial[:7]=qtargets[0];initial[7:13]=grip(data['gripper'][0])
s0.joint_q.assign(initial);newton.eval_fk(m,s0.joint_q,s0.joint_qd,s0)
target=control.joint_target_q.numpy();target[:13]=initial[:13];control.joint_target_q.assign(target)
# Let drive settle at first pose; bar remains fully dynamic.
for _ in range(30):s0,s1=engine.simulate_substeps(s0,s1,control,cfg.sim_dt)
graph=None
try:
 with wp.ScopedCapture(device=m.device) as cap:
  end0,end1=engine.simulate_substeps(s0,s1,control,cfg.sim_dt)
 assert end0 is s0
 graph=cap.graph
except Exception as ex:print('Graph unavailable',str(ex),flush=True)
flange=next(i for i,n in enumerate(m.body_label) if n.endswith('/link7'))
B=np.array(settings['T_sim_base'])
out=ROOT/'results'/f"{a.mode}_{a.engine}_{'bar' if a.bar else 'free'}";out=out.with_name(out.name+('_'+a.tag if a.tag else ''));out.mkdir(exist_ok=True);(out/'scene_config.json').write_text(json.dumps(settings,indent=2))
rows=[];actual=[];targets=[];bodies=[];obj=[];errs=[];tlist=[];glist=[];qout=[];contact_log=[]
n=min(a.frames,len(qtargets))
for i in range(n):
 target[:7]=qtargets[i];target[7:13]=grip(gripper_commands[i]);control.joint_target_q.assign(target)
 if graph is not None:wp.capture_launch(graph)
 else:s0,s1=engine.simulate_substeps(s0,s1,control,cfg.sim_dt)
 q=s0.joint_q.numpy();v=s0.body_q.numpy();T=transform(v[flange,:3],v[flange,3:]);T[:3,3]+=T[:3,:3]@np.array([0,0,.172])
 goal=B@transform(data['tcp_m'][i],data['tcp_quat_xyzw'][i])
 err=[np.linalg.norm(T[:3,3]-goal[:3,3])*1000,Rotation.from_matrix(T[:3,:3].T@goal[:3,:3]).magnitude()*180/np.pi]
 actual.append(T);targets.append(goal);bodies.append(v);qout.append(q);errs.append(err);tlist.append(i/30);glist.append(float(gripper_commands[i]))
 if a.bar:
  obj.append(v[w.object_body_index('bar')])
  if engine.contacts is not None:
   engine.solver.update_contacts(engine.contacts,s0)
   c=engine.contacts;ncontact=int(c.rigid_contact_count.numpy()[0])
   c0=c.rigid_contact_shape0.numpy()[:ncontact];c1=c.rigid_contact_shape1.numpy()[:ncontact]
   barshapes=w.object_shape_indices('bar');mask=np.isin(c0,barshapes)|np.isin(c1,barshapes)
   force=c.force.numpy()[:ncontact]
   distances=engine.solver.mjw_data.contact.dist.numpy().reshape(-1)[:ncontact]
   contact_log.append({'frame':i,'bar_contact_count':int(mask.sum()),'sum_contact_force_magnitudes_N':float(np.linalg.norm(force[mask],axis=-1).sum()) if mask.any() else 0.,'max_bar_penetration_m':float(max(0,-distances[mask].min())) if mask.any() else 0.})
 if i%60==0:print('FRAME',i,'error',err,flush=True)
assert np.isfinite(qout).all()
np.savez_compressed(out/'states.npz',time=tlist,body_labels=np.asarray(list(m.body_label)),length_unit='m',quaternion_order='xyzw',joint_q=qout,body_q=bodies,tcp_actual_sim=actual,tcp_target_sim=targets,gripper=glist,bar=obj)
report={'mode':a.mode,'engine':a.engine,'cuda_graph':graph is not None,'episode':a.episode,'frames':n,'p95_position_mm':float(np.percentile(errs,95,axis=0)[0]),'p95_rotation_deg':float(np.percentile(errs,95,axis=0)[1]),'max_error':np.max(errs,axis=0).tolist(),'bar_enabled':a.bar,'no_object_attachment':True,'estimated_object':True}
if a.bar:
 obj=np.array(obj);normal=np.array(settings['table_matrix'])[:3,2];height=(obj[:,:3]-obj[0,:3])@normal
 report['max_bar_lift_m']=float(height.max());report['held_5cm_for_2s']=any(np.all(height[i:i+60]>=.05) for i in range(max(0,n-59)))
if contact_log:
 (out/'contacts.json').write_text(json.dumps(contact_log,indent=2))
 report['max_bar_penetration_m']=max(x['max_bar_penetration_m'] for x in contact_log)
 report['max_bar_contact_force_sum_N']=max(x['sum_contact_force_magnitudes_N'] for x in contact_log)
 report['contact_force_note']='sum of contact force magnitudes, not net force or SDK force percent'
export_trajectory(out/'trajectory_dry_run.jsonl',tlist,qtargets[:n],np.array(qout)[:,:7],np.array(targets),np.array(actual),glist,B,(m.joint_limit_lower.numpy()[:7],m.joint_limit_upper.numpy()[:7]))
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
