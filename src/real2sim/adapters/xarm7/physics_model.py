"""Calibrated xArm7 adapter; reuses existing Newton world and contact engines."""
import pathlib,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
import os
ROOT=pathlib.Path(os.environ['R2S_CASE_ROOT']).resolve()
REPO=pathlib.Path(os.environ['R2S_NEWTON_PROJECT']).resolve()
sys.path.insert(0,str(REPO))
from newton_gen.robot.xarm7 import XArm7,XArm7Info
from newton_gen.core.config import SimConfig
from newton_gen.scenes.spec import SceneSpec,ObjectSpec,Pose
from newton_gen.sim.world import build_world
import newton,warp as wp
def urdf_signature(file):
 import hashlib,xml.etree.ElementTree as ET
 file=pathlib.Path(file);h=hashlib.sha256(file.read_bytes())
 for filename in sorted({m.get('filename') for m in ET.parse(file).findall('.//mesh')}):
  p=pathlib.Path(filename);p=p if p.is_absolute() else file.parent/p;h.update(filename.encode());h.update(p.read_bytes())
 return h.hexdigest()

def pose(T):
 q=Rotation.from_matrix(np.asarray(T)[:3,:3]).as_quat()
 return Pose(position=tuple(np.asarray(T)[:3,3]),quat_wxyz=tuple(q[[3,0,1,2]]))
class CalibratedArm(XArm7):
 def urdf_path(self):return str(ROOT/'inputs/xarm7_calibrated.urdf')
 def _load_urdf(self,builder):
  cache=ROOT/'inputs/xarm7_collision_cached.urdf'
  stamp=ROOT/'inputs/collision_cache.sha256'
  import hashlib
  signature=urdf_signature(self.urdf_path())
  if cache.exists() and (not stamp.exists() or stamp.read_text().strip()!=signature):raise RuntimeError('Collision cache has no matching source hash. Rebuild in a new case directory.')
  self._using_collision_cache=cache.exists() and getattr(self,'_desired_engine','mujoco_fast')=='mujoco_fast'
  builder.add_urdf(str(cache if self._using_collision_cache else self.urdf_path()),xform=self._base_xform(),enable_self_collisions=False,parse_visuals_as_colliders=not self._using_collision_cache)
 def _setup_collision(self,builder,cfg):
  if self._using_collision_cache and cfg.engine=='mujoco_fast':
   self.observer_finger_meshes=None
   return
  super()._setup_collision(builder,cfg)
  if cfg.engine=='mujoco_fast' and not self._using_collision_cache:
   self._save_collision_cache(builder)
 def _write_joint_setup(self,builder):
  super()._write_joint_setup(builder)
  # Current Newton URDF importer creates mimic equality constraints; only the
  # leader is actuated. Six independent drives over-actuated the old adapter.
  if len(builder.constraint_mimic_joint0)!=5:raise RuntimeError('Expected five gripper mimic constraints')
  builder.joint_target_ke[7]=getattr(self,'gripper_kp_override',200.)
  builder.joint_target_kd[7]=getattr(self,'gripper_kd_override',5.)
  builder.joint_effort_limit[7]=getattr(self,'gripper_effort_override',1.)
  for i in range(8,13):
   builder.joint_target_ke[i]=0.;builder.joint_target_kd[i]=0.;builder.joint_effort_limit[i]=1.
 def _save_collision_cache(self,builder):
  import xml.etree.ElementTree as ET
  import struct
  tree=ET.parse(self.urdf_path());folder=ROOT/'inputs/colliders';folder.mkdir(exist_ok=True)
  for link in tree.findall('./link'):
   for c in list(link.findall('collision')):link.remove(c)
  for i,body in enumerate(builder.shape_body):
   if body<0 or not (int(builder.shape_flags[i]) & int(newton.ShapeFlags.COLLIDE_SHAPES)):continue
   if builder.shape_type[i] not in (newton.GeoType.MESH,newton.GeoType.CONVEX_MESH):raise RuntimeError('Unexpected robot primitive in collision cache')
   linkname=builder.body_label[body].split('/')[-1];link=tree.find("./link[@name='"+linkname+"']")
   mesh=builder.shape_source[i];verts=np.asarray(mesh.vertices)*np.asarray(builder.shape_scale[i])
   faces=np.asarray(mesh.indices).reshape(-1,3);tri=verts[faces]
   dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
   records=np.zeros(len(tri),dtype=dtype);records['vertices']=tri
   filename=folder/f'shape_{i:03d}.stl'
   with filename.open('wb') as f:f.write(b'EEF collision cache'.ljust(80,b' '));f.write(struct.pack('<I',len(tri)));f.write(records.tobytes())
   col=ET.SubElement(link,'collision');geo=ET.SubElement(col,'geometry');ET.SubElement(geo,'mesh',filename=str(filename))
   xf=builder.shape_transform[i];xyz=np.array(wp.transform_get_translation(xf));q=np.array(wp.transform_get_rotation(xf));rpy=Rotation.from_quat(q).as_euler('xyz')
   ET.SubElement(col,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
  tree.write(ROOT/'inputs/xarm7_collision_cached.urdf')
  import hashlib
  (ROOT/'inputs/collision_cache.sha256').write_text(urdf_signature(self.urdf_path()))
  print('Saved reusable convex collision cache',flush=True)
def build(engine='mujoco_fast',with_bar=True,episode=0,offset=(0,0,0),friction=None,size_scale=1.,arm_kp=None):
 s=json.loads((ROOT/'scene_config.json').read_text())
 T=np.array(s['T_sim_base']);p=pose(T)
 if size_scale<=0:raise ValueError('size_scale must be positive')
 oldh=s['bar']['size_m'][2];s['bar']['size_m']=(np.array(s['bar']['size_m'])*size_scale).tolist()
 s['bar']['position_m']=(np.array(s['bar']['position_m'])+np.array(s['table_matrix'])[:3,2]*(s['bar']['size_m'][2]-oldh)/2).tolist()
 data=np.load(ROOT/'inputs/episodes'/f'{episode:03d}.npz')
 # Object pose is explicit per case; never infer it from the trajectory being evaluated.
 s['bar']['position_m']=(np.array(s['bar']['position_m'])+np.asarray(offset)).tolist()
 if friction is not None:s['bar']['friction']=friction
 controller=s.setdefault('controller',{})
 if arm_kp is not None:controller['arm_kp']=arm_kp
 info=XArm7Info(home_q=tuple(data['q'][0]),arm_kp=controller.get('arm_kp',2500.),arm_kd=controller.get('arm_kd',80.))
 robot=CalibratedArm(base_position=p.position,base_quat_wxyz=p.quat_wxyz,info=info)
 robot.gripper_kp_override=controller.get('gripper_kp',200.)
 robot.gripper_kd_override=controller.get('gripper_kd',5.)
 robot.gripper_effort_override=controller.get('gripper_effort_limit_Nm',1.)
 robot._desired_engine=engine
 cfg=SimConfig(engine=engine,fps=30,sim_substeps=20,contact_surface_observer=False)
 table=ObjectSpec(id='table',size=tuple(s['table_size_m']),pose=pose(s['table_matrix']),dynamic=False)
 b=s['bar'];q=np.array(b['quaternion_xyzw']);bx,by,bz=b['size_m'];bm=b['mass_kg'];b['inertia_diagonal_kg_m2']=[bm*(by*by+bz*bz)/12,bm*(bx*bx+bz*bz)/12,bm*(bx*bx+by*by)/12]
 bar=ObjectSpec(id='bar',size=tuple(b['size_m']),pose=Pose(position=tuple(b['position_m']),quat_wxyz=tuple(q[[3,0,1,2]])),mass=b['mass_kg'])
 scene=SceneSpec(name='EEFAlignmentV1',table=table,objects=[bar] if with_bar else [],ground=False)
 if engine=='mujoco':
  import newton_gen.sim.world as world_module
  original=world_module.sdf_resolution_for_extent
  world_module.sdf_resolution_for_extent=lambda extent,cfg:8*((original(extent,cfg)+7)//8)
 try:world=build_world(scene,robot,cfg,load_visuals=False)
 finally:
  if engine=='mujoco':world_module.sdf_resolution_for_extent=original
 return world,s
if __name__=='__main__':
 wp.init()
 w,s=build(with_bar=False)
 m=w.model;state=m.state();newton.eval_fk(m,m.joint_q,m.joint_qd,state)
 print('BODIES',m.body_label);print('JOINTS',m.joint_label);print('Q',m.joint_q.numpy())
 print('TCP',w.tcp_body_index,state.body_q.numpy()[w.tcp_body_index])
 np.savez(ROOT/'reports/model_probe.npz',body_q=state.body_q.numpy(),q=m.joint_q.numpy())
 (ROOT/'reports/model_labels.json').write_text(json.dumps({'body':m.body_label,'joint':m.joint_label,'tcp_index':w.tcp_body_index,'ee_index':w.ee_link_index},indent=2))
