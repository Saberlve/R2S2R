"""Canonical conventions: T_A_B maps column coordinates in B into A."""
import hashlib,json,pathlib,numpy as np

def load(path):
 return json.loads(pathlib.Path(path).read_text(encoding='utf-8-sig'),parse_constant=lambda x:(_ for _ in ()).throw(ValueError('Non-finite JSON: '+x)))
def save(path,value):
 p=pathlib.Path(path);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def sha(path):
 h=hashlib.sha256()
 with pathlib.Path(path).open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
def rigid(T,name='transform'):
 T=np.asarray(T,float)
 if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-7):raise ValueError(name+': invalid homogeneous matrix')
 if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-5):raise ValueError(name+': rotation contains scale/shear/reflection')
 return T
def pose(xyz,xyzw):
 from scipy.spatial.transform import Rotation
 q=np.asarray(xyzw,float)
 if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-4:raise ValueError('Expected unit xyzw quaternion')
 T=np.eye(4);T[:3,:3]=Rotation.from_quat(q).as_matrix();T[:3,3]=xyz;return rigid(T)
def pixel_transform(c):
 w,h=map(int,c['native_wh']);A=np.eye(3);op=c['pixel_ops']
 if 'crop_xywh' in op:
  x,y,cw,ch=op['crop_xywh']
  if any(v!=int(v) for v in [x,y,cw,ch]) or min(x,y)<0 or min(cw,ch)<=0 or x+cw>w or y+ch>h:raise ValueError('Invalid crop')
  A=np.array([[1,0,-x],[0,1,-y],[0,0,1]])@A;w,h=int(cw),int(ch)
 if 'resize_wh' in op:
  ow,oh=op['resize_wh']
  if min(ow,oh)<=0 or ow!=int(ow) or oh!=int(oh):raise ValueError('Invalid resize')
  sx,sy=ow/w,oh/h
  # Pixel-center convention consistent with OpenCV/Pillow resize.
  A=np.array([[sx,0,(sx-1)/2],[0,sy,(sy-1)/2],[0,0,1]])@A;w,h=int(ow),int(oh)
 if op.get('rotate_180'):A=np.array([[-1,0,w-1],[0,-1,h-1],[0,0,1]])@A
 return A,(w,h)
def profile_id(c):
 fields={k:c[k] for k in ['serial','native_wh','K_native','distortion','pixel_ops']}
 return hashlib.sha256(json.dumps(fields,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def validate_scene(s,check_schema=True):
 if check_schema:
  from jsonschema import Draft202012Validator
  schema=load(pathlib.Path(__file__).parent/'schemas/scene.schema.json');Draft202012Validator(schema).validate(s)
 if not np.isfinite(s['world_strength']) or s['world_strength']<0:raise ValueError('Invalid world strength')
 for key in ['assets','cameras','lights']:
  ids=[x['id'] for x in s[key]]
  if len(ids)!=len(set(ids)):raise ValueError('Duplicate '+key+' ids')
 all_ids=[x['id'] for group in ['assets','cameras','lights'] for x in s[group]]
 if len(all_ids)!=len(set(all_ids)):raise ValueError('Logical ids must be unique across assets, cameras and lights')
 for a in s['assets']:
  rigid(a['T_world_object'],a['id'])
  if 'size_m' in a and min(a['size_m'])<=0:raise ValueError('Non-positive object size')
  if 'color_linear' in a and not all(0<=v<=1 for v in a['color_linear']):raise ValueError('Albedo outside [0,1]')
  if not 0<=a.get('roughness',.5)<=1:raise ValueError('Invalid roughness')
 for light in s['lights']:
  if not np.isfinite(light['power_W']) or light['power_W']<0 or not np.isfinite(light['size_m']) or light['size_m']<=0:raise ValueError('Invalid light power/size')
  if 'target_m' in light and np.linalg.norm(np.array(light['target_m'])-light['position_m'])<1e-8:raise ValueError('Light target equals its position')
 for c in s['cameras']:
  rigid(c['T_world_optical'],c['id']);K=np.array(c['K_native']);w,h=c['native_wh']
  if min(w,h)<=0 or w!=int(w) or h!=int(h) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0 or not np.allclose(K[2],[0,0,1]) or abs(K[0,1])+abs(K[1,0])>1e-8:raise ValueError('Unsupported camera K/resolution')
  if c['distortion']['model']=='none' and any(c['distortion']['coefficients']):raise ValueError('Nonzero distortion labelled none')
  if c['distortion']['model']=='opencv_brown' and len(c['distortion']['coefficients']) not in [4,5,8]:raise ValueError('Brown model needs 4,5 or 8 coefficients')
  pixel_transform(c)
 return s
