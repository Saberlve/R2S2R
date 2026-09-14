"""Run with the existing bpy Python runtime or through Blender MCP.
Creates a new scene on build; never resets/deletes the user's other scenes.
"""
import sys,pathlib,argparse,json,hashlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]))
import numpy as np,bpy,bmesh
from mathutils import Matrix,Vector
from PIL import Image
from real2sim.contracts import load,save,validate_scene,rigid,pixel_transform,profile_id,sha
COLS={'table':'COL_Table','robot':'COL_Robot','task':'COL_TaskObjects','misc':'COL_Misc','background':'COL_Background'}
def parent_depth(obj):
 n=0
 while obj is not None and obj.parent is not None:n+=1;obj=obj.parent
 return n

def lookup(scene,name):
 found=[o for o in scene.objects if o.get('r2s_id')==name]
 if len(found)>1:raise ValueError('Duplicate logical id '+name)
 return found[0] if found else scene.objects.get(name)
def apply_camera(scene,obj,c):
 w,h=map(int,c['native_wh']);K=np.array(c['K_native']);fx,fy=K[0,0],K[1,1];cx,cy=K[0,2],K[1,2]
 scene.render.resolution_x=w;scene.render.resolution_y=h;scene.render.resolution_percentage=100
 scene.render.pixel_aspect_x=1.;scene.render.pixel_aspect_y=fx/fy
 obj.data.type='PERSP';obj.data.sensor_fit='HORIZONTAL';obj.data.sensor_width=36;obj.data.lens=36*fx/w
 obj.data.shift_x=(w/2-(cx+.5))/w;obj.data.shift_y=((cy+.5)-h/2)*(fx/fy)/w
 obj.matrix_world=Matrix((np.array(c['T_world_optical'])@np.diag([1,-1,-1,1])).tolist());scene.camera=obj

def audit(scene,s):
 rows=[]
 for a in s['assets']:
  o=lookup(scene,a['id'])
  if o is None or o.type!='MESH':raise ValueError('Missing independent Mesh object: '+a['id'])
  bm=bmesh.new();bm.from_mesh(o.data);bad=sum(not e.is_manifold for e in bm.edges);deg=sum(f.calc_area()<1e-12 for f in bm.faces);bm.free()
  unit=bool(np.max(np.abs(np.array(o.scale)-1))<1e-5)
  rows.append({'id':a['id'],'blender_object':o.name,'vertices':len(o.data.vertices),'nonmanifold_edges':bad,'degenerate_faces':deg,'unit_scale':unit,'passed':unit and (bad==0 or not a['closed_required']) and deg==0})
 return {'objects':rows,'passed':all(r['passed'] for r in rows),'semantic_segmentation_verified':False,'note':'Object separation audited; semantic correctness and occlusion completion require human review.'}

def build(s,path,out):
 scene=bpy.data.scenes.new(s['scene_id']);bpy.context.window.scene=scene;scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1
 collections={}
 for role,name in COLS.items():
  c=bpy.data.collections.new(name);scene.collection.children.link(c);collections[role]=c
 for a in s['assets']:
  if a['kind']=='in_template':raise ValueError('in_template assets require an existing blend, not build')
  if a['kind']=='box':
   bpy.ops.mesh.primitive_cube_add();o=bpy.context.object;o.dimensions=a['size_m'];bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
  else:
   source=(path.parent/a['path']).resolve();before=set(scene.objects)
   if source.suffix.lower() not in ['.glb','.gltf']:raise ValueError('Build accepts GLB/GLTF semantic assets; convert other formats explicitly')
   bpy.ops.import_scene.gltf(filepath=str(source));created=set(scene.objects)-before;meshes=[x for x in created if x.type=='MESH']
   if not meshes:raise ValueError('Asset contains no mesh')
   bpy.ops.object.select_all(action='DESELECT')
   for x in meshes:
    T=x.matrix_world.copy();x.parent=None;x.matrix_world=T;x.select_set(True)
   bpy.context.view_layer.objects.active=meshes[0]
   if len(meshes)>1:bpy.ops.object.join()
   o=bpy.context.object;bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
   if a.get('weld_distance_m'):
    bm=bmesh.new();bm.from_mesh(o.data);bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=a['weld_distance_m']);bm.to_mesh(o.data);bm.free();o.data.update()
   for x in created-set(meshes):
    if x.name in bpy.data.objects:bpy.data.objects.remove(x,do_unlink=True)
  o.name=s['scene_id']+'__'+a['id'];o['r2s_id']=a['id'];o['r2s_role']=a['role'];o.matrix_world=Matrix(a['T_world_object'])
  for c in list(o.users_collection):c.objects.unlink(o)
  collections[a['role']].objects.link(o)
  if a['kind']=='box' or 'color_linear' in a:
   m=bpy.data.materials.new(a['id']+'_Material');m.use_nodes=True;bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=(*a.get('color_linear',[.7,.7,.7]),1);bs.inputs['Roughness'].default_value=a.get('roughness',.5);o.data.materials.clear();o.data.materials.append(m)
 for c in s['cameras']:
  d=bpy.data.cameras.new(c['id']);o=bpy.data.objects.new(c['blender_object'],d);o['r2s_id']=c['id'];scene.collection.objects.link(o);apply_camera(scene,o,c)
 for l in s['lights']:
  d=bpy.data.lights.new(l['id'],'AREA');d.energy=l['power_W'];d.size=l['size_m'];d.color=l['color_linear'];o=bpy.data.objects.new(l['id'],d);o['r2s_id']=l['id'];scene.collection.objects.link(o);o.location=l['position_m']
  if 'target_m' in l:o.rotation_euler=(Vector(l['target_m'])-o.location).to_track_quat('-Z','Y').to_euler()
 scene.world=bpy.data.worlds.new('World_'+s['scene_id']);scene.world.use_nodes=True;scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=s['world_strength']
 bpy.ops.file.pack_all();bpy.ops.wm.save_as_mainfile(filepath=str(out));report=audit(scene,s);save(out.with_suffix('.audit.json'),report)
 if not report['passed']:raise ValueError('Mesh audit failed; artifact retained for inspection only')

def image_ops(image,c,nearest=False):
 op=c['pixel_ops']
 if 'crop_xywh' in op:
  x,y,w,h=map(int,op['crop_xywh']);image=image.crop((x,y,x+w,y+h))
 if 'resize_wh' in op:image=image.resize(tuple(map(int,op['resize_wh'])),Image.Resampling.NEAREST if nearest else Image.Resampling.BILINEAR)
 if op.get('rotate_180'):image=image.transpose(Image.Transpose.ROTATE_180)
 return image

def render(s,blend,out,samples,states,parameters):
 bpy.ops.wm.open_mainfile(filepath=str(blend));scene=bpy.context.scene
 scene.render.engine='CYCLES';scene.cycles.samples=samples;scene.cycles.use_denoising=True;scene.cycles.seed=42
 try:
  prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='CUDA';prefs.refresh_devices()
  for d in prefs.devices:d.use=d.type=='CUDA'
  scene.cycles.device='GPU' if any(d.type=='CUDA' for d in prefs.devices) else 'CPU'
 except Exception:scene.cycles.device='CPU'
 scene.render.image_settings.file_format='PNG';scene.render.image_settings.color_mode='RGB';scene.render.use_border=False
 # Fresh template each run; no repeated multiplication of an already fitted scene.
 for key,value in parameters.items():
  kind,name,field=key.split('/')
  if kind=='light' and field=='power_W':lookup(scene,name).data.energy=float(value)
  elif kind=='material' and field=='albedo_gain':
   o=lookup(scene,name)
   for slot in o.material_slots:
    if not slot.material:continue
    slot.material=slot.material.copy();m=slot.material;bs=m.node_tree.nodes.get('Principled BSDF');inp=bs.inputs['Base Color'];node=m.node_tree.nodes.new('ShaderNodeMixRGB');node.blend_type='MULTIPLY';node.use_clamp=True;node.inputs[0].default_value=1.;node.inputs[2].default_value=(value,value,value,1)
    if inp.is_linked:m.node_tree.links.new(inp.links[0].from_socket,node.inputs[1])
    else:node.inputs[1].default_value=inp.default_value
    m.node_tree.links.new(node.outputs[0],inp)
  else:raise ValueError('Appearance may only change declared light power or albedo gain: '+key)
 base={o.name:o.matrix_world.copy() for o in sorted(scene.objects,key=lambda o:parent_depth(o))};out.mkdir(parents=True,exist_ok=False);records=[];roles={x['id']:x['role'] for x in s['assets']}
 frames=states or [{'schema_version':'1.0','frame_id':0,'time_s':0.,'T_world_objects':{}}];previous=-float('inf');ids=set()
 for frame in frames:
  if frame['schema_version']!='1.0' or not np.isfinite(frame['time_s']) or frame['time_s']<=previous or not isinstance(frame['frame_id'],int) or frame['frame_id']<0 or frame['frame_id'] in ids:raise ValueError('Invalid state sequence')
  previous=frame['time_s'];ids.add(frame['frame_id'])
  for name,T in base.items():scene.objects[name].matrix_world=T
  for name,T in sorted(frame['T_world_objects'].items(),key=lambda pair:parent_depth(lookup(scene,pair[0])) if lookup(scene,pair[0]) else 0):
   if roles.get(name) not in ['robot','task','misc']:raise ValueError('State attempted to move frozen/unregistered object '+name)
   lookup(scene,name).matrix_world=Matrix(rigid(T).tolist())
  scene.view_layers[0].update()
  for c0 in s['cameras']:
   c=dict(c0)
   if c.get('mount','fixed')=='body':
    if c['id'] not in frame.get('T_world_optical_cameras',{}):raise ValueError('Missing moving camera pose')
    c['T_world_optical']=rigid(frame['T_world_optical_cameras'][c['id']]).tolist()
   elif c['id'] in frame.get('T_world_optical_cameras',{}):raise ValueError('State attempted to move fixed camera')
   cam=lookup(scene,c['id']) or lookup(scene,c['blender_object'])
   if cam is None:raise ValueError('Missing camera '+c['id'])
   apply_camera(scene,cam,c);folder=out/c['id'];folder.mkdir(exist_ok=True);file=folder/f"{frame['frame_id']:06d}.png";scene.render.filepath=str(file);bpy.ops.render.render(write_still=True)
   im=Image.open(file).convert('RGB');valid=Image.new('L',im.size,255)
   if c['distortion']['model']!='none':
    import cv2
    w,h=im.size;grid=np.stack(np.meshgrid(np.arange(w),np.arange(h)),axis=-1).astype(np.float32);K=np.array(c['K_native']);mapped=cv2.undistortPoints(grid.reshape(-1,1,2),K,np.array(c['distortion']['coefficients']),P=K).reshape(h,w,2)
    array=cv2.remap(np.asarray(im),mapped[:,:,0],mapped[:,:,1],cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT);im=Image.fromarray(array);valid=Image.fromarray((255*((mapped[:,:,0]>=0)&(mapped[:,:,0]<w-1)&(mapped[:,:,1]>=0)&(mapped[:,:,1]<h-1))).astype(np.uint8))
   image_ops(im,c).save(file);image_ops(valid,c,True).save(folder/f"{frame['frame_id']:06d}_valid.png")
   records.append({'frame_id':frame['frame_id'],'time_s':frame['time_s'],'camera_id':c['id'],'profile_id':profile_id(c),'rgb':str(file.relative_to(out)),'valid_mask':str((folder/f"{frame['frame_id']:06d}_valid.png").relative_to(out)),'width_height':list(pixel_transform(c)[1])})
 save(out/'render_manifest.json',{'schema_version':'1.0','scene_id':s['scene_id'],'template_sha256':sha(blend),'channels':['display_rgb_uint8','valid_mask_uint8'],'depth_available':False,'parameters':parameters,'frames':records})

def main(argv=None):
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['build','render','audit']);p.add_argument('--scene',required=True);p.add_argument('--out',required=True);p.add_argument('--blend');p.add_argument('--samples',type=int,default=32);p.add_argument('--states');p.add_argument('--parameters');a=p.parse_args(argv);path=pathlib.Path(a.scene).resolve();s=validate_scene(load(path),check_schema=False);out=pathlib.Path(a.out).resolve()
 if a.mode=='build':
  if out.exists():raise FileExistsError(out)
  out.parent.mkdir(parents=True,exist_ok=True);build(s,path,out)
 elif a.mode=='audit':
  bpy.ops.wm.open_mainfile(filepath=a.blend);report=audit(bpy.context.scene,s);save(out,report)
  if not report['passed']:raise ValueError('Mesh audit failed')
 else:render(s,pathlib.Path(a.blend),out,a.samples,[json.loads(x) for x in pathlib.Path(a.states).read_text().splitlines()] if a.states else None,load(a.parameters) if a.parameters else {})
if __name__=='__main__':main()
