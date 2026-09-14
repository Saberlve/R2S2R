"""Headless inspect/edit for the reference runtime, not a physics model editor."""
import argparse,json,pathlib,sys
import bpy,numpy as np
from mathutils import Matrix,Vector
from server import copy_template,load,save
p=argparse.ArgumentParser();p.add_argument('command',choices=['inspect','edit']);p.add_argument('--template',required=True);p.add_argument('--out',required=True);p.add_argument('--patch');a=p.parse_args();source=pathlib.Path(a.template);out=pathlib.Path(a.out)
bpy.ops.wm.open_mainfile(filepath=str(source/'Baseline.blend'));manifest=load(source/'runtime_manifest.json');scene=bpy.data.scenes[manifest['scene']]
if bpy.context.window:bpy.context.window.scene=scene

def inventory():
 result={'units':'meters; Z up; matrices map local points to world','scene':scene.name,'objects':[],'materials':[],'world':{}}
 for o in scene.objects:
  r={'name':o.name,'type':o.type,'parent':o.parent.name if o.parent else None,'collections':[c.name for c in o.users_collection],'matrix_world':np.array(o.matrix_world).tolist(),'dimensions_m':list(o.dimensions),'materials':[s.material.name if s.material else None for s in o.material_slots]}
  if o.type=='LIGHT':r['light']={'type':o.data.type,'power_W':o.data.energy,'color_linear':list(o.data.color)}
  result['objects'].append(r)
 for m in bpy.data.materials:
  bs=m.node_tree.nodes.get('Principled BSDF') if m.use_nodes else None
  result['materials'].append({'name':m.name,'base_color':list(bs.inputs['Base Color'].default_value) if bs else list(m.diffuse_color),'texture_linked':bool(bs and bs.inputs['Base Color'].is_linked),'roughness':float(bs.inputs['Roughness'].default_value) if bs else None})
 if scene.world and scene.world.use_nodes:
  bg=scene.world.node_tree.nodes.get('Background');result['world']={'strength':float(bg.inputs['Strength'].default_value) if bg else None}
 result['external_images']=[i.filepath for i in bpy.data.images if not i.packed_file and i.source not in ('GENERATED','VIEWER')]
 return result
before=inventory();save(out/'scene_inventory.json',before)
if a.command=='edit':
 patch=load(a.patch);allowed={'schema_version','world_strength','object_transforms','lights','materials'}
 if set(patch)-allowed:raise ValueError('Unknown patch fields: '+str(set(patch)-allowed))
 if patch.get('schema_version')!=1:raise ValueError('schema_version must be 1')
 def number(v,lo=0,hi=float('inf')):
  v=float(v)
  if not np.isfinite(v) or not lo<=v<=hi:raise ValueError('Invalid finite parameter '+str(v))
  return v
 def color(v):
  if len(v)!=3:raise ValueError('RGB requires 3 values')
  return [number(x,0,1) for x in v]
 def rigid(v):
  T=np.array(v,dtype=float)
  if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(T[:3,:3])<.999:raise ValueError('Transform must be rigid, in meters')
  return Matrix(T.tolist())
 def depth(o):return 0 if o.parent is None else 1+depth(o.parent)
 for name,v in sorted(patch.get('object_transforms',{}).items(),key=lambda item:depth(scene.objects[item[0]])):
  o=scene.objects[name]
  if o.type=='CAMERA' or any(c.name.startswith('COL_Robot') for c in o.users_collection) or name in [x['name'] for x in manifest['joints']] or name in manifest['attachments']:raise ValueError('Use calibrated camera/robot bindings instead of a visual transform patch: '+name)
  o.matrix_world=rigid(v);scene.view_layers[0].update()
 for name,v in patch.get('lights',{}).items():
  if set(v)-{'power_W','color_linear','position_m','target_m','size_m'}:raise ValueError('Unknown light fields')
  o=scene.objects[name]
  if o.type!='LIGHT':raise ValueError(name+' is not a light')
  if 'power_W' in v:o.data.energy=number(v['power_W'])
  if 'color_linear' in v:o.data.color=color(v['color_linear'])
  if 'position_m' in v:
   xyz=np.asarray(v['position_m'],float)
   if xyz.shape!=(3,) or not np.isfinite(xyz).all():raise ValueError('position_m must have three finite values')
   T=o.matrix_world.copy();T.translation=xyz;o.matrix_world=T
  if 'target_m' in v:
   xyz=np.asarray(v['target_m'],float)
   if xyz.shape!=(3,) or not np.isfinite(xyz).all():raise ValueError('target_m must have three finite values')
   pos=o.matrix_world.translation.copy();T=(Vector(xyz)-pos).to_track_quat('-Z','Y').to_matrix().to_4x4();T.translation=pos;o.matrix_world=T
  if 'size_m' in v:
   if o.data.type!='AREA':raise ValueError('size_m is only supported for AREA lights')
   o.data.size=number(v['size_m'],1e-6)
 for name,v in patch.get('materials',{}).items():
  if set(v)-{'base_color_linear','albedo_gain','roughness'}:raise ValueError('Unknown material fields')
  m=bpy.data.materials[name];bs=m.node_tree.nodes.get('Principled BSDF') if m.use_nodes else None
  if bs is None:raise ValueError('Material needs Principled BSDF: '+name)
  if 'base_color_linear' in v:
   if bs.inputs['Base Color'].is_linked:raise ValueError('Textured material: use albedo_gain to preserve its texture')
   bs.inputs['Base Color'].default_value=(*color(v['base_color_linear']),1)
  if 'roughness' in v:
   if bs.inputs['Roughness'].is_linked:raise ValueError('Linked roughness requires explicit node edit')
   bs.inputs['Roughness'].default_value=number(v['roughness'],0,1)
  if 'albedo_gain' in v:
   gain=number(v['albedo_gain'],0,4);inp=bs.inputs['Base Color'];node=m.node_tree.nodes.new('ShaderNodeMixRGB');node.blend_type='MULTIPLY';node.use_clamp=True;node.inputs[0].default_value=1;node.inputs[2].default_value=(gain,gain,gain,1)
   if inp.is_linked:m.node_tree.links.new(inp.links[0].from_socket,node.inputs[1])
   else:node.inputs[1].default_value=inp.default_value
   m.node_tree.links.new(node.outputs[0],inp)
 if 'world_strength' in patch:
  if not scene.world or not scene.world.use_nodes or scene.world.node_tree.nodes.get('Background') is None:raise ValueError('No world Background node')
  scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=number(patch['world_strength'])
 scene.view_layers[0].update();after=inventory();locked=[o['name'] for o in before['objects'] if o['type']=='CAMERA' or any(n.startswith('COL_Robot') for n in o['collections'])];assert all(np.allclose(next(o for o in before['objects'] if o['name']==n)['matrix_world'],next(o for o in after['objects'] if o['name']==n)['matrix_world'],atol=1e-6,rtol=0) for n in locked),'Patch moved a locked robot/camera indirectly';save(out/'scene_inventory_after.json',after);save(out/'patch.json',patch);dest=out/'template';copy_template(source,dest)
 bpy.ops.file.pack_all();bpy.context.preferences.filepaths.save_version=0;bpy.ops.wm.save_as_mainfile(filepath=str(dest/'Baseline.blend'))
 save(out/'edit_report.json',{'visual_only':True,'physics_configuration_changed':False,'source':str(source),'output':str(dest),'changed_transforms':[o['name'] for o in after['objects'] if next(x for x in before['objects'] if x['name']==o['name'])['matrix_world']!=o['matrix_world']],'patch':patch})
print('SCENE_TASK_COMPLETE',out)
