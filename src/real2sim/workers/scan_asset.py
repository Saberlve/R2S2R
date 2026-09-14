"""Convert one prepared scanner surface into one packed, closed GLB object."""
import sys,pathlib,json
import bpy,bmesh
root=pathlib.Path(sys.argv[1]);out=root/'asset.glb'
if out.exists():raise FileExistsError(out)
cfg=json.loads((root/'build.json').read_text());asset=json.loads((root/'asset.json').read_text());scene=bpy.data.scenes.new('PreparedScan');bpy.context.window.scene=scene
# Generated OBJ is already expressed in declared asset-local meters; avoid importer axis guesses.
v=[];uv=[];faces=[];uvfaces=[]
for line in (root/'surface.obj').read_text().splitlines():
 parts=line.split()
 if not parts:continue
 if parts[0]=='v':v.append(tuple(map(float,parts[1:4])))
 elif parts[0]=='vt':uv.append(tuple(map(float,parts[1:3])))
 elif parts[0]=='f':
  pairs=[x.split('/') for x in parts[1:]];faces.append([int(x[0])-1 for x in pairs]);uvfaces.append([int(x[1])-1 for x in pairs])
mesh=bpy.data.meshes.new(asset['id']);mesh.from_pydata(v,[],faces);mesh.update();o=bpy.data.objects.new(asset['id'],mesh);scene.collection.objects.link(o);bpy.context.view_layer.objects.active=o;o.select_set(True)
layer=mesh.uv_layers.new(name='ScanUV')
for poly,indices in zip(mesh.polygons,uvfaces):
 for loop_index,uv_index in zip(poly.loop_indices,indices):layer.data[loop_index].uv=uv[uv_index]
mat=bpy.data.materials.new(asset['id']+'_Texture');mat.use_nodes=True;node=mat.node_tree.nodes.new('ShaderNodeTexImage');node.image=bpy.data.images.load(str(root/'basecolor.png'));mat.node_tree.links.new(node.outputs['Color'],mat.node_tree.nodes['Principled BSDF'].inputs['Base Color']);mesh.materials.append(mat)
solid=o.modifiers.new('VisualThickness','SOLIDIFY');solid.thickness=cfg['thickness_m'];solid.offset=-1.;bpy.ops.object.modifier_apply(modifier=solid.name)
for poly in o.data.polygons:poly.use_smooth=cfg['mode']=='relief'
for mat in o.data.materials:
 if mat.use_nodes:
  bs=mat.node_tree.nodes.get('Principled BSDF')
  if bs:bs.inputs['Roughness'].default_value=.98 if cfg['mode']=='relief' else .62
bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);bm=bmesh.new();bm.from_mesh(o.data);bad=sum(not e.is_manifold for e in bm.edges);bm.free()
if bad:raise ValueError('Prepared scan shell is not closed')
bpy.ops.object.select_all(action='DESELECT');o.select_set(True);bpy.ops.export_scene.gltf(filepath=str(out),export_format='GLB',use_selection=True,use_active_scene=True)
import struct
data=out.read_bytes();chunk_size=struct.unpack_from('<I',data,12)[0];gltf=json.loads(data[20:20+chunk_size])
if len(gltf['meshes'])!=1 or len(gltf['scenes'])!=1:raise ValueError('Export unexpectedly includes another object or scene')
(root/'mesh_audit.json').write_text(json.dumps({'glb_meshes':len(gltf['meshes']),'glb_scenes':len(gltf['scenes']),'vertices':len(o.data.vertices),'faces':len(o.data.polygons),'nonmanifold_edges':bad,'unit_scale':all(abs(x-1)<1e-6 for x in o.scale),'visual_only_static':True,'geometry_is_regularized_not_raw':True},indent=2))
