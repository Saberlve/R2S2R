"""Static textured scanner surfaces -> independent regularized visual assets.
OBJ units, semantic crop and registration are explicit operator inputs.
"""
import pathlib,numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt,gaussian_filter
from scipy.spatial import cKDTree
from ..contracts import load,save,sha,rigid

def read_obj(path):
 v=[];uv=[];faces=[];ft=[];materials=set()
 for line in pathlib.Path(path).open(encoding='utf-8-sig'):
  a=line.split()
  if not a:continue
  if a[0]=='v':v.append(list(map(float,a[1:4])))
  elif a[0]=='vt':uv.append(list(map(float,a[1:3])))
  elif a[0]=='usemtl':materials.add(' '.join(a[1:]))
  elif a[0]=='f':
   if len(a)!=4:raise ValueError('Triangulate the source explicitly; OBJ input must contain triangles')
   pairs=[x.split('/') for x in a[1:]]
   if any(len(x)<2 or not x[1] for x in pairs):raise ValueError('Source needs per-corner UV indices')
   def index(i,n):return int(i)-1 if int(i)>0 else n+int(i)
   faces.append([index(x[0],len(v)) for x in pairs]);ft.append([index(x[1],len(uv)) for x in pairs])
 v=np.asarray(v,float);uv=np.asarray(uv,float);f=np.asarray(faces,int);ft=np.asarray(ft,int)
 if len(materials)>1:raise ValueError('Multiple source materials need separate atlas processing')
 if v.ndim!=2 or v.shape[1]!=3 or uv.ndim!=2 or uv.shape[1]!=2 or f.ndim!=2 or not len(f) or not np.isfinite(v).all() or not np.isfinite(uv).all():raise ValueError('Invalid textured OBJ')
 if f.min()<0 or f.max()>=len(v) or ft.min()<0 or ft.max()>=len(uv):raise ValueError('Invalid OBJ indices')
 return v,uv,f,ft

def inspect(obj,texture,output):
 if pathlib.Path(output).exists():raise FileExistsError(output)
 v,uv,f,ft=read_obj(obj)
 with Image.open(texture) as im:wh=list(im.size)
 edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1);_,count=np.unique(edges,axis=0,return_counts=True)
 report={'source_obj_sha256':sha(obj),'source_texture_sha256':sha(texture),'vertices':len(v),'triangles':len(f),'texture_wh':wh,'bounds_scan_units':[v.min(0).tolist(),v.max(0).tolist()],'indexed_boundary_edges':int((count==1).sum()),'uv_seams_may_duplicate_vertices':True,'units_verified':False,'closed_collision_asset':False,'depth_frames_verified':False}
 save(output,report);return report

def register(config,output):
 if pathlib.Path(output).exists():raise FileExistsError(output)
 c=load(config)
 if set(c)!={'schema_version','scan_points','world_points_m','split','units_to_m'} or c['schema_version']!='1.0':raise ValueError('Invalid registration contract')
 scale=float(c['units_to_m']);p=np.asarray(c['scan_points'],float)*scale;q=np.asarray(c['world_points_m'],float);split=np.asarray(c['split']);fit=split=='fit';hold=split=='holdout'
 if not np.isfinite(scale) or scale<=0 or p.ndim!=2 or p.shape[1]!=3 or p.shape!=q.shape or split.shape!=(len(p),) or fit.sum()<3 or hold.sum()<1 or not np.all(fit|hold) or not np.isfinite(p).all() or not np.isfinite(q).all():raise ValueError('Need positive explicit scale, 3 fit + 1 holdout 3D correspondences')
 x=p[fit]-p[fit].mean(0);y=q[fit]-q[fit].mean(0)
 if min(np.linalg.matrix_rank(x),np.linalg.matrix_rank(y))<2:raise ValueError('Collinear scan anchors')
 U,_,Vt=np.linalg.svd(x.T@y);D=np.eye(3);D[2,2]=np.linalg.det(Vt.T@U.T);R=Vt.T@D@U.T;T=np.eye(4);T[:3,:3]=R;T[:3,3]=q[fit].mean(0)-R@p[fit].mean(0);rigid(T);err=np.linalg.norm(p@R.T+T[:3,3]-q,axis=1)
 report={'units_to_m':scale,'T_world_scan_m':T.tolist(),'fit_rmse_m':float(np.sqrt(np.mean(err[fit]**2))),'holdout_rmse_m':float(np.sqrt(np.mean(err[hold]**2))),'errors_m':err.tolist(),'scale_optimized':False,'automatic_adoption':False,'coordinate_chain':'world = T_world_scan_m @ [units_to_m * raw_scan_point, 1]'};save(output,report);return report

def bake(config_file,output):
 import cv2
 c=load(config_file);root=pathlib.Path(config_file).resolve().parent;out=pathlib.Path(output)
 expected={'schema_version','id','source_obj','source_texture','units_to_m','T_scan_m_plane','T_world_asset','crop','texture_wh','max_fill_distance_m','max_unknown_fraction','surface','appearance'}
 if set(c)!=expected or c['schema_version']!='1.0':raise ValueError('Invalid scan bake config')
 if out.exists():raise FileExistsError(out)
 scale=c['units_to_m'];T=rigid(c['T_scan_m_plane']);world=rigid(c['T_world_asset']);v,uv,f,ft=read_obj(root/c['source_obj']);v=(v*scale-T[:3,3])@T[:3,:3]
 if not np.isfinite(scale) or scale<=0:raise ValueError('Units must be explicitly positive')
 crop=c['crop'];required={'u_range_m','v_range_m','depth_range_m','normal_abs_depth_min','green_gate'}
 if set(crop)!=required:raise ValueError('Invalid semantic crop')
 ranges=np.asarray([crop['u_range_m'],crop['v_range_m'],crop['depth_range_m']],float)
 if ranges.shape!=(3,2) or not np.isfinite(ranges).all() or np.any(ranges[:,0]>=ranges[:,1]):raise ValueError('Invalid crop ranges')
 W,H=c['texture_wh']
 if not isinstance(W,int) or not isinstance(H,int) or min(W,H)<8:raise ValueError('Texture dimensions must be integer >=8')
 p=v[f].mean(1);n=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]);n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-15)
 atlas=np.asarray(Image.open(root/c['source_texture']).convert('RGB'));uc=uv[ft].mean(1);rgb=atlas[np.clip(((1-uc[:,1])*(len(atlas)-1)).astype(int),0,len(atlas)-1),np.clip((uc[:,0]*(atlas.shape[1]-1)).astype(int),0,atlas.shape[1]-1)]/255
 selected=np.all((p>=ranges[:,0])&(p<=ranges[:,1]),axis=1)&(abs(n[:,2])>=crop['normal_abs_depth_min'])
 gate=crop['green_gate']
 if gate is not None:
  if set(gate)!={'min_g','g_over_r','g_over_b'}:raise ValueError('Invalid color gate')
  selected &= (rgb[:,1]>=gate['min_g'])&(rgb[:,1]>=rgb[:,0]*gate['g_over_r'])&(rgb[:,1]>=rgb[:,2]*gate['g_over_b'])
 if selected.sum()<3:raise ValueError('Crop has too few triangles')
 low=ranges[:2,0];high=ranges[:2,1];q=np.c_[(v[:,0]-low[0])/(high[0]-low[0])*(W-1),(high[1]-v[:,1])/(high[1]-low[1])*(H-1)];mx=np.full((H,W),-1,np.float32);my=mx.copy();zbuf=np.full((H,W),np.inf)
 for face,texids in zip(f[selected],ft[selected]):
  a,b,d=q[face];x0=max(0,int(np.floor(min(a[0],b[0],d[0]))));x1=min(W-1,int(np.ceil(max(a[0],b[0],d[0]))));y0=max(0,int(np.floor(min(a[1],b[1],d[1]))));y1=min(H-1,int(np.ceil(max(a[1],b[1],d[1]))))
  det=(b[1]-d[1])*(a[0]-d[0])+(d[0]-b[0])*(a[1]-d[1])
  if x0>x1 or y0>y1 or abs(det)<1e-8:continue
  X,Y=np.meshgrid(np.arange(x0,x1+1),np.arange(y0,y1+1));wa=((b[1]-d[1])*(X-d[0])+(d[0]-b[0])*(Y-d[1]))/det;wb=((d[1]-a[1])*(X-d[0])+(a[0]-d[0])*(Y-d[1]))/det;wc=1-wa-wb;depth=abs(wa*v[face[0],2]+wb*v[face[1],2]+wc*v[face[2],2]);z=zbuf[y0:y1+1,x0:x1+1];ok=(wa>=-.001)&(wb>=-.001)&(wc>=-.001)&(depth<z)
  mx[y0:y1+1,x0:x1+1][ok]=(wa*uv[texids[0],0]+wb*uv[texids[1],0]+wc*uv[texids[2],0])[ok];my[y0:y1+1,x0:x1+1][ok]=(wa*uv[texids[0],1]+wb*uv[texids[1],1]+wc*uv[texids[2],1])[ok];z[ok]=depth[ok]
 observed=mx>=0
 if not observed.any():raise ValueError('No observed texture pixels')
 distance,indices=distance_transform_edt(~observed,sampling=[(high[1]-low[1])/(H-1),(high[0]-low[0])/(W-1)],return_indices=True)
 if c['max_fill_distance_m']<0 or not 0<=c['max_unknown_fraction']<1:raise ValueError('Invalid hole-fill limits')
 filled=(~observed)&(distance<=c['max_fill_distance_m']);unknown=~(observed|filled);mx[filled]=mx[tuple(indices[:,filled])];my[filled]=my[tuple(indices[:,filled])]
 image=cv2.remap(atlas,mx*(atlas.shape[1]-1),(1-my)*(atlas.shape[0]-1),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT);image[unknown]=127
 app=c['appearance']
 if set(app)!={'erase_mask','exposure_normalization'}:raise ValueError('Invalid appearance controls')
 if app['erase_mask']:
  erase=np.asarray(Image.open(root/app['erase_mask']).convert('L'))
  if erase.shape!=(H,W):raise ValueError('Erase mask must exactly match baked texture')
  image=cv2.inpaint(image,erase,7,cv2.INPAINT_TELEA)
 norm=app['exposure_normalization']
 if norm is not None:
  if set(norm)!={'sigma_px','target_mean','gain_bounds'} or norm['sigma_px']<=0:raise ValueError('Invalid exposure normalization')
  lowpass=gaussian_filter(image.astype(float).mean(2)/255,norm['sigma_px']);gain=np.clip(norm['target_mean']/np.maximum(lowpass,.01),*norm['gain_bounds']);image=np.clip(image.astype(float)*gain[:,:,None],0,255).astype(np.uint8)
 out.mkdir(parents=True);Image.fromarray(image).save(out/'basecolor.png');Image.fromarray((observed*255).astype('uint8')).save(out/'observed_mask.png');Image.fromarray((filled*255).astype('uint8')).save(out/'filled_mask.png');np.save(out/'fill_distance_m.npy',distance.astype(np.float32))
 surf=c['surface'];expected_surface={'mode','grid_wh','smooth_sigma_cells','trend_sigma_cells','relief_limit_m','depth_sign','thickness_m','target_size_m','max_grid_fill_distance_m'}
 if set(surf)!=expected_surface or surf['mode'] not in ['panel','relief'] or surf['thickness_m']<=0:raise ValueError('Invalid regularized surface contract')
 if unknown.mean()>c['max_unknown_fraction']:
  save(out/'rejected.json',{'unknown_fraction':float(unknown.mean()),'reason':'Need better crop/scan; refusing unbounded hole completion'});raise ValueError('Insufficient observed surface coverage')
 nx,ny=surf['grid_wh'] if surf['mode']=='relief' else [2,2]
 if not all(isinstance(x,int) and x>=2 for x in [nx,ny]):raise ValueError('Invalid grid size')
 U,V=np.meshgrid(np.linspace(low[0],high[0],nx),np.linspace(low[1],high[1],ny));height=np.zeros((ny,nx));max_grid_distance=0.
 if surf['mode']=='relief':
  if surf['depth_sign'] not in [-1,1] or surf['relief_limit_m']<=0 or min(surf['smooth_sigma_cells'],surf['trend_sigma_cells'])<=0:raise ValueError('Invalid relief controls')
  tree=cKDTree(p[selected,:2]);dist,idx=tree.query(np.c_[U.ravel(),V.ravel()],k=min(8,int(selected.sum())));weights=1/np.maximum(dist,.004);weights/=weights.sum(1)[:,None];height=(p[selected,2][idx]*weights).sum(1).reshape(ny,nx);height=gaussian_filter(height,surf['smooth_sigma_cells']);trend=gaussian_filter(height,surf['trend_sigma_cells']);height=surf['depth_sign']*np.clip(height-trend,-surf['relief_limit_m'],surf['relief_limit_m']);max_grid_distance=float(dist[:,0].max())
 if surf['mode']=='relief' and (surf['target_size_m'] is not None or max_grid_distance>surf['max_grid_fill_distance_m']):raise ValueError('Relief cannot stretch to target dimensions or complete geometry beyond its explicit distance limit')
 verts=np.c_[U.ravel(),V.ravel(),height.ravel()];texuv=np.c_[((U-low[0])/(high[0]-low[0])).ravel(),((V-low[1])/(high[1]-low[1])).ravel()];faces=[]
 for j in range(ny-1):
  for i in range(nx-1):
   x=j*nx+i;faces.extend([[x,x+1,x+nx+1],[x,x+nx+1,x+nx]])
 if surf['mode']=='panel':
  size=np.asarray(surf['target_size_m'],float)
  if size.shape!=(2,) or not np.isfinite(size).all() or np.any(size<=0):raise ValueError('Panel requires independently specified width/height in meters')
  verts[:,:2]=(verts[:,:2]-(low+high)/2)/(high-low)*size
 with (out/'surface.obj').open('w',encoding='utf-8') as stream:
  stream.write('mtllib surface.mtl\no ScanSurface\n')
  for x in verts:stream.write('v '+' '.join(map(str,x))+'\n')
  for x in texuv:stream.write('vt '+' '.join(map(str,x))+'\n')
  stream.write('usemtl ScanTexture\n')
  for x in faces:stream.write('f '+' '.join(f'{i+1}/{i+1}' for i in x)+'\n')
 (out/'surface.mtl').write_text('newmtl ScanTexture\nKd 1 1 1\nmap_Kd basecolor.png\n')
 report={'schema_version':'1.0','source_obj_sha256':sha(root/c['source_obj']),'source_texture_sha256':sha(root/c['source_texture']),'config_sha256':sha(config_file),'selected_triangles':int(selected.sum()),'observed_fraction':float(observed.mean()),'filled_fraction':float(filled.mean()),'unknown_fraction':float(unknown.mean()),'max_grid_nearest_distance_m':max_grid_distance,'mode':surf['mode'],'relief_range_m':[float(height.min()),float(height.max())],'target_size_m':surf['target_size_m'],'output_front_triangles':len(faces),'texture_is_true_albedo':False,'photographed_lighting_remains':True,'physics_role':'visual_only_static; supply independent simple collision proxy','regularization':'relief removes low frequency scan bow, clips local depth; panel uses a clean plane','erase_mask_sha256':sha(root/app['erase_mask']) if app['erase_mask'] else None}
 save(out/'scan_report.json',report);save(out/'asset.json',{'id':c['id'],'role':'background','kind':'mesh','path':'asset.glb','T_world_object':world.tolist(),'closed_required':True,'confidence':'estimated','weld_distance_m':1e-7});save(out/'build.json',{'thickness_m':surf['thickness_m'],'mode':surf['mode']});return report
