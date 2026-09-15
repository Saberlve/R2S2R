"""Image-based focal + 6DoF fitting when factory intrinsics are unavailable.
Only the selected camera changes; geometry/other cameras are immutable.
"""
import copy,hashlib,json,pathlib
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from ..contracts import load,save,sha,rigid,validate_scene,profile_id

def project(points,T,f,cx,cy):
 q=(np.asarray(points)-T[:3,3])@T[:3,:3]
 z=np.where(abs(q[:,2])>=1e-5,q[:,2],np.where(q[:,2]>=0,1e-5,-1e-5))
 return np.c_[cx+f*q[:,0]/z,cy+f*q[:,1]/z],q[:,2]

def fit(config_file,output):
 cfg=load(config_file);root=pathlib.Path(config_file).resolve().parent;out=pathlib.Path(output)
 if out.exists():raise FileExistsError(out)
 if set(cfg)!={'schema_version','scene','camera_id','annotations','search'} or cfg['schema_version']!='1.0':raise ValueError('Invalid camera-fit config')
 scene_path=root/cfg['scene'];s=validate_scene(load(scene_path));a=load(root/cfg['annotations']);opt=cfg['search'];cam=next(c for c in s['cameras'] if c['id']==cfg['camera_id'])
 if set(a)!={'scene_sha256','profile_id','coordinates','points','lines','depth_guard_points_world_m'}:raise ValueError('Invalid annotation contract')
 if a['scene_sha256']!=sha(scene_path) or a['profile_id']!=profile_id(cam):raise ValueError('Annotations refer to different geometry/camera profile')
 if a['coordinates']!='native_pixels' or cam['distortion']['model']!='none':raise ValueError('Joint focal fit requires explicitly undistorted or assumed-zero-distortion native pixels')
 expected={'focal_bounds_px','position_bounds_m','rotation_bound_rad','starts','seed','max_nfev','focal_prior_px','focal_prior_log_sigma','rotation_prior_rad','point_sigma_px','line_sigma_px'}
 if set(opt)!=expected:raise ValueError('Unknown/missing search parameters')
 if any(set(v)!={'world_m','pixel_xy','split'} for v in a['points']):raise ValueError('Invalid point fields')
 P=np.asarray([v['world_m'] for v in a['points']],float);uv=np.asarray([v['pixel_xy'] for v in a['points']],float);split=np.array([v['split'] for v in a['points']]);train=split=='fit';hold=split=='holdout'
 if P.ndim!=2 or P.shape[1]!=3 or uv.shape!=(len(P),2) or not np.isfinite(P).all() or not np.isfinite(uv).all() or train.sum()<4 or hold.sum()<1 or not np.all(train|hold):raise ValueError('Need finite points, at least 4 fit and 1 held-out point')
 if np.linalg.matrix_rank(P[train]-P[train].mean(0))<2:raise ValueError('Collinear fit points')
 lines=[]
 for v in a['lines']:
  if set(v)!={'world_points_m','pixel_endpoints_xy','split'}:raise ValueError('Invalid line fields')
  p=np.asarray(v['world_points_m'],float);ends=np.asarray(v['pixel_endpoints_xy'],float)
  if p.ndim!=2 or p.shape[1]!=3 or len(p)<2 or ends.shape!=(2,2) or not np.isfinite(p).all() or not np.isfinite(ends).all() or v['split'] not in ['fit','holdout']:raise ValueError('Invalid line annotation')
  direction=ends[1]-ends[0]
  if np.linalg.norm(direction)<1:raise ValueError('Degenerate image line')
  normal=np.array([-direction[1],direction[0]])/np.linalg.norm(direction);lines.append((p,ends[0],normal,v['split']))
 guards=np.asarray(a['depth_guard_points_world_m'],float)
 if guards.ndim!=2 or guards.shape[1]!=3 or not len(guards) or not np.isfinite(guards).all():raise ValueError('Need explicit finite positive-depth guard points')
 T0=rigid(cam['T_world_optical']);K=np.array(cam['K_native']);cx,cy=K[0,2],K[1,2];f0=K[0,0]
 if not np.isclose(K[0,0],K[1,1],rtol=1e-6):raise ValueError('Joint focal fit requires an explicit square-pixel initial model')
 flo,fhi=opt['focal_bounds_px'];pos=np.asarray(opt['position_bounds_m'],float);rb=opt['rotation_bound_rad']
 if pos.shape!=(2,3) or np.any(pos[0]>=pos[1]) or not 0<flo<fhi or not 0<rb<np.pi:raise ValueError('Invalid bounds')
 for key in ['starts','max_nfev']:
  if not isinstance(opt[key],int) or opt[key]<1:raise ValueError('Invalid search budget')
 for key in ['focal_prior_px','focal_prior_log_sigma','rotation_prior_rad','point_sigma_px','line_sigma_px']:
  if not np.isfinite(opt[key]) or opt[key]<=0:raise ValueError('Invalid prior/sigma')
 lo=np.r_[[-rb]*3,pos[0],np.log(flo)];hi=np.r_[[rb]*3,pos[1],np.log(fhi)]
 def decode(x):
  T=T0.copy();T[:3,:3]=Rotation.from_rotvec(x[:3]).as_matrix()@T0[:3,:3];T[:3,3]=x[3:6];return T,np.exp(x[6])
 def residual(x):
  T,f=decode(x);pred,depth=project(P[train],T,f,cx,cy);v=list(((pred-uv[train])/(opt['point_sigma_px']*np.sqrt(train.sum()))).ravel())
  depths=[depth]
  for p,origin,n,sp in lines:
   if sp=='fit':
    pp,dd=project(p,T,f,cx,cy);v.extend(((pp-origin)@n)/(opt['line_sigma_px']*np.sqrt(len(p))));depths.append(dd)
  v.append(np.log(f/opt['focal_prior_px'])/opt['focal_prior_log_sigma']);v.extend(x[:3]/opt['rotation_prior_rad']);v.extend(np.maximum(.05-project(guards,T,f,cx,cy)[1],0)*100)
  return np.asarray(v)
 rng=np.random.default_rng(opt['seed']);trials=[];best=None
 out.mkdir(parents=True)
 for i in range(opt['starts']):
  x=np.r_[rng.normal(0,.06,3),T0[:3,3]+rng.normal(0,.15,3),np.log(rng.uniform(flo,fhi))];x=np.clip(x,lo+1e-7,hi-1e-7)
  res=least_squares(residual,x,bounds=(lo,hi),loss='soft_l1',f_scale=2,max_nfev=opt['max_nfev'])
  trials.append({'start':i,'cost':float(res.cost),'nfev':res.nfev,'converged':bool(res.success)})
  if best is None or res.cost<best.cost:best=res
 T,f=decode(best.x)
 def metrics(T,f):
  pred,z=project(P,T,f,cx,cy);err=np.linalg.norm(pred-uv,axis=1);le=[]
  for p,o,n,sp in lines:
   q,d=project(p,T,f,cx,cy);le.append({'split':sp,'rmse_px':float(np.sqrt(np.mean(((q-o)@n)**2))),'positive_depth':bool(np.all(d>0))})
  return {'fit_mean_px':float(err[train].mean()),'holdout_mean_px':float(err[hold].mean()),'point_error_px':err.tolist(),'line_errors':le,'all_positive_depth':bool(np.all(z>0) and np.all(project(guards,T,f,cx,cy)[1]>0) and all(x['positive_depth'] for x in le))}
 before=metrics(T0,f0);after=metrics(T,f);sv=np.linalg.svd(best.jac,compute_uv=False)
 original=copy.deepcopy(s);cam['K_native']=[[float(f),0,float(cx)],[0,float(f),float(cy)],[0,0,1]];cam['T_world_optical']=T.tolist();cam['quality']='image_fitted';cam['evidence']='camera_fit_report.json'
 # Deep equality protects all non-target cameras, assets, lights and world parameters.
 unchanged=copy.deepcopy(s);unchanged['cameras']=copy.deepcopy(original['cameras'])
 assert unchanged==original
 assert [c for c in s['cameras'] if c['id']!=cam['id']]==[c for c in original['cameras'] if c['id']!=cam['id']]
 report={'schema_version':'1.0','method':'bounded multistart robust focal + six extrinsics; native point/line residuals','source_scene_sha256':sha(scene_path),'annotation_sha256':sha(root/cfg['annotations']),'before':before,'after':after,'trials':trials,'focal_px':float(f),'K_native':cam['K_native'],'T_world_optical':T.tolist(),'profile_before':profile_id(next(c for c in original['cameras'] if c['id']==cam['id'])),'profile_after':profile_id(cam),'parameters_at_bound':bool(np.any(np.minimum(best.x-lo,hi-best.x)<1e-3)),'regularized_jacobian_singular_values':sv.tolist(),'jacobian_note':'Includes priors; not physical parameter confidence','frozen_geometry_and_other_cameras_unchanged':True,'independent_physical_calibration':False,'automatic_adoption':False,'assumptions':['fixed principal point','square pixels','zero distortion','fixed geometry and robot pose'],'candidate_passes_depth_check':after['all_positive_depth'],'reference_coordinates':'native pixels; no implicit image resizing'}
 save(out/'scene.json',validate_scene(s));save(out/'camera_fit_report.json',report);return report
