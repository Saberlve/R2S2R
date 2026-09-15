import copy,json,pathlib,numpy as np,pytest
from PIL import Image
from scipy.spatial.transform import Rotation
from real2sim.contracts import save,sha,profile_id
from real2sim.scene.scans import register,bake,read_obj
from real2sim.align.camera_fit import fit,project

def test_scan_registration_preserves_scale_and_holdout(tmp_path):
 p=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1.]])*1000;R=Rotation.from_euler('z',.3).as_matrix();q=p*.001@R.T+[.2,.3,.4];c={'schema_version':'1.0','scan_points':p.tolist(),'world_points_m':q.tolist(),'split':['fit']*3+['holdout'],'units_to_m':.001};save(tmp_path/'c.json',c);o=register(tmp_path/'c.json',tmp_path/'out.json');assert o['holdout_rmse_m']<1e-10 and not o['scale_optimized'];assert np.isclose(np.linalg.det(np.array(o['T_world_scan_m'])[:3,:3]),1)

def fixture_scan(tmp):
 v=[(x,y,0) for y in [0,.5,1] for x in [0,.5,1]];faces=[]
 for j in range(2):
  for i in range(2):
   a=j*3+i;faces.extend([[a,a+1,a+4],[a,a+4,a+3]])
 lines=['v '+' '.join(map(str,p)) for p in v]+['vt '+str(p[0])+' '+str(p[1]) for p in v]+['f '+' '.join(f'{i+1}/{i+1}' for i in f) for f in faces];(tmp/'source.obj').write_text('\n'.join(lines));Image.new('RGB',(32,32),(30,160,50)).save(tmp/'tex.png')
 c={'schema_version':'1.0','id':'Door','source_obj':'source.obj','source_texture':'tex.png','units_to_m':1,'T_scan_m_plane':np.eye(4).tolist(),'T_world_asset':np.eye(4).tolist(),'crop':{'u_range_m':[0,1],'v_range_m':[0,1],'depth_range_m':[-.1,.1],'normal_abs_depth_min':.4,'green_gate':None},'texture_wh':[32,32],'max_fill_distance_m':.02,'max_unknown_fraction':.01,'surface':{'mode':'panel','grid_wh':[3,3],'smooth_sigma_cells':1,'trend_sigma_cells':2,'relief_limit_m':.025,'depth_sign':1,'thickness_m':.02,'target_size_m':[1,1],'max_grid_fill_distance_m':.05},'appearance':{'erase_mask':None,'exposure_normalization':None}};save(tmp/'c.json',c);return c

def test_uv_bake_and_bounded_hole_fill(tmp_path):
 c=fixture_scan(tmp_path);report=bake(tmp_path/'c.json',tmp_path/'good');assert report['unknown_fraction']==0 and report['observed_fraction']>.95;image=np.asarray(Image.open(tmp_path/'good/basecolor.png'));assert np.abs(image.astype(float)-[30,160,50]).max()<1
 c['crop']['u_range_m']=[0,2];save(tmp_path/'bad.json',c)
 with pytest.raises(ValueError):bake(tmp_path/'bad.json',tmp_path/'bad')

def camera_fixture(tmp):
 r=pathlib.Path(__file__).parents[1];s=json.loads((r/'examples/minimal/scene.json').read_text());c=s['cameras'][0];T=np.array(c['T_world_optical']);truef=c['K_native'][0][0];cx,cy=np.array(c['K_native'])[:2,2]
 P=np.array([[-.3,-.2,.8],[.3,-.2,.8],[-.3,.2,.8],[.3,.2,.8],[-.15,-.1,1.1],[.15,.1,1.05],[.1,-.15,.9],[-.12,.18,1.15]]);uv,_=project(P,T,truef,cx,cy)
 c['K_native'][0][0]=c['K_native'][1][1]=100.;c['T_world_optical'][2][3]+=.15;s['cameras'].append(copy.deepcopy(c));s['cameras'][1].update(id='FrozenCamera',blender_object='FrozenCamera',serial='synthetic-other')
 save(tmp/'scene.json',s);points=[{'world_m':p.tolist(),'pixel_xy':u.tolist(),'split':'fit' if i<6 else 'holdout'} for i,(p,u) in enumerate(zip(P,uv))];save(tmp/'points.json',{'scene_sha256':sha(tmp/'scene.json'),'profile_id':profile_id(c),'coordinates':'native_pixels','points':points,'lines':[],'depth_guard_points_world_m':P.tolist()})
 cfg={'schema_version':'1.0','scene':'scene.json','camera_id':c['id'],'annotations':'points.json','search':{'focal_bounds_px':[80,220],'position_bounds_m':[[-.5,-.5,1.2],[.5,.5,2.8]],'rotation_bound_rad':.65,'starts':8,'seed':24,'max_nfev':250,'focal_prior_px':truef,'focal_prior_log_sigma':2,'rotation_prior_rad':2,'point_sigma_px':1,'line_sigma_px':1}};save(tmp/'fit.json',cfg);return s

def test_focal_pose_fit_and_frozen_other_camera(tmp_path):
 s=camera_fixture(tmp_path);result=fit(tmp_path/'fit.json',tmp_path/'out');after=json.loads((tmp_path/'out/scene.json').read_text());assert result['after']['holdout_mean_px']<.1 and abs(result['focal_px']-140)<.5;assert after['assets']==s['assets'] and after['cameras'][1]==s['cameras'][1];assert not result['independent_physical_calibration']
 a=json.loads((tmp_path/'points.json').read_text());a['scene_sha256']='stale';save(tmp_path/'points.json',a)
 with pytest.raises(ValueError):fit(tmp_path/'fit.json',tmp_path/'stale')
