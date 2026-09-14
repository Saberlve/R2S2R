import copy,json,pathlib,numpy as np,pytest
from real2sim.contracts import validate_scene,rigid,profile_id,pixel_transform,pose
from real2sim.cameras import fit_extrinsics
from real2sim.trajectory import convert_episode
from real2sim.artifacts import freeze,inventory,check_inventory
from real2sim.bridge import snapshot_from_newton
P=pathlib.Path(__file__).parents[1]
def scene():return json.loads((P/'examples/minimal/scene.json').read_text())
def test_scene_and_extra_fields():
 s=scene();validate_scene(s);s['unexpected']=1
 with pytest.raises(Exception):validate_scene(s)
def test_transform_rejects_scale_and_reflection():
 for x in [2,-1]:
  T=np.eye(4);T[0,0]=x
  with pytest.raises(ValueError):rigid(T)
def test_pixel_centers_profile_and_roundtrip():
 c=scene()['cameras'][0];h=profile_id(c);c['pixel_ops']={'crop_xywh':[10,20,100,80],'resize_wh':[50,40],'rotate_180':True};A,wh=pixel_transform(c)
 assert wh==(50,40) and profile_id(c)!=h
 p=np.array([25.,30.,1]);assert np.allclose(np.linalg.solve(A,A@p),p)
 assert np.allclose(A@np.array([10.,20.,1]),[49.25,39.25,1])
def test_pnp_native_profile():
 import cv2
 c=scene()['cameras'][0];xyz=np.array([[-.2,-.2,.8],[.2,-.2,.8],[-.2,.2,.8],[.2,.2,.8],[-.1,-.1,.9],[.1,.1,1.]])
 T=np.linalg.inv(np.array(c['T_world_optical']));uv=cv2.projectPoints(xyz,cv2.Rodrigues(T[:3,:3])[0],T[:3,3],np.array(c['K_native']),None)[0].reshape(-1,2)
 points={'profile_id':profile_id(c),'coordinates':'native_distorted_pixels','world_points_m':xyz.tolist(),'pixels_xy':uv.tolist()};estimated,report=fit_extrinsics(c,points)
 assert np.allclose(estimated,c['T_world_optical'],atol=1e-5) and not report['independent_validation']
 points['profile_id']='stale'
 with pytest.raises(ValueError):fit_extrinsics(c,points)
def test_joint_observation_action_separated():
 cal={'joint_axes':[[0,0,1]],'joint_origins_xyz_m_rpy_rad':[[0,0,0,0,0,0]],'T_flange_tcp':np.eye(4).tolist()};cal['T_flange_tcp'][0][3]=1
 d={'time':np.array([0,.1]),'q':np.zeros((2,1)),'action_q':np.ones((2,1)),'gripper':np.zeros(2),'action_gripper':np.ones(2)};o=convert_episode(d,cal)
 assert np.allclose(o['tcp_m'][:,0],1) and not np.allclose(o['tcp_m'],o['action_tcp_m'])
 d['time']=np.array([0,0])
 with pytest.raises(ValueError):convert_episode(d,cal)
def test_inventory_and_freeze_no_overwrite(tmp_path):
 raw=tmp_path/'raw';raw.mkdir();(raw/'f').write_text('one');inventory(raw,tmp_path/'manifest.json');assert check_inventory(tmp_path/'manifest.json')['passed'];(raw/'f').write_text('two');assert not check_inventory(tmp_path/'manifest.json')['passed']
 freeze(P/'examples/minimal/scene.json',raw/'f',tmp_path/'baseline')
 with pytest.raises(FileExistsError):freeze(P/'examples/minimal/scene.json',raw/'f',tmp_path/'baseline')
def test_newton_binding_missing_duplicate_and_units():
 class Array:
  def numpy(self):return np.array([[1,2,3,0,0,0,1.]])
 class Model:body_label=['link']
 class State:body_q=Array()
 T=np.eye(4);T[0,3]=.1;s=snapshot_from_newton(Model(),State(),{'mesh':{'body_label':'link','T_body_object':T}},0,0)
 assert np.isclose(s['T_world_objects']['mesh'][0][3],1.1)
 with pytest.raises(ValueError):snapshot_from_newton(Model(),State(),{'x':{'body_label':'missing','T_body_object':T}},0,0)

def test_masked_metrics_excludes_unmodelled_objects(tmp_path):
 from PIL import Image
 from real2sim.metrics import score
 a=np.zeros((32,32,3),np.uint8);b=a.copy();b[:8]=255;m=np.zeros((32,32),np.uint8);m[10:30,2:30]=255
 for name,v in [('a',a),('b',b),('mask',m)]:Image.fromarray(v).save(tmp_path/(name+'.png'))
 result=score(tmp_path/'a.png',tmp_path/'b.png',tmp_path/'mask.png');assert result['mae']==0 and abs(result['ssim']-1)<1e-10 and result['lpips'] is None
 Image.fromarray(m[:20]).save(tmp_path/'mask.png')
 with pytest.raises(ValueError):score(tmp_path/'a.png',tmp_path/'b.png',tmp_path/'mask.png')
def test_pipeline_does_not_allow_outside_output(tmp_path):
 from real2sim.runner import run
 plan={'schema_version':'1.0','variables':{},'stages':[{'id':'invalid','requires':[],'argv':['build','--out','/outside/scene.blend'],'outputs':[]}]};p=tmp_path/'plan.json';p.write_text(json.dumps(plan))
 with pytest.raises(ValueError):run(p,tmp_path/'run')


def test_moving_camera_uses_same_newton_snapshot():
 class Array:
  def numpy(self):return np.array([[1,2,3,0,0,0,1.]])
 class Model:body_label=['wrist']
 class State:body_q=Array()
 T=np.eye(4);T[2,3]=.08
 s=snapshot_from_newton(Model(),State(),{},5,.5,{'wrist_cam':{'body_label':'wrist','T_body_optical':T}})
 assert np.allclose(np.array(s['T_world_optical_cameras']['wrist_cam'])[:3,3],[1,2,3.08])
 with pytest.raises(ValueError):snapshot_from_newton(Model(),State(),{},.5,0)

def test_ids_are_global():
 s=scene();s['lights'][0]['id']=s['assets'][0]['id']
 with pytest.raises(ValueError):validate_scene(s)


def test_video_requires_synchronized_uniform_frames():
 from real2sim.video import frame_groups
 rows=[{'camera_id':c,'frame_id':i,'time_s':i/30,'profile_id':c,'width_height':[640,480]} for i in range(3) for c in ['b','wrist']]
 groups,wh,fps=frame_groups({'frames':rows},['b','wrist'])
 assert len(groups)==3 and wh==[640,480] and np.isclose(fps,30)
 rows[-1]['time_s']+=.01
 with pytest.raises(ValueError):frame_groups({'frames':rows},['b','wrist'])
