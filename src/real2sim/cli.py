import argparse,pathlib,sys,json,subprocess,os
from .contracts import load,save,validate_scene,profile_id,pixel_transform,sha

def main():
 p=argparse.ArgumentParser(prog='r2s');sub=p.add_subparsers(dest='cmd',required=True)
 v=sub.add_parser('case-init');v.add_argument('case');v.add_argument('--cycles-python',required=True)
 v=sub.add_parser('case-run');v.add_argument('case');v.add_argument('--retry-failed',action='store_true')
 v=sub.add_parser('case-status');v.add_argument('case')
 v=sub.add_parser('case-review');v.add_argument('case');v.add_argument('--stage',required=True);v.add_argument('--token',required=True);v.add_argument('--decision',choices=['approve','reject'],required=True);v.add_argument('--by',required=True);v.add_argument('--note',required=True)
 v=sub.add_parser('fit-camera');v.add_argument('config');v.add_argument('--out',required=True)
 v=sub.add_parser('scan-register');v.add_argument('config');v.add_argument('--out',required=True)
 v=sub.add_parser('scan-inspect');v.add_argument('--obj',required=True);v.add_argument('--texture',required=True);v.add_argument('--out',required=True)
 v=sub.add_parser('scan-bake');v.add_argument('config');v.add_argument('--out',required=True);v.add_argument('--python',required=True,help='Existing bpy Python to package closed GLB')
 v=sub.add_parser('validate');v.add_argument('scene')
 v=sub.add_parser('run');v.add_argument('plan');v.add_argument('--out',required=True)
 v=sub.add_parser('video');v.add_argument('--manifest',required=True);v.add_argument('--cameras',nargs='+',required=True);v.add_argument('--out',required=True);v.add_argument('--ffmpeg',default='ffmpeg')
 v=sub.add_parser('fit-appearance');v.add_argument('config');v.add_argument('--out',required=True)
 for name in ['build','render','audit']:
  v=sub.add_parser(name);v.add_argument('--scene',required=True);v.add_argument('--out',required=True);v.add_argument('--blend');v.add_argument('--python',required=True,help='Existing Python with bpy; do not install/replace Newton');v.add_argument('--samples',type=int,default=32);v.add_argument('--states');v.add_argument('--parameters');v.add_argument('--mcp',help='Optional host:port; paths must be visible to Blender')
 v=sub.add_parser('inventory');v.add_argument('root');v.add_argument('--out',required=True)
 v=sub.add_parser('check-inventory');v.add_argument('manifest')
 v=sub.add_parser('freeze');v.add_argument('--scene',required=True);v.add_argument('--blend',required=True);v.add_argument('--out',required=True)
 v=sub.add_parser('calibrate');v.add_argument('--scene',required=True);v.add_argument('--camera',required=True);v.add_argument('--points',required=True);v.add_argument('--out',required=True)
 v=sub.add_parser('score');v.add_argument('--real',required=True);v.add_argument('--sim',required=True);v.add_argument('--mask',required=True);v.add_argument('--out',required=True);v.add_argument('--lpips-cache')
 v=sub.add_parser('convert');v.add_argument('--input',required=True);v.add_argument('--calibration',required=True);v.add_argument('--out',required=True)
 v=sub.add_parser('xarm7');v.add_argument('job',choices=['validate_kinematics','simulate_replay','generate_grasp','gripper_mapping','import_eef','trajectory_adapter']);v.add_argument('--case',required=True);v.add_argument('--newton-project',required=True);v.add_argument('job_args',nargs=argparse.REMAINDER)
 a=p.parse_args()
 if a.cmd.startswith('case-'):
  from . import cases
  if a.cmd=='case-init':print(cases.init(a.case,a.cycles_python))
  elif a.cmd=='case-run':
   result=cases.run(a.case,a.retry_failed);print(json.dumps(result,ensure_ascii=False,indent=2));sys.exit(0 if result['status']=='complete' else 2)
  elif a.cmd=='case-status':print(json.dumps(load(pathlib.Path(a.case)/'runs/status.json'),ensure_ascii=False,indent=2))
  else:cases.review(a.case,a.stage,a.token,a.decision,a.by,a.note)
  return
 if a.cmd=='fit-camera':
  from .camera_fit import fit
  fit(a.config,a.out);return
 if a.cmd in ['scan-register','scan-inspect','scan-bake']:
  from .scans import register,inspect,bake
  if a.cmd=='scan-register':register(a.config,a.out)
  elif a.cmd=='scan-inspect':inspect(a.obj,a.texture,a.out)
  else:
   bake(a.config,a.out);subprocess.run([a.python,str(pathlib.Path(__file__).parent/'workers/scan_asset.py'),str(pathlib.Path(a.out).resolve())],check=True)
  return
 if a.cmd=='video':
  from .video import encode
  encode(a.manifest,a.cameras,a.out,a.ffmpeg);return
 if a.cmd=='run':
  from .runner import run
  run(a.plan,a.out);return
 if a.cmd=='fit-appearance':
  from .appearance import fit
  fit(a.config,a.out);return
 if a.cmd=='validate':
  s=validate_scene(load(a.scene));print(json.dumps({'valid':True,'profiles':{c['id']:profile_id(c) for c in s['cameras']}}));return
 if a.cmd in ['build','render','audit']:
  validate_scene(load(a.scene));worker=pathlib.Path(__file__).parent/'workers/blender.py';args=[a.cmd,'--scene',str(pathlib.Path(a.scene).resolve()),'--out',str(pathlib.Path(a.out).resolve()),'--samples',str(a.samples)]
  for k in ['blend','states','parameters']:
   if getattr(a,k):args+=['--'+k,str(pathlib.Path(getattr(a,k)).resolve())]
  if a.mcp:
   from .mcp import execute
   host,port=a.mcp.rsplit(':',1);code='import runpy,sys\nsys.argv='+repr([str(worker),*args])+'\nrunpy.run_path('+repr(str(worker))+',run_name="__main__")';print(execute(code,host,int(port)))
  else:subprocess.run([a.python,str(worker),*args],check=True)
 elif a.cmd=='inventory':
  from .artifacts import inventory
  print(inventory(a.root,a.out))
 elif a.cmd=='check-inventory':
  from .artifacts import check_inventory
  report=check_inventory(a.manifest);print(json.dumps(report));sys.exit(0 if report['passed'] else 2)
 elif a.cmd=='freeze':
  from .artifacts import freeze
  validate_scene(load(a.scene));freeze(a.scene,a.blend,a.out)
 elif a.cmd=='calibrate':
  from .cameras import fit_extrinsics
  s=validate_scene(load(a.scene));c=next(c for c in s['cameras'] if c['id']==a.camera);T,report=fit_extrinsics(c,load(a.points));c['T_world_optical']=T.tolist();c['quality']='image_fitted';c['evidence']=str(pathlib.Path(a.out).with_suffix('.calibration.json'));save(a.out,s);save(c['evidence'],report)
 elif a.cmd=='score':
  from .metrics import score
  save(a.out,score(a.real,a.sim,a.mask,a.lpips_cache))
 elif a.cmd=='convert':
  import numpy as np
  from .trajectory import convert_episode
  out=pathlib.Path(a.out)
  if out.exists():raise FileExistsError(out)
  manifest=load(pathlib.Path(a.input)/'manifest.json')
  if manifest.get('joint_unit')!='rad' or manifest.get('time_unit')!='s' or manifest.get('gripper_convention')!='0=open,1=closed':raise ValueError('Normalized episode manifest unit mismatch')
  out.mkdir(parents=True);cal=load(a.calibration);files=sorted(pathlib.Path(a.input).glob('*.npz'))
  if not files:raise ValueError('No normalized episode NPZ inputs')
  rows=[]
  for f in files:
   d=dict(np.load(f,allow_pickle=False));converted=convert_episode(d,cal);np.savez_compressed(out/f.name,**converted);rows.append({'episode':f.stem,'frames':len(d['time']),'source_npz_sha256':sha(f)})
  save(out/'source_manifest.json',manifest);save(out/'calibration.json',cal)
  save(out/'conversion.json',{'source_manifest_sha256':sha(pathlib.Path(a.input)/'manifest.json'),'calibration_sha256':sha(a.calibration),'episodes':rows,'tcp_frame':'robot_base','position_unit':'m','quaternion':'xyzw','observation_action_separate':True})
 elif a.cmd=='xarm7':
  root=pathlib.Path(a.case).resolve();project=pathlib.Path(a.newton_project).resolve();s=load(root/'scene_config.json')
  if s.get('tcp_offset_m')!=[0,0,.172]:raise ValueError('This validated physical adapter supports xArm7 G2 TCP172 only; implement/test a new robot adapter for other TCPs')
  env=os.environ.copy();env['R2S_CASE_ROOT']=str(root);env['R2S_NEWTON_PROJECT']=str(project)
  args=a.job_args[1:] if a.job_args[:1]==['--'] else a.job_args
  subprocess.run([str(project/'.venv/bin/python'),str(pathlib.Path(__file__).parent/'adapters/xarm7'/(a.job+'.py')),*args],env=env,check=True)
if __name__=='__main__':main()
