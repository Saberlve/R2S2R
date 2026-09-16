"""Grouped r2s CLI: scene / align / traj / tactile domains plus top-level workflow verbs.

Legacy flat verbs (e.g. `r2s scan-bake`, `r2s calibrate`, `r2s xarm7 ...`) are
rewritten to their grouped form with a deprecation warning on stderr.
"""
import argparse,pathlib,sys,json,subprocess,os
from .contracts import load,save,validate_scene,profile_id,pixel_transform,sha

SCENE_COMMANDS=('validate','build','audit','render','scan-inspect','scan-register','scan-bake','fit-appearance','score','video')
ALIGN_COMMANDS=('calibrate','fit-camera')
TRAJ_COMMANDS=('convert',)
# Verb names must stay unique across groups: LEGACY flattens them into one mapping, so a
# duplicate (e.g. `asset validate`) would silently steal the flat alias `r2s validate`.
ASSET_COMMANDS=('list','show','check')
GROUP_COMMANDS={'scene':SCENE_COMMANDS,'align':ALIGN_COMMANDS,'traj':TRAJ_COMMANDS,'asset':ASSET_COMMANDS}
LEGACY={verb:group for group,verbs in GROUP_COMMANDS.items() for verb in verbs}
LEGACY['xarm7']='traj'

TOP_COMMANDS=('freeze','inventory','check-inventory')

def normalize_command(argv):
 """Rewrite legacy flat verbs to grouped form; pass everything else through."""
 argv=list(argv)
 if argv and argv[0] in LEGACY:return [LEGACY[argv[0]],*argv]
 return argv

def allowlisted(argv):
 """Command allowlist shared by cases.py and runner.py (grouped + top-level)."""
 if argv and argv[0] in TOP_COMMANDS:return True
 return len(argv)>1 and argv[0] in GROUP_COMMANDS and argv[1] in GROUP_COMMANDS[argv[0]]

def submodule_root():
 """The Data-MechanicSim submodule a `--recursive` clone carries."""
 return pathlib.Path(__file__).resolve().parents[2]/'external'/'Data-MechanicSim'

def default_newton_project():
 """R2S_NEWTON_PROJECT, else the external/Data-MechanicSim submodule once it is populated.

 A clone made without `--recursive` leaves an empty directory, which must not be mistaken for a
 usable checkout -- same rule as tools/server.py.
 """
 explicit=os.environ.get('R2S_NEWTON_PROJECT')
 if explicit:return explicit
 candidate=submodule_root()
 return str(candidate) if (candidate/'newton_gen').is_dir() else None

def default_cycles_python():
 """R2S_CYCLES_PYTHON, else the render_cycles environment `uv sync` builds in the submodule."""
 explicit=os.environ.get('R2S_CYCLES_PYTHON')
 if explicit:return explicit
 candidate=submodule_root()/'render_cycles'/'.venv'/'bin/python'
 return str(candidate) if candidate.is_file() else None

def build_parser():
 p=argparse.ArgumentParser(prog='r2s');sub=p.add_subparsers(dest='group',required=True)
 s=sub.add_parser('scene',help='Scene visual reconstruction');sp=s.add_subparsers(dest='verb',required=True)
 v=sp.add_parser('validate');v.add_argument('scene')
 for name in ['build','render','audit']:
  v=sp.add_parser(name);v.add_argument('--scene',required=True);v.add_argument('--out',required=True);v.add_argument('--blend');v.add_argument('--python',required=True,help='Existing Python with bpy; do not install/replace Newton');v.add_argument('--samples',type=int,default=32);v.add_argument('--states');v.add_argument('--parameters');v.add_argument('--mcp',help='Optional host:port; paths must be visible to Blender')
 v=sp.add_parser('scan-register');v.add_argument('config');v.add_argument('--out',required=True)
 v=sp.add_parser('scan-inspect');v.add_argument('--obj',required=True);v.add_argument('--texture',required=True);v.add_argument('--out',required=True)
 v=sp.add_parser('scan-bake');v.add_argument('config');v.add_argument('--out',required=True);v.add_argument('--python',required=True,help='Existing bpy Python to package closed GLB')
 v=sp.add_parser('fit-appearance');v.add_argument('config');v.add_argument('--out',required=True)
 v=sp.add_parser('score');v.add_argument('--real',required=True);v.add_argument('--sim',required=True);v.add_argument('--mask',required=True);v.add_argument('--out',required=True);v.add_argument('--lpips-cache')
 v=sp.add_parser('video');v.add_argument('--manifest',required=True);v.add_argument('--cameras',nargs='+',required=True);v.add_argument('--out',required=True);v.add_argument('--ffmpeg',default='ffmpeg')
 v=sp.add_parser('draft-model',help='Initial video modeling (interface reserved)');v.add_argument('--intake',required=True);v.add_argument('--out',required=True)
 v=sp.add_parser('spatial-fit',help='Optimize spatial relations from distance measurements (interface reserved)');v.add_argument('config');v.add_argument('--out',required=True)
 a=sub.add_parser('align',help='Real-to-sim alignment');ap=a.add_subparsers(dest='verb',required=True)
 v=ap.add_parser('calibrate',help='Fixed-camera PnP extrinsic calibration');v.add_argument('--scene',required=True);v.add_argument('--camera',required=True);v.add_argument('--points',required=True);v.add_argument('--out',required=True)
 v=ap.add_parser('fit-camera',help='Intrinsics unknown: joint focal length and pose fit');v.add_argument('config');v.add_argument('--out',required=True)
 v=ap.add_parser('base-check',help='Validate the scene_config base alignment fields');v.add_argument('config')
 t=sub.add_parser('traj',help='Trajectory generation');tp=t.add_subparsers(dest='verb',required=True)
 v=tp.add_parser('convert');v.add_argument('--input',required=True);v.add_argument('--calibration',required=True);v.add_argument('--out',required=True)
 v=tp.add_parser('xarm7');v.add_argument('job',choices=['validate_kinematics','simulate_replay','generate_grasp','gripper_mapping','import_eef','trajectory_adapter','plan_trajectory']);v.add_argument('--case',required=True);v.add_argument('--newton-project',help='Data-MechanicSim checkout; defaults to R2S_NEWTON_PROJECT, else the external/Data-MechanicSim submodule');v.add_argument('job_args',nargs=argparse.REMAINDER)
 c=sub.add_parser('tactile',help='Tactile integration');cp=c.add_subparsers(dest='verb',required=True)
 cp.add_parser('describe',help='Print the tactile contract')
 v=cp.add_parser('doctor',help='Check the Photon runtime item by item');v.add_argument('--python',default='python3',help='Target interpreter with tacsim dependencies')
 v=cp.add_parser('photon-render',help='Photon offline tactile render (synthetic stimulus)');v.add_argument('config');v.add_argument('--out',required=True);v.add_argument('--python',required=True,help='Existing Python 3.10 with tacsim deps; do not install/replace Newton')
 d=sub.add_parser('asset',help='Committed object asset library under assets/');dp=d.add_subparsers(dest='verb',required=True)
 v=dp.add_parser('list',help='List the objects in the library (a listing; `check` is the gate)')
 v=dp.add_parser('show',help='Print one object spec and its resolved geometry');v.add_argument('id')
 v=dp.add_parser('check',help='Validate every object; exit 2 on a bad asset')
 v=sub.add_parser('case-init');v.add_argument('case');v.add_argument('--cycles-python',help='Existing Python with bpy; defaults to R2S_CYCLES_PYTHON, else the submodule render_cycles environment')
 v=sub.add_parser('case-run');v.add_argument('case');v.add_argument('--retry-failed',action='store_true')
 v=sub.add_parser('case-status');v.add_argument('case')
 v=sub.add_parser('case-review');v.add_argument('case');v.add_argument('--stage',required=True);v.add_argument('--token',required=True);v.add_argument('--decision',choices=['approve','reject'],required=True);v.add_argument('--by',required=True);v.add_argument('--note',required=True)
 v=sub.add_parser('run');v.add_argument('plan');v.add_argument('--out',required=True)
 v=sub.add_parser('inventory');v.add_argument('root');v.add_argument('--out',required=True)
 v=sub.add_parser('check-inventory');v.add_argument('manifest')
 v=sub.add_parser('freeze');v.add_argument('--scene',required=True);v.add_argument('--blend',required=True);v.add_argument('--out',required=True)
 return p

def main(argv=None):
 argv=sys.argv[1:] if argv is None else list(argv)
 rewritten=normalize_command(argv)
 if rewritten!=list(argv):
  print('Deprecated command spelling: "r2s %s" is now "r2s %s %s"; the flat form will be removed in a future release'%(argv[0],rewritten[0],rewritten[1]),file=sys.stderr)
 a=build_parser().parse_args(rewritten)
 if a.group.startswith('case-') or a.group=='case-init':
  from . import cases
  if a.group=='case-init':
   cycles=a.cycles_python or default_cycles_python()
   if not cycles:raise SystemExit('Missing --cycles-python and R2S_CYCLES_PYTHON: run `uv sync` in external/Data-MechanicSim/render_cycles, or pass an existing Python with bpy')
   print(cases.init(a.case,cycles))
  elif a.group=='case-run':
   result=cases.run(a.case,a.retry_failed);print(json.dumps(result,ensure_ascii=False,indent=2));sys.exit(0 if result['status']=='complete' else 2)
  elif a.group=='case-status':print(json.dumps(load(pathlib.Path(a.case)/'runs/status.json'),ensure_ascii=False,indent=2))
  else:cases.review(a.case,a.stage,a.token,a.decision,a.by,a.note)
  return
 if a.group=='scene':
  if a.verb=='validate':
   s=validate_scene(load(a.scene));print(json.dumps({'valid':True,'profiles':{c['id']:profile_id(c) for c in s['cameras']}}));return
  if a.verb in ['build','render','audit']:
   validate_scene(load(a.scene));worker=pathlib.Path(__file__).parent/'workers/blender.py';args=[a.verb,'--scene',str(pathlib.Path(a.scene).resolve()),'--out',str(pathlib.Path(a.out).resolve()),'--samples',str(a.samples)]
   for k in ['blend','states','parameters']:
    if getattr(a,k):args+=['--'+k,str(pathlib.Path(getattr(a,k)).resolve())]
   if a.mcp:
    from .mcp import execute
    host,port=a.mcp.rsplit(':',1);code='import runpy,sys\nsys.argv='+repr([str(worker),*args])+'\nrunpy.run_path('+repr(str(worker))+',run_name="__main__")';print(execute(code,host,int(port)))
   else:subprocess.run([a.python,str(worker),*args],check=True)
   return
  if a.verb in ['scan-register','scan-inspect','scan-bake']:
   from .scene.scans import register,inspect,bake
   if a.verb=='scan-register':register(a.config,a.out)
   elif a.verb=='scan-inspect':inspect(a.obj,a.texture,a.out)
   else:
    bake(a.config,a.out);subprocess.run([a.python,str(pathlib.Path(__file__).parent/'workers/scan_asset.py'),str(pathlib.Path(a.out).resolve())],check=True)
   return
  if a.verb=='fit-appearance':
   from .scene.appearance import fit
   fit(a.config,a.out);return
  if a.verb=='score':
   from .scene.metrics import score
   save(a.out,score(a.real,a.sim,a.mask,a.lpips_cache));return
  if a.verb=='video':
   from .scene.video import encode
   encode(a.manifest,a.cameras,a.out,a.ffmpeg);return
  if a.verb=='draft-model':
   from .scene.modeling import interface
   print(json.dumps({'status':'reserved_not_implemented','contract':interface.describe()},ensure_ascii=False,indent=2));sys.exit(2)
  if a.verb=='spatial-fit':
   from .scene import spatial
   cfg=load(a.config);scene=validate_scene(load((pathlib.Path(a.config).resolve().parent/cfg['scene']).resolve()))
   ids={x['id'] for x in scene['assets']}|{c['id'] for c in scene['cameras']}
   constraints=spatial.validate_constraints(cfg['constraints'],ids)
   print(json.dumps({'status':'reserved_not_implemented','constraints_valid':True,'constraints':constraints,'detail':'The distance optimizer is a reserved interface; constraint validation passed'},ensure_ascii=False,indent=2));sys.exit(2)
 if a.group=='align':
  if a.verb=='calibrate':
   from .align.cameras import fit_extrinsics
   s=validate_scene(load(a.scene));cam=next(c for c in s['cameras'] if c['id']==a.camera);T,report=fit_extrinsics(cam,load(a.points));cam['T_world_optical']=T.tolist();cam['quality']='image_fitted';cam['evidence']=str(pathlib.Path(a.out).with_suffix('.calibration.json'));save(a.out,s);save(cam['evidence'],report);return
  if a.verb=='fit-camera':
   from .align.camera_fit import fit
   fit(a.config,a.out);return
  if a.verb=='base-check':
   from .align.base import validate_scene_config
   print(json.dumps(validate_scene_config(load(a.config)),ensure_ascii=False,indent=2));return
 if a.group=='traj':
  if a.verb=='convert':
   import numpy as np
   from .traj.trajectory import convert_episode
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
   return
  if a.verb=='xarm7':
   project_root=a.newton_project or default_newton_project()
   if not project_root:raise SystemExit('Missing --newton-project and R2S_NEWTON_PROJECT: pass the Data-MechanicSim checkout, or clone with --recursive so external/Data-MechanicSim is populated')
   root=pathlib.Path(a.case).resolve();project=pathlib.Path(project_root).resolve();s=load(root/'scene_config.json')
   if s.get('tcp_offset_m')!=[0,0,.172]:raise ValueError('This validated physical adapter supports xArm7 G2 TCP172 only; implement/test a new robot adapter for other TCPs')
   env=os.environ.copy();env['R2S_CASE_ROOT']=str(root);env['R2S_NEWTON_PROJECT']=str(project)
   args=a.job_args[1:] if a.job_args[:1]==['--'] else a.job_args
   try:
    subprocess.run([str(project/'.venv/bin/python'),str(pathlib.Path(__file__).parent/'traj/adapters/xarm7'/(a.job+'.py')),*args],env=env,check=True)
   except subprocess.CalledProcessError as e:
    sys.exit(e.returncode)  # pass the adapter exit code through (e.g. 2=Infeasible from plan_trajectory)
   return
 if a.group=='tactile':
  if a.verb=='describe':
   from .tactile import interface
   print(json.dumps(interface.describe(),ensure_ascii=False,indent=2));return
  if a.verb=='doctor':
   from .tactile import photon
   report=photon.check_photon_runtime(a.python);print(json.dumps(report,ensure_ascii=False,indent=2));sys.exit(0 if report['ready'] else 2)
  if a.verb=='photon-render':
   from .tactile import photon
   cfg=photon.validate_render_config(load(a.config))
   out=pathlib.Path(a.out).resolve()
   if out.exists():raise FileExistsError('A run directory must be new: '+str(out))
   # No env injection: the worker must resolve tacsim from --python itself, so that the
   # interpreter is the single source of truth for which checkout runs.
   worker=pathlib.Path(__file__).parent/'tactile/photon_worker.py'
   subprocess.run([a.python,str(worker),str(pathlib.Path(a.config).resolve()),str(out)],check=True)
   return
 if a.group=='asset':
  from . import assets
  if a.verb=='list':
   report=assets.validate_library();print(json.dumps({'root':report['root'],'objects':report['objects'],'errors':report['errors']},ensure_ascii=False,indent=2));return
  if a.verb=='show':
   directory=assets.object_dir(assets.library_root(),a.id)
   if not (directory/'asset.json').is_file():raise SystemExit('No such object asset: '+a.id)
   doc=assets.load_asset(directory/'asset.json',directory);out={'asset':doc,'directory':str(directory),'mesh':None}
   if doc['geometry']['kind']=='mesh':out['mesh']=str(assets.mesh_path(doc,directory,must_exist=True))
   print(json.dumps(out,ensure_ascii=False,indent=2));return
  report=assets.validate_library();print(json.dumps(report,ensure_ascii=False,indent=2));sys.exit(0 if report['passed'] else 2)
 if a.group=='run':
  from .runner import run
  run(a.plan,a.out);return
 if a.group=='inventory':
  from .artifacts import inventory
  print(inventory(a.root,a.out));return
 if a.group=='check-inventory':
  from .artifacts import check_inventory
  report=check_inventory(a.manifest);print(json.dumps(report));sys.exit(0 if report['passed'] else 2)
 if a.group=='freeze':
  from .artifacts import freeze
  validate_scene(load(a.scene));freeze(a.scene,a.blend,a.out);return
if __name__=='__main__':main()
