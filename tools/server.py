"""Headless research entry points.

Runtime paths come from the R2S_* environment variables (see examples/site.example.env).
This tool never contacts hardware.
"""
import argparse,datetime,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
# Data-MechanicSim ships as a submodule, so a `git clone --recursive` already carries the Newton
# project -- and `uv sync` inside it builds both interpreters at fixed paths. The matching
# variables therefore only have to be set to point somewhere else.
SUBMODULE_NEWTON_PROJECT=ROOT/'external'/'Data-MechanicSim'
DEFAULT_MAIN_PYTHON=SUBMODULE_NEWTON_PROJECT/'.venv'/'bin/python'
DEFAULT_CYCLES_PYTHON=SUBMODULE_NEWTON_PROJECT/'render_cycles'/'.venv'/'bin/python'
SITE_ENV={'main_python':'R2S_MAIN_PYTHON','cycles_python':'R2S_CYCLES_PYTHON','ffmpeg':'R2S_FFMPEG','newton_project':'R2S_NEWTON_PROJECT','measured_case':'R2S_MEASURED_CASE','reference_template':'R2S_REFERENCE_TEMPLATE','runs_root':'R2S_RUNS_ROOT'}
# What each command cannot run without. `smoke` and `doctor` are deliberately empty: a fresh
# --recursive clone that has run `uv sync` twice must be able to check itself and render the
# minimal scene without any R2S_* variable set.
SITE_REQUIREMENTS={'doctor':[],'smoke':['main_python','cycles_python'],'inspect':['cycles_python','reference_template'],
 'edit':['cycles_python','reference_template'],'render':['cycles_python','reference_template'],
 'simulate':['main_python','cycles_python','ffmpeg','measured_case','reference_template','newton_project']}
# Printed when a path check fails, so `doctor` says where to get the thing rather than just
# reporting that it is absent.
PATH_HINTS={'reference_template':'R2S_REFERENCE_TEMPLATE: the accepted scene runtime directory; see docs/SERVER_GUIDE.md',
 'measured_case':'R2S_MEASURED_CASE: the measured bar-grasp case; see docs/RESOURCES.md',
 'newton_project':'R2S_NEWTON_PROJECT: a Data-MechanicSim checkout (this also provides tacsim); see docs/DEPENDENCIES.md'}
def load(path):return json.loads(pathlib.Path(path).read_text())
def save(path,value):pathlib.Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False)+"\n")
def newton_project_is_populated(path):
    # A clone made without --recursive leaves an empty directory behind, which would pass a bare
    # exists() check while `newton_gen` stays unimportable.
    return (pathlib.Path(path)/'newton_gen').is_dir()
def site_defaults():
    """Paths a prepared clone already satisfies, so they need no environment variable.

    The interpreters are checked for existence: a clone that never ran `uv sync` reports the
    variable as missing instead of being handed a path to nothing. The lab's own case data has no
    entry here -- no default can invent it.
    """
    return {'main_python':str(DEFAULT_MAIN_PYTHON) if DEFAULT_MAIN_PYTHON.is_file() else None,
            'cycles_python':str(DEFAULT_CYCLES_PYTHON) if DEFAULT_CYCLES_PYTHON.is_file() else None,
            'newton_project':str(SUBMODULE_NEWTON_PROJECT) if newton_project_is_populated(SUBMODULE_NEWTON_PROJECT) else None,
            'runs_root':str(ROOT/'runs'),
            'ffmpeg':shutil.which('ffmpeg')}
def site_from_env(environ):
    defaults=site_defaults()
    # An explicit variable always wins, even when it points at something that does not exist:
    # `doctor` is where that gets reported.
    return {key:(environ.get(name) or defaults.get(key)) for key,name in SITE_ENV.items()}
def require_site(site,command):
    missing=[SITE_ENV[key] for key in SITE_REQUIREMENTS[command] if not site.get(key)]
    if not missing:return
    hint=''
    if not DEFAULT_MAIN_PYTHON.is_file() or not DEFAULT_CYCLES_PYTHON.is_file():
        hint+='\nBuild the two interpreters with `uv sync` in external/Data-MechanicSim and again in its render_cycles/; see docs/DEPENDENCIES.md.'
    if 'newton_project' in SITE_REQUIREMENTS[command] and SUBMODULE_NEWTON_PROJECT.is_dir():
        hint+='\nR2S_NEWTON_PROJECT: external/Data-MechanicSim exists but holds no newton_gen/; run `git submodule update --init --recursive`.'
    raise SystemExit('Missing environment variables: '+', '.join(missing)+'\nSet them or see examples/site.example.env'+hint)
def run(cmd,env,log=None):
    print("RUN", " ".join(map(str,cmd)),flush=True)
    if log:
        with pathlib.Path(log).open('w') as f:subprocess.run(list(map(str,cmd)),env=env,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
    else:subprocess.run(list(map(str,cmd)),env=env,cwd=ROOT,check=True)
def copy_template(source,dest):
    source=pathlib.Path(source)
    dest.mkdir(parents=True,exist_ok=False)
    for name in ['Baseline.blend','camera_config.json','runtime_manifest.json','render.py','render_api.py','appearance_config.json']:
        if (source/name).exists():shutil.copy2(source/name,dest/name)
    for name in ['Baseline.blend','camera_config.json','runtime_manifest.json','render.py']:
        if not (dest/name).is_file():raise FileNotFoundError(source/name)
    manifest=load(dest/'runtime_manifest.json')
    if manifest.get('wrist_calibration'):
        cal=(source/manifest['wrist_calibration']).resolve()
        if not cal.is_file():raise FileNotFoundError(cal)
        shutil.copy2(cal,dest/'wrist_calibration.json');manifest['wrist_calibration']='wrist_calibration.json';save(dest/'runtime_manifest.json',manifest)
def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='command',required=True)
    sub.add_parser('site',help='Print the runtime paths taken from the environment')
    for name in ['doctor','smoke','inspect','edit','render','simulate']:
        p=sub.add_parser(name);p.add_argument('--out',help='New output directory; generated under runs_root if omitted')
        if name in ['inspect','edit','render','simulate']:p.add_argument('--template',help='Runtime directory; defaults to R2S_REFERENCE_TEMPLATE')
        if name=='edit':p.add_argument('--patch',required=True)
        if name in ['smoke','render','simulate']:p.add_argument('--samples',type=int,default=16);p.add_argument('--gpu',default=None,help='Explicit CUDA index; CPU used by render/smoke if omitted')
        if name=='render':p.add_argument('--states');p.add_argument('--limit',type=int,default=0)
        if name=='simulate':
            p.add_argument('--frames',type=int,default=481);p.add_argument('--engine',choices=['mujoco_fast','mujoco'],default='mujoco_fast');p.add_argument('--stride',type=int,default=15);p.add_argument('--no-render',action='store_true')
    a=ap.parse_args();site=site_from_env(os.environ)
    if a.command=='site':print(json.dumps(site,indent=2,ensure_ascii=False));return
    # Checked before any output directory exists, so a missing path never leaves a half-made run.
    require_site(site,a.command)
    env=os.environ.copy();env.update(PYTHONPATH=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH',''),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='8')
    if getattr(a,'gpu',None) is not None:env['CUDA_VISIBLE_DEVICES']=a.gpu
    elif a.command in ['render','smoke']:env['CUDA_VISIBLE_DEVICES']=''
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');out=pathlib.Path(a.out).resolve() if a.out else pathlib.Path(site['runs_root'])/(a.command+'_'+stamp);out.mkdir(parents=True,exist_ok=False)
    receipt={'command':a.command,'arguments':vars(a),'site':site,'out':str(out),'status':'running','started_utc':stamp};save(out/'receipt.json',receipt)
    py=site['main_python'];bpy=site['cycles_python']
    def template_path():
        # Resolved where it is used: `smoke` has no template and must run with no lab path set at all.
        return pathlib.Path(getattr(a,'template',None) or site['reference_template']).resolve()
    try:
        if a.command=='doctor':
            checks=[]
            for label,exe,mods in [('main',py,['numpy','scipy','jsonschema','cv2','newton','warp','mujoco']),('cycles',bpy,['bpy','numpy','PIL'])]:
                if not exe or not pathlib.Path(exe).is_file():
                    where='external/Data-MechanicSim' if label=='main' else 'external/Data-MechanicSim/render_cycles'
                    checks.append({'runtime':label,'passed':False,'hint':'R2S_'+label.upper()+'_PYTHON is unset and no built environment was found; run `uv sync` in '+where+' (see docs/DEPENDENCIES.md)'})
                    continue
                code="import importlib,json,sys; names="+repr(mods)+"; print(json.dumps({'python':sys.version,'modules':{n:{'version':str(getattr(importlib.import_module(n),'__version__',getattr(getattr(importlib.import_module(n),'app',None),'version_string','unknown'))),'path':str(getattr(importlib.import_module(n),'__file__',''))} for n in names}}))"
                r=subprocess.run([exe,'-c',code],env=env,text=True,capture_output=True);(out/(label+'.log')).write_text(r.stdout+r.stderr);checks.append({'runtime':label,'passed':r.returncode==0})
            if site['ffmpeg']:
                ff=subprocess.run([site['ffmpeg'],'-version'],capture_output=True,text=True);(out/'ffmpeg.log').write_text(ff.stdout+ff.stderr);checks.append({'runtime':'ffmpeg','passed':ff.returncode==0})
            else:checks.append({'runtime':'ffmpeg','passed':False,'hint':'R2S_FFMPEG is unset and no ffmpeg was found on PATH; install it or point the variable at one'})
            for key in ['reference_template','measured_case','newton_project']:
                value=site.get(key);ok=bool(value) and pathlib.Path(value).exists()
                # An uninitialized submodule is a directory that exists but holds nothing; without
                # this the check would pass and `newton_gen` would fail later.
                if ok and key=='newton_project':ok=newton_project_is_populated(value)
                # A path the environment never claimed is reported but is not fatal -- only the
                # lab-specific commands need it. A path that IS set but unusable is a real failure.
                check={'path':key,'passed':ok,'critical':bool(value)}
                if not ok:
                    check['hint']=PATH_HINTS[key]
                    if key=='newton_project' and value and pathlib.Path(value).is_dir():check['hint']+=' The directory exists but has no newton_gen/; the submodule is probably uninitialized.'
                checks.append(check)
            # tacsim is reported but never critical: the tactile backend has its own entry point
            # (`r2s tactile doctor`) and no scene/Cycles command depends on it.
            if py and pathlib.Path(py).is_file():
                code="import importlib.util,json;s=importlib.util.find_spec('tacsim');print(json.dumps({'tacsim':getattr(s,'origin',None)}))"
                r=subprocess.run([py,'-c',code],env=env,text=True,capture_output=True);(out/'tacsim.log').write_text(r.stdout+r.stderr)
                checks.append({'runtime':'tacsim','passed':r.returncode==0 and 'null' not in r.stdout,'critical':False})
            else:checks.append({'runtime':'tacsim','passed':False,'critical':False,'hint':'No main interpreter available to probe tacsim resolution'})
            save(out/'checks.json',checks)
            if not all(x['passed'] for x in checks if x.get('critical',True)):raise RuntimeError('Environment check failed; see logs')
        elif a.command=='smoke':
            scene=ROOT/'examples/minimal/scene.json';base=[py,'-m','real2sim.cli'];run(base+['scene','validate',scene],env,out/'validate.log');run(base+['scene','build','--scene',scene,'--out',out/'scene.blend','--python',bpy],env,out/'build.log');run(base+['scene','render','--scene',scene,'--blend',out/'scene.blend','--out',out/'render','--python',bpy,'--samples',a.samples],env,out/'render.log')
        elif a.command in ['inspect','edit']:
            args=[bpy,ROOT/'tools/server_scene.py',a.command,'--template',template_path(),'--out',out]
            if a.command=='edit':args+=['--patch',pathlib.Path(a.patch).resolve()]
            run(args,env,out/'worker.log')
        elif a.command=='render':
            dest=out/'template';copy_template(template_path(),dest)
            args=[bpy,dest/'render.py','--root',dest,'--samples',a.samples,'--device','GPU' if a.gpu is not None else 'CPU']
            if a.states:args+=['--states',pathlib.Path(a.states).resolve()]
            if a.limit:args+=['--limit',a.limit]
            run(args,env,out/'render.log')
        elif a.command=='simulate':
            if a.frames<1 or a.stride<1:raise ValueError('frames and stride must be positive')
            if a.gpu is None:raise ValueError('Specify --gpu explicitly for Newton contact simulation')
            case=out/'case';case.mkdir();source=pathlib.Path(site['measured_case']);template=template_path();(case/'results').mkdir();(case/'reports').mkdir()
            for name in ['inputs','src']:shutil.copytree(source/name,case/name)
            shutil.copy2(source/'scene_config.json',case/'scene_config.json');shutil.copy2(source/'reports/model_labels.json',case/'reports/model_labels.json')
            for name in ['episode000_ik.npz','grasp_clearance.npz']:shutil.copy2(source/'results'/name,case/'results'/name)
            # The old files are confined to this fresh run; copy accepted optical template over them.
            for name in ['Baseline.blend','camera_config.json','runtime_manifest.json']:shutil.copy2(template/name,case/'inputs/render_baseline'/name)
            rm=load(case/'inputs/render_baseline/runtime_manifest.json')
            if rm.get('wrist_calibration'):
                cal=(template/rm['wrist_calibration']).resolve()
                if not cal.is_file():raise FileNotFoundError(cal)
                shutil.copy2(cal,case/'inputs/render_baseline/wrist_calibration.json');rm['wrist_calibration']='wrist_calibration.json';save(case/'inputs/render_baseline/runtime_manifest.json',rm)
            env.update(R2S_CASE_ROOT=str(case),R2S_NEWTON_PROJECT=site['newton_project'])
            run([py,case/'src/simulate_replay.py','--engine',a.engine,'--bar','--frames',a.frames,'--tag','server','--trajectory',case/'results/grasp_clearance.npz'],env,out/'simulation.log')
            sim=case/'results'/('eef_'+a.engine+'_bar_server')
            if not a.no_render:
                run([bpy,case/'src/render_motion.py','--run',sim,'--stride',a.stride,'--samples',a.samples],env,out/'render.log')
                run([site['ffmpeg'],'-n','-hide_banner','-loglevel','error','-framerate',str(30/a.stride),'-i',sim/'render/three_views_%06d.png','-c:v','libx264','-crf','19','-pix_fmt','yuv420p',out/'ThreeViews.mp4'],env,out/'encode.log')
            receipt['simulation_report']=str(sim/'report.json');receipt['note']='Offline generated trajectory. Short frame runs are smoke tests, not grasp success or real robot validation.'
        receipt['status']='complete'
    except Exception as exc:
        receipt.update(status='failed',error=repr(exc));raise
    finally:save(out/'receipt.json',receipt)
    print('OUTPUT',out,flush=True)
if __name__=='__main__':main()
