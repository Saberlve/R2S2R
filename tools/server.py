"""Headless research entry points.

Runtime paths come from the R2S_* environment variables (see examples/site.example.env).
This tool never contacts hardware.
"""
import argparse,datetime,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
SITE_ENV={'main_python':'R2S_MAIN_PYTHON','cycles_python':'R2S_CYCLES_PYTHON','ffmpeg':'R2S_FFMPEG','newton_project':'R2S_NEWTON_PROJECT','measured_case':'R2S_MEASURED_CASE','reference_template':'R2S_REFERENCE_TEMPLATE','runs_root':'R2S_RUNS_ROOT'}
def load(path):return json.loads(pathlib.Path(path).read_text())
def save(path,value):pathlib.Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False)+"\n")
def site_from_env(environ):
    missing=[name for name in SITE_ENV.values() if not environ.get(name)]
    if missing:raise SystemExit('Missing environment variables: '+', '.join(missing)+'\nSet the runtime paths for this machine; see examples/site.example.env')
    return {key:environ[name] for key,name in SITE_ENV.items()}
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
    env=os.environ.copy();env.update(PYTHONPATH=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH',''),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='8')
    if getattr(a,'gpu',None) is not None:env['CUDA_VISIBLE_DEVICES']=a.gpu
    elif a.command in ['render','smoke']:env['CUDA_VISIBLE_DEVICES']=''
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');out=pathlib.Path(a.out).resolve() if a.out else pathlib.Path(site['runs_root'])/(a.command+'_'+stamp);out.mkdir(parents=True,exist_ok=False)
    receipt={'command':a.command,'arguments':vars(a),'site':site,'out':str(out),'status':'running','started_utc':stamp};save(out/'receipt.json',receipt)
    py=site['main_python'];bpy=site['cycles_python'];template=pathlib.Path(getattr(a,'template',None) or site['reference_template']).resolve()
    try:
        if a.command=='doctor':
            checks=[]
            for label,exe,mods in [('main',py,['numpy','scipy','jsonschema','cv2','newton','warp','mujoco']),('cycles',bpy,['bpy','numpy','PIL'])]:
                code="import importlib,json,sys; names="+repr(mods)+"; print(json.dumps({'python':sys.version,'modules':{n:{'version':str(getattr(importlib.import_module(n),'__version__',getattr(getattr(importlib.import_module(n),'app',None),'version_string','unknown'))),'path':str(getattr(importlib.import_module(n),'__file__',''))} for n in names}}))"
                r=subprocess.run([exe,'-c',code],env=env,text=True,capture_output=True);(out/(label+'.log')).write_text(r.stdout+r.stderr);checks.append({'runtime':label,'passed':r.returncode==0})
            ff=subprocess.run([site['ffmpeg'],'-version'],capture_output=True,text=True);(out/'ffmpeg.log').write_text(ff.stdout+ff.stderr);checks.append({'runtime':'ffmpeg','passed':ff.returncode==0})
            for key in ['reference_template','measured_case','newton_project']:checks.append({'path':key,'passed':pathlib.Path(site[key]).exists()})
            save(out/'checks.json',checks)
            if not all(x['passed'] for x in checks):raise RuntimeError('Environment check failed; see logs')
        elif a.command=='smoke':
            scene=ROOT/'examples/minimal/scene.json';base=[py,'-m','real2sim.cli'];run(base+['scene','validate',scene],env,out/'validate.log');run(base+['scene','build','--scene',scene,'--out',out/'scene.blend','--python',bpy],env,out/'build.log');run(base+['scene','render','--scene',scene,'--blend',out/'scene.blend','--out',out/'render','--python',bpy,'--samples',a.samples],env,out/'render.log')
        elif a.command in ['inspect','edit']:
            args=[bpy,ROOT/'tools/server_scene.py',a.command,'--template',template,'--out',out]
            if a.command=='edit':args+=['--patch',pathlib.Path(a.patch).resolve()]
            run(args,env,out/'worker.log')
        elif a.command=='render':
            dest=out/'template';copy_template(template,dest)
            args=[bpy,dest/'render.py','--root',dest,'--samples',a.samples,'--device','GPU' if a.gpu is not None else 'CPU']
            if a.states:args+=['--states',pathlib.Path(a.states).resolve()]
            if a.limit:args+=['--limit',a.limit]
            run(args,env,out/'render.log')
        elif a.command=='simulate':
            if a.frames<1 or a.stride<1:raise ValueError('frames and stride must be positive')
            if a.gpu is None:raise ValueError('Specify --gpu explicitly for Newton contact simulation')
            case=out/'case';case.mkdir();source=pathlib.Path(site['measured_case']);(case/'results').mkdir();(case/'reports').mkdir()
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
