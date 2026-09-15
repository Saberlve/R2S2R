"""Bounded full-render fitting. No camera/geometry fitting in this stage."""
import pathlib,subprocess,sys,json,numpy as np
from scipy.optimize import minimize
from PIL import Image
from ..contracts import load,save,profile_id,validate_scene
from .metrics import score

def fit(config_file,output):
 cfg=load(config_file);root=pathlib.Path(config_file).resolve().parent;out=pathlib.Path(output).resolve()
 if out.exists():raise FileExistsError(out)
 out.mkdir(parents=True);scene=(root/cfg['scene']).resolve();blend=(root/cfg['blend']).resolve();s=validate_scene(load(scene));profiles={c['id']:profile_id(c) for c in s['cameras']};params=cfg['parameters'];views=cfg['references']
 if not any(v['split']=='fit' for v in views) or not any(v['split']=='holdout' for v in views):raise ValueError('Need fit AND independent holdout references')
 if any(v['profile_id']!=profiles[v['camera_id']] for v in views):raise ValueError('Reference camera profile mismatch')
 if not params or len(set(keys_['key'] for keys_ in params))!=len(params):raise ValueError('Need nonempty unique parameter keys')
 if set((v['camera_id'],str((root/v['real']).resolve())) for v in views if v['split']=='fit') & set((v['camera_id'],str((root/v['real']).resolve())) for v in views if v['split']=='holdout'):raise ValueError('Fit and holdout must not reuse the same reference')
 for v in views:
  if v['split'] not in ['fit','holdout']:raise ValueError('Invalid split')
 keys=[v['key'] for v in params];bounds=[v['bounds'] for v in params];x0=[v['initial'] for v in params]
 for k,b,x in zip(keys,bounds,x0):
  pieces=k.split('/')
  if len(pieces)!=3 or (pieces[0],pieces[2]) not in [('light','power_W'),('material','albedo_gain')] or len(b)!=2 or not 0<=b[0]<b[1] or not b[0]<=x<=b[1]:raise ValueError('Invalid appearance parameter')
 history=[];best=None
 def evaluate(x):
  nonlocal best
  i=len(history);p=out/f'parameters_{i:04d}.json';save(p,dict(zip(keys,map(float,x))));render=out/f'render_{i:04d}'
  subprocess.run([sys.executable,'-m','real2sim.cli','scene','render','--scene',str(scene),'--blend',str(blend),'--out',str(render),'--parameters',str(p),'--python',cfg['cycles_python'],'--samples',str(cfg.get('samples',16))],check=True,stdout=subprocess.DEVNULL)
  scores=[]
  for v in views:
   mask=np.asarray(Image.open(root/v['mask']).convert('L'))>127;valid=np.asarray(Image.open(render/v['camera_id']/'000000_valid.png').convert('L'))>127
   if mask.shape!=valid.shape or np.any(mask & ~valid):raise ValueError('Evaluation mask must be a subset of rendered valid pixels; fix the common mask before fitting')
   score_=score(root/v['real'],render/v['camera_id']/'000000.png',root/v['mask']);scores.append({'camera_id':v['camera_id'],'split':v['split'],**score_})
  loss=float(np.mean([v['loss'] for v in scores if v['split']=='fit']));row={'evaluation':i,'fit_loss':loss,'parameters':dict(zip(keys,map(float,x))),'scores':scores};history.append(row)
  if best is None or loss<best['fit_loss']:best=row
  save(out/'history.json',history);return loss
 evaluate(x0)
 if cfg.get('max_evaluations',8)>1:minimize(evaluate,x0,method='Powell',bounds=bounds,options={'maxfev':cfg.get('max_evaluations',8)-1})
 save(out/'best.json',best);baseline=history[0];basehold=np.mean([v['loss'] for v in baseline['scores'] if v['split']=='holdout']);finalhold=np.mean([v['loss'] for v in best['scores'] if v['split']=='holdout'])
 save(out/'acceptance.json',{'fit_improved':best['fit_loss']<baseline['fit_loss'],'holdout_baseline':float(basehold),'holdout_final':float(finalhold),'holdout_not_degraded':bool(finalhold<=basehold+cfg.get('holdout_tolerance',0.)),'selected_using':'fit split only','full_cycles_render_each_candidate':True,'automatic_adoption':False})
