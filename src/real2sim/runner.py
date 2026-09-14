"""Declarative CLI DAG runner; no shell command strings and no hardware plugins."""
import pathlib,subprocess,sys,datetime,json,re
from .contracts import load,save,sha
ALLOWED={'validate','inventory','check-inventory','build','render','audit','calibrate','score','convert','freeze','video','fit-appearance','scan-inspect','scan-register','scan-bake','fit-camera'}
def run(plan_path,output):
 plan_path=pathlib.Path(plan_path).resolve();plan=load(plan_path);out=pathlib.Path(output).resolve()
 if out.exists():raise FileExistsError('A run directory must be new')
 if set(plan)!={'schema_version','variables','stages'} or plan['schema_version']!='1.0':raise ValueError('Invalid pipeline plan')
 out.mkdir(parents=True);variables={**plan['variables'],'run':str(out),'plan_dir':str(plan_path.parent)};done={};receipts=[]
 for stage in plan['stages']:
  if set(stage)!={'id','requires','argv','outputs'} or not re.fullmatch('[A-Za-z][A-Za-z0-9_-]*',stage['id']) or stage['id'] in done:raise ValueError('Invalid/duplicate stage')
  if not all(done.get(k) for k in stage['requires']):raise ValueError('Dependency missing or failed')
  args=[str(v).format_map(variables) for v in stage['argv']]
  if not args or args[0] not in ALLOWED:raise ValueError('Unapproved pipeline command')
  if '--out' in args and not pathlib.Path(args[args.index('--out')+1]).resolve().is_relative_to(out):raise ValueError('Command output escapes run directory')
  outputs=[pathlib.Path(str(v).format_map(variables)).resolve() for v in stage['outputs']]
  if any(not p.is_relative_to(out) for p in outputs):raise ValueError('Declared output escapes run directory')
  inputs={str(pathlib.Path(v).resolve()):sha(v) for v in args if pathlib.Path(v).is_file()}
  log=out/(stage['id']+'.log')
  with log.open('w') as f:result=subprocess.run([sys.executable,'-m','real2sim.cli',*args],cwd=plan_path.parent,stdout=f,stderr=subprocess.STDOUT)
  passed=result.returncode==0 and all(p.exists() for p in outputs)
  receipt={'stage':stage['id'],'argv':args,'input_hashes':inputs,'returncode':result.returncode,'outputs':[str(p) for p in outputs],'passed':passed,'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()};receipts.append(receipt);save(out/'pipeline_receipts.json',receipts);done[stage['id']]=passed
  if not passed:raise RuntimeError('Stage failed: '+stage['id']+'; see '+str(log))
 return receipts
