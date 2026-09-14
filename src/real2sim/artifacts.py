import pathlib,shutil,datetime,json
from .contracts import save,load,sha

def inventory(root,output):
 root=pathlib.Path(root).resolve();output=pathlib.Path(output).resolve()
 if output.is_relative_to(root):raise ValueError('Inventory output must be outside immutable input root')
 rows=[]
 for p in sorted(root.rglob('*')):
  if p.is_symlink():raise ValueError('Raw inventory rejects symlinks: '+str(p))
  if p.is_file():rows.append({'path':str(p.relative_to(root)),'bytes':p.stat().st_size,'sha256':sha(p)})
 save(output,{'schema_version':'1.0','root':str(root),'files':rows});return len(rows)
def check_inventory(manifest):
 j=load(manifest);root=pathlib.Path(j['root']);bad=[]
 for row in j['files']:
  p=root/row['path']
  if not p.is_file() or sha(p)!=row['sha256']:bad.append(row['path'])
 return {'passed':not bad,'checked':len(j['files']),'changed_or_missing':bad}
def freeze(scene,blend,destination):
 dst=pathlib.Path(destination)
 if dst.exists():raise FileExistsError('Freeze never overwrites a baseline')
 dst.mkdir(parents=True)
 shutil.copy2(scene,dst/'scene.json');shutil.copy2(blend,dst/'scene.blend')
 save(dst/'manifest.json',{'schema_version':'1.0','created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'files':{p.name:sha(p) for p in [dst/'scene.json',dst/'scene.blend']},'scope':'visual artifact snapshot; does not certify calibration or contact physics'})
