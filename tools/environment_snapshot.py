"""Record/check the exact external source tree used by the robot adapter."""
import argparse,pathlib,json,hashlib,sys,subprocess,importlib.metadata
p=argparse.ArgumentParser();p.add_argument('--project');p.add_argument('--out');p.add_argument('--check');a=p.parse_args()
def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def source_files(root):
 return sorted(f for sub in ['newton_gen','third_party/newton/newton'] for f in (root/sub).rglob('*.py') if not any(part in ['.venv','venv','__pycache__','.git'] for part in f.relative_to(root).parts))
if a.check:
 d=json.loads(pathlib.Path(a.check).read_text());root=pathlib.Path(d['project']);bad=[n for n,h in d['source_sha256'].items() if not (root/n).is_file() or digest(root/n)!=h]
 current={str(f.relative_to(root)) for f in source_files(root)};added=sorted(current-set(d['source_sha256']))
 print(json.dumps({'source_tree_matches':not bad and not added,'changed_or_missing':bad,'added':added,'scope':'Source files only; compare recorded package versions separately.'}));sys.exit(1 if bad or added else 0)
if not a.project or not a.out:raise ValueError('Need --project and --out, or --check')
root=pathlib.Path(a.project).resolve();out=pathlib.Path(a.out)
if out.exists():raise FileExistsError(out)
versions={}
for name in ['numpy','scipy','warp-lang','newton','mujoco','torch','opencv-python','jsonschema','pillow','scikit-image']:
 try:versions[name]=importlib.metadata.version(name)
 except importlib.metadata.PackageNotFoundError:versions[name]=None
commits={}
for name,folder in [('project',root),('newton',root/'third_party/newton')]:
 commits[name]={'head':subprocess.run(['git','-C',str(folder),'rev-parse','HEAD'],capture_output=True,text=True,check=True).stdout.strip(),'dirty_paths':subprocess.run(['git','-C',str(folder),'status','--porcelain'],capture_output=True,text=True,check=True).stdout.splitlines()}
d={'schema_version':'1.0','python':sys.version,'executable':sys.executable,'project':str(root),'packages':versions,'git':commits,'source_sha256':{str(f.relative_to(root)):digest(f) for f in source_files(root)},'note':'Working-tree hashes are authoritative; a dirty checkout is not reproduced by checking out HEAD alone. Third-party code is not bundled here.'}
out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(d,indent=2)+'\n');print('Recorded',len(d['source_sha256']),'source files')
