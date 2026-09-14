"""Verify a project snapshot and add missing files, preserving all server conflicts."""
import argparse,hashlib,json,pathlib,shutil,tarfile
p=argparse.ArgumentParser();p.add_argument('--bundle',required=True);p.add_argument('--cases-root',required=True);a=p.parse_args();b=pathlib.Path(a.bundle).resolve();root=pathlib.Path(a.cases_root).resolve()
def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
expected=(b/'archive.sha256').read_text().split()[0];assert sha(b/'local-project.tar.gz')==expected,'Archive hash mismatch'
dest=b/'snapshot'
if not dest.exists():
 dest.mkdir()
 with tarfile.open(b/'local-project.tar.gz') as tf:tf.extractall(dest,filter='data')
m=json.loads((b/'manifest.json').read_text());bad=[]
for row in m['files']:
 f=(dest/row['archive_path']).resolve()
 if not f.is_relative_to(dest) or not f.is_file() or f.stat().st_size!=row['size'] or sha(f)!=row['sha256']:bad.append(row['archive_path'])
if bad:raise RuntimeError('Snapshot verification failed: '+str(bad))
result={'files_verified':len(m['files']),'bytes_verified':m['total_bytes'],'archive_sha256':expected,'copied':[],'identical':[],'conflicts_preserved':[],'snapshot_only':[],'missing_original_paths':m['missing_previously_supplied_paths']}
for row in m['files']:
 rel=pathlib.PurePosixPath(row['archive_path']);parts=rel.parts
 if parts[0]=='LabSceneNewtonV1':
  tail=list(parts[1:])
  if tail[0]=='Baseline':tail[0]='baseline'
  target=root.joinpath(*tail)
 elif parts[0]=='supplied-originals':target=root/'source_inputs'/'supplied-originals'/pathlib.Path(*parts[1:])
 elif parts[0]=='desktop-workspace':target=root/'source_inputs'/'desktop-workspace'/pathlib.Path(*parts[1:])
 else:result['snapshot_only'].append(row['archive_path']);continue
 if not target.resolve().is_relative_to(root):raise ValueError('Target escaped case root')
 if target.exists():
  if target.is_file() and sha(target)==row['sha256']:result['identical'].append(str(target.relative_to(root)))
  else:result['conflicts_preserved'].append({'server':str(target),'local_snapshot':str(dest/rel),'local_sha256':row['sha256']})
 else:
  target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(dest/rel,target);assert sha(target)==row['sha256'];result['copied'].append(str(target.relative_to(root)))
(b/'verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
print(json.dumps({k:len(v) if isinstance(v,list) else v for k,v in result.items()},indent=2))
