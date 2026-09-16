"""Fail if tracked Git history contains resource binaries or oversized blobs.

`assets/` is the single place a small mesh or texture may be committed. That exception is
scoped by path prefix, format and size so it cannot become a way to move case data into Git.

This scans every reachable object, not the worktree: a binary that enters a commit cannot be
taken back out of history, so relax the rule before the first asset lands, never after.
"""
import pathlib,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1]
allowed={'examples/lab_reference/scanner/door_erase_mask.png'}
blocked={'.png','.jpg','.jpeg','.mp4','.mov','.heic','.blend','.blend1','.npz','.npy','.zip','.glb','.gltf','.obj','.stl','.ply','.parquet','.pt','.pth','.onnx'}
asset_prefix='assets/'
asset_mesh_suffixes={'.glb','.gltf','.bin'}
asset_raster_suffixes={'.png','.jpg'}
asset_max_bytes=2_000_000
max_blob_bytes=5_000_000

def classify(name,size):
 """Return None when the blob may be committed, else the reason it may not.

 Rasters are confined to a `textures/` directory so a render cannot ride along as
 `assets/<id>/render.png`. These rules mirror the negations in .gitignore exactly.
 """
 if name in allowed:return None
 if size>max_blob_bytes:return 'blob larger than 5MB'
 suffix=pathlib.Path(name).suffix.lower()
 if name.startswith(asset_prefix):
  if suffix in asset_mesh_suffixes:return None if size<=asset_max_bytes else 'object asset larger than 2MB'
  if suffix in asset_raster_suffixes:
   if '/textures/' not in name:return 'object asset raster outside a textures/ directory'
   return None if size<=asset_max_bytes else 'object asset larger than 2MB'
  return 'unsupported object asset format' if suffix in blocked else None
 return 'resource binary' if suffix in blocked else None

def main():
 objects=subprocess.check_output(['git','-C',str(root),'rev-list','--objects','--all'],text=True)
 rows=subprocess.run(['git','-C',str(root),'cat-file','--batch-check=%(objecttype) %(objectsize) %(rest)'],input=objects,capture_output=True,text=True,check=True).stdout.splitlines()
 bad=[];count=0;maximum=0
 for row in rows:
  typ,size,*tail=row.split(' ',2)
  if typ!='blob':continue
  name=tail[0] if tail else '';size=int(size);count+=1;maximum=max(maximum,size)
  reason=classify(name,size)
  if reason:bad.append({'path':name,'reason':reason})
 print({'checked_blobs':count,'largest_blob_bytes':maximum,'forbidden_files':[b['path'] for b in bad],'forbidden_reasons':{b['path']:b['reason'] for b in bad}})
 return 1 if bad else 0

if __name__=='__main__':sys.exit(main())
