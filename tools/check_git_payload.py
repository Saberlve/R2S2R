"""Fail if tracked Git history contains resource binaries or oversized blobs."""
import pathlib,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1]
allowed={'examples/lab_reference/scanner/door_erase_mask.png'}
blocked={'.png','.jpg','.jpeg','.mp4','.mov','.heic','.blend','.blend1','.npz','.npy','.zip','.glb','.gltf','.obj','.stl','.ply','.parquet','.pt','.pth','.onnx'}
objects=subprocess.check_output(['git','-C',str(root),'rev-list','--objects','--all'],text=True)
rows=subprocess.run(['git','-C',str(root),'cat-file','--batch-check=%(objecttype) %(objectsize) %(rest)'],input=objects,capture_output=True,text=True,check=True).stdout.splitlines()
bad=[];count=0;maximum=0
for row in rows:
 typ,size,*tail=row.split(' ',2)
 if typ!='blob':continue
 name=tail[0] if tail else '';size=int(size);count+=1;maximum=max(maximum,size)
 if size>5_000_000 or (pathlib.Path(name).suffix.lower() in blocked and name not in allowed):bad.append(name)
print({'checked_blobs':count,'largest_blob_bytes':maximum,'forbidden_files':bad})
sys.exit(1 if bad else 0)
