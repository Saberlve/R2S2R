"""Synchronized equal-size camera mosaic, without silently changing time or aspect."""
import pathlib,subprocess,numpy as np
from PIL import Image,ImageDraw
from .contracts import load,save,sha

def frame_groups(manifest,cameras):
 if not cameras or len(set(cameras))!=len(cameras):raise ValueError('Need ordered unique camera ids')
 rows={}
 for row in manifest['frames']:
  if row['camera_id'] in cameras:
   key=row['frame_id'];group=rows.setdefault(key,{})
   if row['camera_id'] in group:raise ValueError('Duplicate camera/frame')
   group[row['camera_id']]=row
 groups=[rows[k] for k in sorted(rows)]
 if len(groups)<2 or any(set(g)!=set(cameras) for g in groups):raise ValueError('Need at least two complete synchronized frames')
 times=np.array([g[cameras[0]]['time_s'] for g in groups],float)
 if not np.isfinite(times).all() or not np.all(np.diff(times)>0) or not np.allclose(np.diff(times),np.diff(times)[0],rtol=1e-5,atol=1e-8):raise ValueError('Video requires uniform increasing timestamps; explicitly resample physics states first')
 wh=groups[0][cameras[0]]['width_height']
 for g,t in zip(groups,times):
  if any(abs(g[c]['time_s']-t)>1e-8 or g[c]['width_height']!=wh for c in cameras):raise ValueError('Camera times/sizes differ; no implicit resize')
 for c in cameras:
  if len({g[c]['profile_id'] for g in groups})!=1:raise ValueError('Camera profile changed during video')
 return groups,wh,float(1/np.diff(times)[0])

def encode(manifest_file,cameras,output,ffmpeg='ffmpeg'):
 path=pathlib.Path(manifest_file).resolve();out=pathlib.Path(output).resolve();groups,wh,fps=frame_groups(load(path),cameras)
 if out.exists():raise FileExistsError(out)
 w,h=wh;w*=len(cameras)
 if w%2 or h%2:raise ValueError('H264 yuv420p requires even mosaic dimensions')
 out.parent.mkdir(parents=True,exist_ok=True)
 command=[ffmpeg,'-hide_banner','-loglevel','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r',str(fps),'-i','-','-an','-c:v','libx264','-crf','18','-pix_fmt','yuv420p',str(out)]
 proc=subprocess.Popen(command,stdin=subprocess.PIPE)
 try:
  for group in groups:
   mosaic=Image.new('RGB',(w,h))
   for i,c in enumerate(cameras):
    file=(path.parent/group[c]['rgb']).resolve()
    if not file.is_relative_to(path.parent):raise ValueError('Image path escapes render directory')
    with Image.open(file) as im:
     if list(im.size)!=wh:raise ValueError('Actual image size differs from manifest')
     mosaic.paste(im.convert('RGB'),(i*wh[0],0))
   proc.stdin.write(mosaic.tobytes())
  proc.stdin.close()
  if proc.wait()!=0:raise RuntimeError('ffmpeg failed')
 except BaseException:
  proc.kill();proc.wait();raise
 save(out.with_suffix('.video.json'),{'schema_version':'1.0','source_manifest_sha256':sha(path),'camera_order':cameras,'frames':len(groups),'fps':fps,'width_height':[w,h],'resampling':'none','codec':'h264 yuv420p','note':'Mosaic pixels are concatenated; compression is lossy.'})
