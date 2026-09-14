"""Operator-run sensor capture. Dry-run unless --capture; never touches a robot."""
import argparse,pathlib,json,time
p=argparse.ArgumentParser();p.add_argument('--serial',required=True);p.add_argument('--width',type=int,required=True);p.add_argument('--height',type=int,required=True);p.add_argument('--fps',type=int,default=30);p.add_argument('--warmup-s',type=float,default=10);p.add_argument('--out',required=True);p.add_argument('--capture',action='store_true');a=p.parse_args()
if not a.capture:print(json.dumps({'dry_run':True,'serial':a.serial,'requested_rgb_profile':[a.width,a.height,a.fps],'warmup_s':a.warmup_s}));raise SystemExit(0)
if a.warmup_s<10:raise ValueError('Use at least 10 seconds settling for this workflow')
import pyrealsense2 as rs,numpy as np
from PIL import Image
out=pathlib.Path(a.out)
if out.exists():raise FileExistsError(out)
pipe=rs.pipeline();cfg=rs.config();cfg.enable_device(a.serial);cfg.enable_stream(rs.stream.color,a.width,a.height,rs.format.rgb8,a.fps);profile=pipe.start(cfg)
try:
 start=time.monotonic()
 while time.monotonic()-start<a.warmup_s:pipe.wait_for_frames()
 frame=pipe.wait_for_frames().get_color_frame();v=frame.profile.as_video_stream_profile();intr=v.get_intrinsics();device=profile.get_device();actual=device.get_info(rs.camera_info.serial_number)
 if actual!=a.serial or (intr.width,intr.height)!=(a.width,a.height):raise RuntimeError('Unexpected stream profile')
 metadata={}
 for key in ['actual_exposure','white_balance','gain','frame_counter','sensor_timestamp']:
  enum=getattr(rs.frame_metadata_value,key,None)
  if enum is not None and frame.supports_frame_metadata(enum):metadata[key]=frame.get_frame_metadata(enum)
 out.mkdir(parents=True);Image.fromarray(np.asarray(frame.get_data()).copy()).save(out/'rgb.png')
 (out/'capture.json').write_text(json.dumps({'serial':actual,'native_wh':[intr.width,intr.height],'fps':v.fps(),'K_native':[[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]],'sdk_distortion_model':str(intr.model),'distortion_coefficients':intr.coeffs,'timestamp_ms':frame.get_timestamp(),'timestamp_domain':str(frame.get_frame_timestamp_domain()),'warmup_s':a.warmup_s,'metadata':metadata,'exposure_stability_verified':False,'pixel_ops':{}},indent=2))
finally:pipe.stop()
