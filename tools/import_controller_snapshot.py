"""Normalize an existing read-only xArm controller snapshot; no SDK connection."""
import argparse,pathlib,json,numpy as np
p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--out',required=True);a=p.parse_args();raw=json.loads(pathlib.Path(a.input).read_text(encoding='utf-8-sig'));T=np.array(raw['end_transform'],float);T[:3,3]*=.001
out=pathlib.Path(a.out)
if out.exists():raise FileExistsError(out)
out.write_text(json.dumps({'schema_version':'1.0','joint_origins_xyz_m_rpy_rad':raw['joint_origins'],'joint_axes':[[0,0,1]]*len(raw['joint_origins']),'T_flange_tcp':T.tolist(),'controller_world_offset_snapshot':raw['world_offset'],'tcp_output_frame':'robot_base','notes':'Controller world offset is retained as metadata, not applied to base-frame FK; explicitly transform when exporting controller-world poses.'},indent=2))
