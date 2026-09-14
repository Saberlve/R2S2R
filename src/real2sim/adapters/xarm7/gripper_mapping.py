import json,xml.etree.ElementTree as ET,numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
import os
R=Path(os.environ['R2S_CASE_ROOT']).resolve()
tree=ET.parse(R/'inputs/xarm7_calibrated.urdf')
joints=tree.findall('./joint')
def chain(drive):
 T={'world':np.eye(4)}
 pending=list(joints)
 while pending:
  progress=False
  for j in pending[:]:
   parent=j.find('parent').get('link');child=j.find('child').get('link')
   if parent not in T:continue
   o=j.find('origin');M=np.eye(4)
   if o is not None:
    M[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
    M[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
   if j.get('type')=='revolute' and not j.get('name').startswith('joint'):
    axis=np.fromstring(j.find('axis').get('xyz'),sep=' ')
    A=np.eye(4);A[:3,:3]=Rotation.from_rotvec(axis*drive).as_matrix();M=M@A
   T[child]=T[parent]@M;pending.remove(j);progress=True
  if not progress:raise RuntimeError('URDF chain')
 return T
qs=np.linspace(0,.85,501);gaps=[];centres=[]
for q in qs:
 t=chain(q);inv=np.linalg.inv(t['xarm_gripper_base_link'])
 left=inv@t['left_finger']@np.array([0,-.026003,.025,1])
 right=inv@t['right_finger']@np.array([0,.026003,.025,1])
 gaps.append(abs(left[1]-right[1]))
 centres.append((np.linalg.inv(t['link7'])@t['xarm_gripper_base_link']@((left+right)/2))[:3].tolist())
gaps=np.array(gaps)
print('GAPS',gaps[0],gaps[-1],gaps.min(),gaps.max())
(R/'inputs/gripper_mapping.json').write_text(json.dumps({'drive_rad':qs.tolist(),'gap_m':gaps.tolist(),'sdk_open_m':.084,'sdk_closed_fraction':'0=open,1=closed','mapping_model':'G2 moving geometry verified against official DAE; sampled inner pad plane', 'sampled_pad_midpoint_in_flange_m':centres, 'grasp_reference_note':'Representative pad points, not independently calibrated grasp center; distinct from fixed SDK TCP at z=0.172m','geometry_is_approximation':True},indent=2))
