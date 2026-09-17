import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from real2sim.traj.planning_frame import SensorTCP, recorded_tcp_poses
from real2sim.traj.planner import validate_task


@pytest.fixture
def case(tmp_path):
    inputs=tmp_path/'inputs';inputs.mkdir()
    robot=ET.Element('robot',name='test')
    ET.SubElement(robot,'link',name='link7')
    for side,letter,sign in [('left','L',1),('right','R',-1)]:
        link=ET.SubElement(robot,'link',name=side+'_finger')
        for part in ('connector','fingertip','sensor'):
            ET.SubElement(link,'collision',name=f'custom_{letter}_{part}_in_finger_frame_m')
        j=ET.SubElement(robot,'joint',name='drive_joint' if sign==1 else 'follower',type='revolute')
        ET.SubElement(j,'parent',link='link7');ET.SubElement(j,'child',link=side+'_finger')
        ET.SubElement(j,'origin',xyz=f'0 {sign*.05} .1');ET.SubElement(j,'axis',xyz=f'{sign} 0 0')
        if sign==-1:ET.SubElement(j,'mimic',joint='drive_joint')
        j=ET.SubElement(robot,'joint',name=f'custom_contact_{letter}_fix',type='fixed')
        ET.SubElement(j,'parent',link=side+'_finger');ET.SubElement(j,'child',link=f'custom_contact_{letter}')
        ET.SubElement(j,'origin',xyz=f'0 {-sign*.01} .15')
    ET.ElementTree(robot).write(inputs/'xarm7_calibrated.urdf')
    (inputs/'gripper_mapping.json').write_text(json.dumps(dict(gap_m=[.084,0],drive_rad=[0,.5],sdk_open_m=.084)))
    return tmp_path


def pose(xyz,angle=0):
    T=np.eye(4);T[:3,3]=xyz;T[:3,:3]=Rotation.from_euler('x',angle).as_matrix();return T


def test_tcp_tracks_linkage_and_transform_roundtrip(case):
    tcp=SensorTCP(case);world=pose([.3,.4,.5],.7)
    for g in (0,.3,1):
        offset=tcp.flange_transform(g)
        expected=.1-.01*np.sin(.5*g)+.15*np.cos(.5*g)
        np.testing.assert_allclose(offset[:3,3],[0,0,expected],atol=1e-12)
        target=world@offset
        np.testing.assert_allclose(target@np.linalg.inv(offset),world,atol=1e-12)
    assert not np.allclose(tcp.flange_transform(0),tcp.flange_transform(1))


def test_actual_tcp_uses_actual_fingers_not_command(case):
    tcp=SensorTCP(case)
    labels=['r/link7','r/left_finger','r/right_finger']
    frames=[pose([.3,.2,.5]),pose([.3,.25,.6],.1),pose([.3,.15,.6],-.2)]
    body=np.array([np.r_[t[:3,3],Rotation.from_matrix(t[:3,:3]).as_quat()] for t in frames])
    actual=tcp.actual_pose(body,labels)
    expected=((frames[1]@tcp.mounts['left_finger'])[:3,3]+(frames[2]@tcp.mounts['right_finger'])[:3,3])/2
    np.testing.assert_allclose(actual[:3,3],expected)
    saved=recorded_tcp_poses(pose([.1,0,0]),[actual]);tcp.require_episode(saved)
    np.testing.assert_allclose(saved['tcp_m'][0],expected-[.1,0,0])
    with pytest.raises(ValueError,match='unique'):tcp.actual_pose(body,['r/link7']*3)


def test_missing_part_or_mount_is_rejected(case):
    path=case/'inputs/xarm7_calibrated.urdf';tree=ET.parse(path)
    link=tree.find("./link[@name='left_finger']");link.remove(link.find('collision'));tree.write(path)
    with pytest.raises(ValueError,match='collider'):SensorTCP(case)


def test_old_tcp_episodes_and_transform_files_are_rejected(case):
    tcp=SensorTCP(case)
    for data in ({},{'tcp_definition':np.asarray('controller_tcp')}):
        with pytest.raises(ValueError,match='replan'):tcp.require_episode(data)
    for g in (-.1,1.1,float('nan')):
        with pytest.raises(ValueError):tcp.flange_transform(g)
    task=dict(schema_version='1.0',waypoints=[dict(position_m=[0,0,0],quaternion_xyzw=[0,0,0,1],gripper=0,duration_s=1)])
    validate_task(task)
    validate_task({**task,'planning_frame':{'kind':'sensor_center'}})
    with pytest.raises(ValueError,match='legacy'):validate_task({**task,'planning_frame':{'kind':'sensor_center','transform_file':'old.json'}})
