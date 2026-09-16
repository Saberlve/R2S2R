import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from real2sim.traj.planning_frame import PlanningFrame, recorded_tcp_poses
from real2sim.traj.planner import validate_task

def pose(xyz, angles=(0,0,0)):
    T=np.eye(4);T[:3,3]=xyz;T[:3,:3]=Rotation.from_euler('xyz',angles).as_matrix();return T

def frame(tmp_path):
    path=tmp_path/'frame.json'
    path.write_text(json.dumps({'schema_version':'1.0','length_unit':'m',
        'gripper_convention':'0=open,1=closed','transform_convention':'T_tcp_sensor',
        'T_flange_tcp':pose([0,0,.172]).tolist(),'gripper_closed_fraction':[0,.5,1],
        'T_tcp_sensor':[pose([.01,.02,.08],(.1,0,0)).tolist(),
                        pose([.02,0,.09],(.2,.1,0)).tolist(),
                        pose([.01,-.02,.085],(.3,.2,0)).tolist()]}))
    return PlanningFrame({'transform_file':path.name},tmp_path),path

def test_noncommuting_transform_roundtrip_and_recording(tmp_path):
    f,_=frame(tmp_path);B=pose([.5,-.2,.8],(.4,-.3,.2));target=pose([.1,.2,.7],(-.2,.5,.8))
    for g in [0,.25,.5,.75,1]:
        tcp=f.tcp_target(B@target,g)
        np.testing.assert_allclose(f.planning_pose(tcp,g),B@target,atol=1e-12)
        saved=recorded_tcp_poses(B,[tcp])
        expected=target@np.linalg.inv(f.tcp_to_sensor(g))
        np.testing.assert_allclose(saved['tcp_m'][0],expected[:3,3],atol=1e-12)
        assert np.linalg.norm(saved['tcp_m'][0]-target[:3,3])>.05
        assert set(saved)=={'tcp_m','tcp_quat_xyzw'}

def test_opening_changes_tcp_goal_while_sensor_target_stays_fixed(tmp_path):
    f,_=frame(tmp_path);S=pose([.4,0,.5])
    assert not np.allclose(f.tcp_target(S,0),f.tcp_target(S,1))
    for g in np.linspace(0,1,21):
        np.testing.assert_allclose(f.tcp_target(S,g)@f.tcp_to_sensor(g),S,atol=1e-12)

def test_legacy_identity_does_not_transform_eef():
    f=PlanningFrame();T=pose([.2,.3,.4],(.1,.2,.3))
    np.testing.assert_allclose(f.tcp_target(T,.4),T)
    np.testing.assert_allclose(f.planning_pose(T,.4),T)

def test_record_actual_fk_not_planned_target():
    actual=pose([.4001,.2,.3]);saved=recorded_tcp_poses(np.eye(4),[actual])
    assert saved['tcp_m'][0,0]==.4001

def test_invalid_calibration_and_out_of_range(tmp_path):
    f,path=frame(tmp_path)
    for g in [-.1,1.1,float('nan')]:
        with pytest.raises(ValueError):f.tcp_to_sensor(g)
    data=json.loads(path.read_text());data['T_flange_tcp'][2][3]=.26;path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='unchanged TCP172'):PlanningFrame({'transform_file':str(path)})

def test_task_requires_explicit_sensor_frame():
    task={'schema_version':'1.0','waypoints':[{'position_m':[0,0,0],
       'quaternion_xyzw':[0,0,0,1],'gripper':0,'duration_s':1}]}
    assert validate_task(task)['planning_frame'] is None
    config={'kind':'sensor_center','transform_file':'frame.json'}
    assert validate_task({**task,'planning_frame':config})['planning_frame']==config
    with pytest.raises(ValueError):validate_task({**task,'planning_frame':{'kind':'sensor'}})
