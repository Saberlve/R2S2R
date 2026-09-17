import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real2sim.traj.grasp_geometry import SideGraspInfeasible, safe_side_grasp


def geometry(height, *, table=None, rotation=None, minimum_overlap=.006):
    table=np.eye(4) if table is None else table
    rotation=np.eye(3) if rotation is None else rotation
    normal=table[:3,2]
    top=table[:3,3]+normal*.02
    center=top+normal*height/2
    return safe_side_grasp(
        object_center_m=center,object_size_m=[.126,.033,height],
        object_quaternion_xyzw=Rotation.from_matrix(rotation).as_quat(),
        table_matrix=table,table_size_m=[1.2,1.6,.04],
        sensor_bottom_offset_m=.01752,clearance_m=.002,
        pad_down_m=.01417,pad_up_m=.01417,
        minimum_overlap_m=minimum_overlap)


def test_low_object_is_raised_only_enough_for_sensor_clearance():
    result=geometry(.033)
    assert result.center_height_m==pytest.approx(.0365)
    assert result.grasp_height_m==pytest.approx(.03952)
    assert result.upward_shift_m==pytest.approx(.00302)
    assert result.contact_overlap_m==pytest.approx(.02765)


def test_high_object_keeps_center_grasp():
    result=geometry(.05)
    assert result.upward_shift_m==pytest.approx(0)
    np.testing.assert_allclose(result.position_m,[0,0,.045])


def test_clearance_uses_table_normal_not_world_z():
    rotation=Rotation.from_euler('x',23,degrees=True).as_matrix()
    table=np.eye(4);table[:3,:3]=rotation;table[:3,3]=[.2,-.1,.7]
    result=geometry(.033,table=table,rotation=rotation)
    normal=table[:3,2]
    assert normal@result.position_m==pytest.approx(normal@table[:3,3]+.02+.01952)
    assert result.upward_shift_m==pytest.approx(.00302)


def test_side_grasp_is_rejected_when_contact_overlap_is_too_small():
    with pytest.raises(SideGraspInfeasible,match='vertical contact overlap'):
        geometry(.004)


@pytest.mark.parametrize('field,value',[
    ('clearance_m',-.001),('minimum_overlap_m',float('nan')),
])
def test_invalid_clearance_inputs_are_rejected(field,value):
    kwargs=dict(object_center_m=[0,0,.03],object_size_m=[.1,.03,.02],
        object_quaternion_xyzw=[0,0,0,1],table_matrix=np.eye(4),table_size_m=[1,1,.04],
        sensor_bottom_offset_m=.01752,clearance_m=.002,pad_down_m=.01417,pad_up_m=.01417,
        minimum_overlap_m=.006)
    kwargs[field]=value
    with pytest.raises(ValueError):safe_side_grasp(**kwargs)
