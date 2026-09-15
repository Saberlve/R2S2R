import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "adopt_wrist_tcp_calibration.py"
SPEC = importlib.util.spec_from_file_location("adopt_wrist_tcp_calibration", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_tcp_pose_is_composed_with_controller_flange_tcp():
    measurement = {
        "post_rotation_degrees": 180,
        "T_tcp_camera_cv": [
            [-0.008212, 0.999961, 0.003294, 0.069371],
            [-0.999880, -0.008255, 0.013139, 0.025467],
            [0.013165, -0.003186, 0.999908, -0.158734],
            [0, 0, 0, 1],
        ]
    }
    controller = {"end_transform": np.eye(4).tolist()}
    controller["end_transform"][2][3] = 172.0
    previous = {
        "K": np.eye(3).tolist(), "resolution": [640, 480],
        "distortion_coeffs": [0] * 5, "image_orientation": "raw",
        "T_flange_camera_cv": np.eye(4).tolist(),
    }
    record = MODULE.build_record(measurement, controller, previous)
    actual = np.asarray(record["T_flange_camera_cv"])
    assert np.allclose(actual[:3, 3], [0.069371, 0.025467, 0.013266])
    expected_rotation = np.asarray(measurement["T_tcp_camera_cv"])[:3, :3]
    assert np.allclose(actual[:3, :3], expected_rotation)
    assert record["post_rotation_degrees"] == 180


def test_rejects_non_rigid_measurement():
    bad = np.eye(4)
    bad[0, 0] = 2
    try:
        MODULE.rigid(bad, "bad")
    except ValueError as exc:
        assert "orthonormal" in str(exc)
    else:
        raise AssertionError("non-rigid transform accepted")
