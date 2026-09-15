"""腕部相机标定核心：TCP→相机手眼变换的组合与校验。

T_A_B 约定与 docs/CONTRACTS.md 一致：T_flange_camera = T_flange_tcp @ T_tcp_camera。
控制器快照的平移单位为毫米，组合前显式转换为米。
本模块只做数学与记录构造；.blend 写入由 tools/adopt_wrist_tcp_calibration.py 完成。
"""
from __future__ import annotations

import math

import numpy as np

CV_TO_BLENDER = np.diag([1.0, -1.0, -1.0, 1.0])


def rigid(value, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"{name} has an invalid homogeneous last row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5):
        raise ValueError(f"{name} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=2e-5):
        raise ValueError(f"{name} rotation is not proper")
    return matrix


def controller_flange_tcp(controller: dict) -> np.ndarray:
    """Controller snapshots store end_transform translation in millimetres."""
    matrix = np.asarray(controller["end_transform"], dtype=float).copy()
    matrix[:3, 3] *= 0.001
    return rigid(matrix, "controller end_transform")


def rotation_distance_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = a[:3, :3].T @ b[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def build_record(measurement: dict, controller: dict, previous: dict) -> dict:
    t_tcp_camera = rigid(measurement["T_tcp_camera_cv"], "T_tcp_camera_cv")
    t_flange_tcp = controller_flange_tcp(controller)
    t_flange_camera = rigid(t_flange_tcp @ t_tcp_camera, "T_flange_camera_cv")
    old = rigid(previous["T_flange_camera_cv"], "previous T_flange_camera_cv")
    delta_mm = (t_flange_camera[:3, 3] - old[:3, 3]) * 1000.0
    post_rotation = int(measurement.get("post_rotation_degrees", 0))
    if post_rotation not in (0, 180):
        raise ValueError("post_rotation_degrees must be 0 or 180")
    return {
        "schema_version": "1.0",
        "quality": "calibrated",
        "adoption_status": "candidate_pending_independent_scene_validation",
        "method": measurement.get("method", "independent hand-eye calibration"),
        "source": measurement.get("source", "user-supplied calibration result"),
        "K": previous["K"],
        "resolution": previous["resolution"],
        "distortion_coeffs": previous["distortion_coeffs"],
        "image_orientation": measurement.get("image_orientation", previous["image_orientation"]),
        "post_rotation_degrees": post_rotation,
        "quaternion_xyzw": measurement.get("quaternion_xyzw"),
        "T_tcp_camera_cv": t_tcp_camera.tolist(),
        "T_flange_tcp": t_flange_tcp.tolist(),
        "T_flange_camera_cv": t_flange_camera.tolist(),
        "T_flange_camera_blender": (t_flange_camera @ CV_TO_BLENDER).tolist(),
        "reported_leaveout_max_error": measurement.get("reported_leaveout_max_error"),
        "comparison_to_previous_video_fit": {
            "previous_T_flange_camera_cv": old.tolist(),
            "optical_center_delta_flange_xyz_mm": delta_mm.tolist(),
            "optical_center_distance_mm": float(np.linalg.norm(delta_mm)),
            "rotation_distance_deg": rotation_distance_deg(old, t_flange_camera),
            "previous_heldout_rmse_px": previous.get("heldout", {}).get("rmse_px"),
            "previous_cross_session_rmse_px": 10.65,
        },
        "notes": [
            "T_A_B maps coordinates in B into A; T_flange_camera = T_flange_tcp @ T_tcp_camera.",
            "Controller end_transform translation was converted explicitly from millimetres to metres.",
            "The supplied leave-out error characterizes the hand-eye calibration, not this scene's pixel reprojection error.",
            "Do not claim scene validation until synchronized real images and measured robot poses are scored independently.",
        ],
    }
