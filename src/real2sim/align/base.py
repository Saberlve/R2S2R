"""基座坐标系对齐：控制器快照归一化与 scene_config 基座标定校验。

坐标约定遵循 docs/CONTRACTS.md：T_A_B 把 B 系坐标列向量映到 A 系；单位米/弧度。
控制器快照 end_transform 平移为毫米，归一化时显式转换为米。
"""
from __future__ import annotations

import math

import numpy as np

from ..contracts import rigid

_REQUIRED_KEYS = ("T_sim_base", "tcp_offset_m", "table_matrix", "table_size_m", "bar")


def snapshot_to_calibration(raw: dict) -> dict:
    """Normalize a read-only xArm controller snapshot into FK calibration JSON."""
    T = np.array(raw["end_transform"], float)
    T[:3, 3] *= 0.001
    rigid(T, "controller end_transform")
    return {
        "schema_version": "1.0",
        "joint_origins_xyz_m_rpy_rad": raw["joint_origins"],
        "joint_axes": [[0, 0, 1]] * len(raw["joint_origins"]),
        "T_flange_tcp": T.tolist(),
        "controller_world_offset_snapshot": raw["world_offset"],
        "tcp_output_frame": "robot_base",
        "notes": "Controller world offset is retained as metadata, not applied to base-frame FK; explicitly transform when exporting controller-world poses.",
    }


def _vec3(value, name, positive=False):
    if (not isinstance(value, list) or len(value) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise ValueError(name + ": expected 3 finite numbers")
    if positive and min(value) <= 0:
        raise ValueError(name + ": expected positive values")
    return [float(v) for v in value]


def validate_scene_config(cfg: dict) -> dict:
    """校验 case scene_config.json 的基座对齐相关字段（docs/ROBOT.md 契约）。

    只校验，不修改输入；通过时返回报告字典。
    """
    missing = [k for k in _REQUIRED_KEYS if k not in cfg]
    if missing:
        raise ValueError("scene_config missing keys: " + ", ".join(missing))
    rigid(np.asarray(cfg["T_sim_base"], float), "T_sim_base")
    rigid(np.asarray(cfg["table_matrix"], float), "table_matrix")
    tcp = _vec3(cfg["tcp_offset_m"], "tcp_offset_m")
    size = _vec3(cfg["table_size_m"], "table_size_m", positive=True)
    bar = cfg["bar"]
    if not isinstance(bar, dict):
        raise ValueError("bar: expected object")
    for key in ("position_m", "quaternion_xyzw", "size_m"):
        if key not in bar:
            raise ValueError("bar missing key: " + key)
    _vec3(bar["position_m"], "bar.position_m")
    q = np.asarray(bar["quaternion_xyzw"], float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-4:
        raise ValueError("bar.quaternion_xyzw: expected unit xyzw quaternion")
    _vec3(bar["size_m"], "bar.size_m", positive=True)
    return {
        "valid": True,
        "checks": {
            "T_sim_base": "rigid 4x4 (T_A_B convention, metres)",
            "table_matrix": "rigid 4x4",
            "tcp_offset_m": tcp,
            "table_size_m": size,
            "bar": "position/quaternion/size valid; friction left to physics stage",
        },
        "notes": "Base alignment evidence only; does not by itself validate contact dynamics.",
    }
