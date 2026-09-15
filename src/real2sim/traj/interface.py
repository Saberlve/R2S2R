"""轨迹产生通用接口（预留）。

现有实现：real2sim.traj.adapters.xarm7（Newton 引擎，xArm7 + G2 + TCP172）。
本接口定义与未来规划器/其他引擎对接的能力面，本轮不新增实现。
轨迹数据契约沿用 docs/CONTRACTS.md 的 episode NPZ schema
（time/q/action_q/gripper/action_gripper，rad/秒，gripper 0=open,1=closed）。
"""
from __future__ import annotations

from typing import Protocol

ENGINE_REGISTRY = {
    "newton_xarm7": {
        "adapter": "real2sim.traj.adapters.xarm7",
        "status": "implemented",
        "hardware_guard": "xArm7 G2 TCP172 only; no real-robot commands",
    },
    "tacsim_tactile": {
        "adapter": None,
        "status": "reserved_not_implemented",
        "notes": "触觉仿真后端接入点，见 real2sim.tactile.interface",
    },
}


class TrajectoryGenerator(Protocol):
    """任务级轨迹生成器接口（预留，未实现）。"""

    def generate(self, task_spec: dict, scene_config: dict) -> dict:
        """按任务描述生成 episode（NPZ + manifest）。

        实现要求：产物通过现有运动学/动力学门禁（FK、跟踪、穿透阈值见
        docs/CONTRACTS.md）；生成轨迹不等于真实轨迹接触验证。
        """
        ...


class EngineAdapter(Protocol):
    """仿真引擎适配器能力面：xarm7 适配器已实现的能力集合。"""

    def validate_kinematics(self, case: str) -> dict: ...
    def simulate_replay(self, case: str) -> dict: ...
    def export_trajectory(self, case: str) -> dict: ...
