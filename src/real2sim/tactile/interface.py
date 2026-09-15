"""触觉接入接口（预留；tacsim 等开源后端为计划项）。

本轮只定义契约草案与传感器模型接口，不含任何触觉仿真实现。
诚实性规则（对齐 docs/LESSONS.md）：触觉输出是仿真观测量，
禁止据此声称真实接触验证成功。
"""
from __future__ import annotations

from typing import Protocol

CONTRACT_DRAFT = {
    "schema_version": "1.0-draft",
    "status": "reserved_not_implemented",
    "sensor": {
        "taxel_grid": "触觉阵列维度 [rows, cols]，每 taxel 输出法向/切向力（N）",
        "frame": "T_world_sensor 为刚体 4x4（T_A_B 约定，米）；复用 contracts.rigid 校验",
        "mount": "fixed | body；body 挂载语义对齐相机契约的腕部挂载（需逐帧位姿）",
        "output": "逐帧 taxel 力 + 合力和合力矩；单位 N / N·m，时间秒",
    },
    "backends_planned": ["tacsim", "other open-source tactile simulators"],
    "notes": [
        "触觉仿真输出不是真实接触验证；报告中必须保留仿真来源与不确定度。",
        "接入时不得修改已锁定的相机/基座标定；摩擦等接触参数遵循 PIPELINE.md 锁定顺序。",
    ],
}


class TactileSensorModel(Protocol):
    """触觉传感器模型接口（预留，未实现）。"""

    def attach(self, body: str, T_body_sensor: list) -> None:
        """把传感器刚体挂载到指定刚体；T_body_sensor 必须过 contracts.rigid。"""
        ...

    def sense(self, state: dict) -> dict:
        """对一帧仿真状态采样，返回符合 CONTRACT_DRAFT.sensor.output 的观测。"""
        ...


def describe() -> dict:
    return CONTRACT_DRAFT
