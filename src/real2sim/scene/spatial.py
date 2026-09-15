"""测距信息优化空间关系（接口预留）。

目标：用激光/卷尺等测距观测约束场景实体的空间关系。
本轮只提供约束契约与校验；优化求解器留待后续实现。

求解器实现时必须遵守 docs/PIPELINE.md 的变量锁定顺序：
尺寸 → 内参 → 可信外参/机器人 → 次要相机/背景 → 光照材质。
测距约束只允许调整当前阶段允许变动的量，禁止借测距误差回移已锁定的相机或机器人基座。
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Protocol


@dataclass(frozen=True)
class DistanceConstraint:
    """一次两点间测距观测。

    entity_a/entity_b：scene.json 中资产或相机的 id；distance_m 单位米；
    sigma_m 为测距不确定度（1σ，米）；evidence 指向原始记录（如照片/笔记路径）。
    """

    entity_a: str
    entity_b: str
    distance_m: float
    sigma_m: float
    evidence: str


def validate_constraints(constraints, entity_ids) -> list[dict]:
    """校验测距约束集合；entity_ids 为场景中可引用的实体 id 集合。

    通过时返回归一化后的 dict 列表；任何一条非法即抛 ValueError。
    """
    ids = set(entity_ids)
    out = []
    for i, c in enumerate(constraints):
        c = c if isinstance(c, DistanceConstraint) else DistanceConstraint(**c)
        name = f"constraints[{i}]"
        if c.entity_a == c.entity_b:
            raise ValueError(name + ": entity_a and entity_b must differ")
        for entity in (c.entity_a, c.entity_b):
            if entity not in ids:
                raise ValueError(name + ": unknown entity " + entity)
        for field, value in (("distance_m", c.distance_m), ("sigma_m", c.sigma_m)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(name + "." + field + ": expected a positive finite value in metres")
        if not c.evidence:
            raise ValueError(name + ": evidence is required for provenance")
        out.append(asdict(c))
    return out


class SpatialOptimizer(Protocol):
    """测距优化求解器接口（预留，未实现）。"""

    def optimize(self, scene: dict, constraints: list[dict]) -> dict:
        """返回调整后的 scene.json 副本与报告；不得就地修改输入。

        实现要求：显式声明每个被调整变量的锁定状态，报告中保留残差与不确定度。
        """
        ...


def optimize_spatial(scene: dict, constraints: list[dict]) -> dict:
    raise NotImplementedError(
        "测距优化求解器为预留接口，本轮仅提供约束契约与 validate_constraints；"
        "实现前请先阅读 docs/PIPELINE.md 的变量锁定顺序"
    )
