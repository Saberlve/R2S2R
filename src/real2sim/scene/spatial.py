"""Optimize spatial relations from distance measurements (interface reserved).

Goal: constrain the spatial relations of scene entities using distance observations from a
laser meter or a tape measure. This round only provides the constraint contract and its
validation; the optimizer itself comes later.

Any solver must follow the variable locking order in docs/PIPELINE.md:
size -> intrinsics -> trusted extrinsics/robot -> secondary cameras/background -> lighting and materials.
A distance constraint may only adjust what the current stage allows, and must never be used to
move an already locked camera or robot base back into place.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Protocol


@dataclass(frozen=True)
class DistanceConstraint:
    """One distance measurement between two points.

    entity_a/entity_b are ids of assets or cameras in scene.json; distance_m is in metres;
    sigma_m is the measurement uncertainty (1 sigma, in metres); evidence points at the
    original record, such as a photo or note path.
    """

    entity_a: str
    entity_b: str
    distance_m: float
    sigma_m: float
    evidence: str


def validate_constraints(constraints, entity_ids) -> list[dict]:
    """Validate a set of distance constraints; entity_ids are the entity ids the scene allows.

    Returns a list of normalized dicts when it passes; any single bad entry raises ValueError.
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
    """Distance-based spatial optimizer interface (reserved, not implemented)."""

    def optimize(self, scene: dict, constraints: list[dict]) -> dict:
        """Return an adjusted copy of scene.json plus a report; never modify the input in place.

        Requirement: state the lock status of every adjusted variable explicitly, and keep the
        residuals and uncertainties in the report.
        """
        ...


def optimize_spatial(scene: dict, constraints: list[dict]) -> dict:
    raise NotImplementedError(
        "The distance-based spatial optimizer is a reserved interface; this round only provides "
        "the constraint contract and validate_constraints. Read the variable locking order in "
        "docs/PIPELINE.md before implementing it."
    )
