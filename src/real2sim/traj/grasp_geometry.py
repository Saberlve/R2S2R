"""Geometry checks for table-supported side grasps.

All distances are meters.  Intervals are projected onto the table normal, so
the calculation also applies when the measured table is not exactly world-Z.
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


class SideGraspInfeasible(ValueError):
    """The requested side grasp cannot keep clearance and useful contact."""


@dataclass(frozen=True)
class SideGraspGeometry:
    position_m: np.ndarray
    object_min_height_m: float
    object_max_height_m: float
    center_height_m: float
    grasp_height_m: float
    upward_shift_m: float
    contact_overlap_m: float


def _finite_vector(value, length, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite {length}-vector")
    return result


def safe_side_grasp(
    *,
    object_center_m,
    object_size_m,
    object_quaternion_xyzw,
    table_matrix,
    table_size_m,
    sensor_bottom_offset_m,
    clearance_m,
    pad_down_m,
    pad_up_m,
    minimum_overlap_m,
):
    """Raise a center side grasp just enough to clear the support surface.

    The object is an oriented box. ``pad_down_m`` and ``pad_up_m`` describe
    the effective contact interval below and above the sensor-center TCP.
    """
    center = _finite_vector(object_center_m, 3, "object_center_m")
    size = _finite_vector(object_size_m, 3, "object_size_m")
    if np.any(size <= 0):
        raise ValueError("object_size_m must be positive")
    quat = _finite_vector(object_quaternion_xyzw, 4, "object_quaternion_xyzw")
    if not np.isclose(np.linalg.norm(quat), 1.0, atol=1e-6, rtol=0):
        raise ValueError("object_quaternion_xyzw must be normalized")
    table = np.asarray(table_matrix, dtype=float)
    if table.shape != (4, 4) or not np.isfinite(table).all():
        raise ValueError("table_matrix must be a finite 4x4 matrix")
    table_size = _finite_vector(table_size_m, 3, "table_size_m")
    if np.any(table_size <= 0):
        raise ValueError("table_size_m must be positive")
    values = np.asarray(
        [sensor_bottom_offset_m, clearance_m, pad_down_m, pad_up_m, minimum_overlap_m],
        dtype=float,
    )
    if not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError("grasp clearance and contact dimensions must be finite and non-negative")

    normal = table[:3, 2]
    normal_norm = np.linalg.norm(normal)
    if not np.isclose(normal_norm, 1.0, atol=1e-6, rtol=0):
        raise ValueError("table_matrix Z axis must be unit length")
    rotation = Rotation.from_quat(quat).as_matrix()
    half_extent = float(np.sum(np.abs(normal @ rotation) * size / 2.0))
    center_height = float(normal @ center)
    object_min = center_height - half_extent
    object_max = center_height + half_extent
    table_top = float(normal @ table[:3, 3] + table_size[2] / 2.0)

    safe_height = table_top + float(sensor_bottom_offset_m) + float(clearance_m)
    grasp_height = max(center_height, safe_height)
    shift = grasp_height - center_height
    position = center + normal * shift
    pad_min = grasp_height - float(pad_down_m)
    pad_max = grasp_height + float(pad_up_m)
    overlap = max(0.0, min(object_max, pad_max) - max(object_min, pad_min))
    if overlap + 1e-12 < float(minimum_overlap_m):
        raise SideGraspInfeasible(
            "side grasp has only "
            f"{overlap * 1000:.3f} mm vertical contact overlap after table clearance; "
            f"requires {float(minimum_overlap_m) * 1000:.3f} mm"
        )
    return SideGraspGeometry(
        position_m=position,
        object_min_height_m=object_min,
        object_max_height_m=object_max,
        center_height_m=center_height,
        grasp_height_m=grasp_height,
        upward_shift_m=shift,
        contact_overlap_m=overlap,
    )
