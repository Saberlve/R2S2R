"""Derived quantities for a replayed grasp run: its outcome metrics and its playback clock.

Every duration here is read from the recorded timestamps.  A frame count is not a duration:
the tactile configuration lets the same run replay at a different control rate, so a fixed
number of frames is a different number of seconds in each configuration.

Distances and heights are meters, times are seconds, rates are hertz, and the lift is projected
onto the table normal, so the metric also applies when the measured table is not exactly world-Z.
"""
from __future__ import annotations

import numpy as np


def playback_fps(time_s, stride: int = 1):
    """Frame rate that plays a strided render of a recorded timeline back in real time.

    The saved timestamps are the only ground truth for a replay's clock: the control rate
    follows the tactile configuration, so a fixed 30 Hz would play a 60 Hz run at half speed
    and leave the three-view video out of step with the tactile one.

    Returns None for a timeline of a single frame, which has no interval to play over.  A
    non-uniform timeline is refused rather than averaged into a rate it never had.
    """
    time = np.asarray(time_s, dtype=float)
    stride = int(stride)
    if time.ndim != 1 or not len(time):
        raise ValueError("time_s must hold one timestamp per frame")
    if not np.isfinite(time).all() or np.any(np.diff(time) <= 0):
        raise ValueError("time_s must be finite and strictly increasing")
    if stride < 1:
        raise ValueError("stride must be positive")
    if len(time) == 1:
        return None
    intervals = np.diff(time)
    # Same uniformity tolerance as the scene video encoder, which is the other place a rate is
    # read off recorded timestamps.
    if not np.allclose(intervals, intervals[0], rtol=1e-5, atol=1e-8):
        raise ValueError(
            "playback requires uniform timestamps; explicitly resample the states first"
        )
    return float(1.0 / (intervals[0] * stride))


def longest_hold_s(satisfied, time_s) -> float:
    """Longest continuous stretch, in seconds, over which ``satisfied`` stays true."""
    flags = np.asarray(satisfied, dtype=bool)
    time = np.asarray(time_s, dtype=float)
    if flags.ndim != 1 or time.ndim != 1 or time.shape != flags.shape:
        raise ValueError("satisfied and time_s must hold one value per frame")
    if not np.isfinite(time).all() or np.any(np.diff(time) < 0):
        raise ValueError("time_s must be finite and non-decreasing")
    longest, start = 0.0, None
    for index, flag in enumerate(flags):
        if not flag:
            start = None
            continue
        if start is None:
            start = index
        longest = max(longest, float(time[index] - time[start]))
    return longest


def bar_lift_report(
    bar_world, table_matrix, time_s, *, clearance_m: float = 0.05, hold_s: float = 2.0
) -> dict:
    """Lift of the held bar along the table normal, and the longest hold above ``clearance_m``.

    Heights are measured from the bar's own first frame, so a bar that starts slightly off the
    support still reads as unlifted before the grasp closes.
    """
    bar = np.asarray(bar_world, dtype=float)
    time = np.asarray(time_s, dtype=float)
    if bar.ndim != 2 or bar.shape[1] < 3 or not len(bar):
        raise ValueError("bar_world must hold one pose per frame")
    if time.ndim != 1 or time.shape[0] != bar.shape[0]:
        raise ValueError("bar_world and time_s must hold one pose per frame")
    if not np.isfinite(bar).all():
        raise ValueError("bar_world must be finite")
    table = np.asarray(table_matrix, dtype=float)
    if table.shape != (4, 4) or not np.isfinite(table).all():
        raise ValueError("table_matrix must be a finite 4x4 matrix")
    normal = table[:3, 2]
    if not np.isclose(np.linalg.norm(normal), 1.0, atol=1e-6, rtol=0):
        raise ValueError("table_matrix Z axis must be unit length")
    if not np.isfinite([clearance_m, hold_s]).all() or min(clearance_m, hold_s) < 0:
        raise ValueError("clearance_m and hold_s must be finite and non-negative")
    height = (bar[:, :3] - bar[0, :3]) @ normal
    hold = longest_hold_s(height >= clearance_m, time)
    return {
        "max_bar_lift_m": float(height.max()),
        "longest_clearance_hold_s": hold,
        "held_5cm_for_2s": bool(hold >= hold_s),
        "lift_clearance_m": float(clearance_m),
        "lift_hold_required_s": float(hold_s),
    }
