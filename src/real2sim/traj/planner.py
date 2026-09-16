"""Trajectory planning core (pure numpy/scipy, this repo's own implementation).

The isolated-contract design (structured refusals, joint limit checks, FK verification kept
separate) takes after Data-MechanicSim newton_gen/motion/planning/interface.py, but the
implementation follows this repo's contract: quaternions are always xyzw, units are metres,
radians and seconds, and gripper is 0=open,1=closed.
It does not import newton_gen or newton, so it can be unit-tested on its own; IK is solved in
the adapter layer.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class Waypoint:
    """One TCP target point. duration_s is the travel time from the previous point to this one."""

    position_m: tuple
    quaternion_xyzw: tuple
    gripper: float
    duration_s: float


@dataclass
class Infeasible(Exception):
    """A structured planning refusal, raised so it travels up through the solver stack; reason can be branched on by machine."""

    reason: str
    detail: str = ""


@dataclass
class JointTrajectory:
    """A time-parameterized joint trajectory. positions[i] is the joint vector (rad) at times[i]."""

    times: np.ndarray
    positions: np.ndarray
    joint_names: tuple
    fk_error_m: float = float("nan")

    @property
    def duration_s(self) -> float:
        return float(self.times[-1] - self.times[0]) if len(self.times) else 0.0


def _finite_numbers(value, n, name):
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ValueError(name + ": expected " + str(n) + " numbers")
    out = [float(v) for v in value]
    if any(isinstance(v, bool) or not math.isfinite(v) for v in value):
        raise ValueError(name + ": values must be finite")
    return out


def validate_task(spec: dict) -> dict:
    """Validate a task JSON (the plan_trajectory input contract); returns a normalized copy when it passes."""
    if spec.get("schema_version") != "1.0":
        raise ValueError("schema_version must be '1.0'")
    waypoints = spec.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("waypoints must be a non-empty list")
    normalized = []
    for i, w in enumerate(waypoints):
        name = "waypoints[%d]" % i
        position = _finite_numbers(w.get("position_m"), 3, name + ".position_m")
        quat = _finite_numbers(w.get("quaternion_xyzw"), 4, name + ".quaternion_xyzw")
        if abs(np.linalg.norm(quat) - 1) > 1e-4:
            raise ValueError(name + ".quaternion_xyzw: expected a unit quaternion (xyzw)")
        gripper = w.get("gripper", 0.0)
        if isinstance(gripper, bool) or not isinstance(gripper, (int, float)) or not 0.0 <= gripper <= 1.0:
            raise ValueError(name + ".gripper: expected 0=open .. 1=closed")
        duration = w.get("duration_s")
        if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration <= 0:
            raise ValueError(name + ".duration_s: expected a positive finite value in seconds")
        normalized.append(Waypoint(tuple(position), tuple(quat), float(gripper), float(duration)))
    q_start = spec.get("q_start_rad")
    if q_start is not None:
        q_start = _finite_numbers(q_start, len(q_start), "q_start_rad")
    fps = spec.get("fps", 30)
    if not isinstance(fps, int) or isinstance(fps, bool) or not 1 <= fps <= 240:
        raise ValueError("fps: expected an integer in [1, 240]")
    planning_frame = spec.get("planning_frame")
    if planning_frame is not None:
        if (not isinstance(planning_frame, dict)
                or set(planning_frame) != {"kind", "transform_file"}
                or planning_frame.get("kind") != "sensor_center"
                or not isinstance(planning_frame.get("transform_file"), str)
                or not planning_frame["transform_file"].strip()):
            raise ValueError("planning_frame requires kind=sensor_center and transform_file")
    return {"waypoints": normalized, "q_start_rad": q_start, "fps": fps,
            "planning_frame": planning_frame,
            "vel_limits_rad_s": spec.get("vel_limits_rad_s"),
            "acc_limits_rad_s2": spec.get("acc_limits_rad_s2")}


def quintic_scale(t: np.ndarray) -> np.ndarray:
    """Time scaling with zero end-point velocity and acceleration: s(t)=6t^5-15t^4+10t^3, t in [0,1]."""
    return 6 * t**5 - 15 * t**4 + 10 * t**3


def interpolate_waypoints(waypoints, fps: int):
    """Turn a waypoint sequence into per-frame TCP targets. Returns (times, positions, quats_xyzw, grippers).

    Uniform time step contract: the gap between frames is exactly 1/fps seconds. Each segment
    duration is quantized to the frame grid (n_frames = max(1, round(duration_s*fps)), so the
    quantization error is at most 1/(2*fps)). Position and gripper use quintic time scaling;
    orientation uses Slerp. waypoints[0] is the start state, built by the caller from the
    current TCP.
    """
    if len(waypoints) < 2:
        raise ValueError("need a start waypoint plus at least one target waypoint")
    start = waypoints[0]
    prev_p = np.asarray(start.position_m, float)
    prev_q = np.asarray(start.quaternion_xyzw, float)
    prev_g = float(start.gripper)
    times_all = [np.array([0.0])]
    pos_all = [prev_p[None, :].copy()]
    quat_all = [prev_q[None, :].copy()]
    grip_all = [np.array([prev_g])]
    frame_offset = 0
    for w in waypoints[1:]:
        n_frames = max(1, int(round(w.duration_s * fps)))
        seg_t = np.linspace(0.0, 1.0, n_frames + 1)
        s = quintic_scale(seg_t)
        target_p = np.asarray(w.position_m, float)
        target_q = np.asarray(w.quaternion_xyzw, float)
        target_g = float(w.gripper)
        seg_pos = prev_p[None, :] + s[:, None] * (target_p - prev_p)[None, :]
        slerp = Slerp([0.0, 1.0], Rotation.from_quat([prev_q, target_q]))
        seg_quat = slerp(s).as_quat()
        seg_grip = prev_g + s * (target_g - prev_g)
        frames = frame_offset + np.arange(1, n_frames + 1)  # segment start repeats the previous segment's last frame, so skip it
        times_all.append(frames / fps)
        pos_all.append(seg_pos[1:])
        quat_all.append(seg_quat[1:])
        grip_all.append(seg_grip[1:])
        prev_p, prev_q, prev_g = target_p, target_q, target_g
        frame_offset += n_frames
    return (np.concatenate(times_all), np.concatenate(pos_all),
            np.concatenate(quat_all), np.concatenate(grip_all))


def validate_limits(traj: JointTrajectory, vel_limits, acc_limits=None, *, tolerance: float = 1.05):
    """Check speed and acceleration against the joint limits using finite differences; returns a list of violations (empty means it passed)."""
    issues = []
    times, positions = np.asarray(traj.times, float), np.asarray(traj.positions, float)
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        return ["trajectory times must be strictly increasing"]
    vel = np.diff(positions, axis=0) / np.diff(times)[:, None]
    vel_limits = np.asarray(vel_limits, float)
    if vel_limits.shape != (positions.shape[1],) or np.any(vel_limits <= 0):
        raise ValueError("vel_limits must be positive per-joint values")
    excess = np.abs(vel) / vel_limits[None, :] - tolerance
    bad = np.argwhere(excess > 0)
    for frame, joint in bad[:20]:
        issues.append("joint %d velocity %.3f rad/s exceeds limit %.3f at frame %d"
                      % (joint, abs(vel[frame, joint]), vel_limits[joint], frame))
    if acc_limits is not None:
        acc_limits = np.asarray(acc_limits, float)
        if acc_limits.shape != (positions.shape[1],) or np.any(acc_limits <= 0):
            raise ValueError("acc_limits must be positive per-joint values")
        acc = np.diff(vel, axis=0) / (0.5 * (np.diff(times)[:-1] + np.diff(times)[1:]))[:, None]
        excess = np.abs(acc) / acc_limits[None, :] - tolerance
        bad = np.argwhere(excess > 0)
        for frame, joint in bad[:20]:
            issues.append("joint %d acceleration %.3f rad/s^2 exceeds limit %.3f at frame %d"
                          % (joint, abs(acc[frame, joint]), acc_limits[joint], frame))
    return issues
