"""Contracts and artifact writer for contact-driven Photon runs.

This module deliberately has no Newton or TacSim imports.  The replay adapter
can therefore validate a run before allocating a GPU or creating an output
directory, and unit tests can exercise the data contract without pretending a
physics or optical backend ran.
"""
from __future__ import annotations

import hashlib
import json
import pathlib
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from real2sim.contracts import rigid

SIDES = ("left", "right")
BACKEND_ENGINE = {"hydroelastic": "mujoco"}
PHOTON_GEL_SIZE_M = (0.0173, 0.02914, 0.003)
PHOTON_DEPTH_SHAPE = (100, 64)
PHOTON_RGB_SHAPE = (700, 400, 3)
# Every quaternion in this repository's episode schema is xyzw (see real2sim.traj.planner).
QUATERNION_SUFFIX = "quat_xyzw"


def _finite_positive(value: Any, name: str) -> float:
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def validate_tactile_config(doc: dict, *, engine: str | None = None) -> dict:
    """Validate and normalize one case-owned tactile configuration."""
    if not isinstance(doc, dict):
        raise ValueError("tactile config must be a JSON object")
    allowed = {
        "schema_version", "sensor", "contact_backend", "mounts", "physics",
        "photon", "parameters_source", "calibration_status", "control_hz",
        "chunk_frames", "existing_contact_face_size_m",
    }
    extra = set(doc) - allowed
    if extra:
        raise ValueError(f"unknown tactile config fields: {sorted(extra)}")
    if doc.get("schema_version") != "1.0":
        raise ValueError("tactile schema_version must be '1.0'")
    if doc.get("sensor") != "photon":
        raise ValueError("only sensor='photon' is supported")
    backend = doc.get("contact_backend")
    if backend not in BACKEND_ENGINE:
        raise ValueError("contact_backend must be hydroelastic")
    mapped = BACKEND_ENGINE[backend]
    if engine is not None and engine != mapped:
        raise ValueError(
            f"contact_backend={backend!r} requires engine={mapped!r}, got {engine!r}"
        )
    mounts = doc.get("mounts")
    if not isinstance(mounts, dict) or set(mounts) != set(SIDES):
        raise ValueError("mounts must contain exactly left and right")
    normalized_mounts = {}
    for side in SIDES:
        item = mounts[side]
        if not isinstance(item, dict) or set(item) != {"body", "T_body_sensor"}:
            raise ValueError(f"mounts.{side} must contain body and T_body_sensor")
        body = item["body"]
        if not isinstance(body, str) or not body:
            raise ValueError(f"mounts.{side}.body must be a non-empty string")
        T = rigid(item["T_body_sensor"])
        normalized_mounts[side] = {"body": body, "T_body_sensor": T.tolist()}
    physics = doc.get("physics")
    photon = doc.get("photon")
    if not isinstance(physics, dict) or not physics:
        raise ValueError("physics must record the backend material preset")
    if not isinstance(photon, dict) or not photon:
        raise ValueError("photon must record the optical/FEM preset")
    source = doc.get("parameters_source")
    if not isinstance(source, dict) or not source.get("physics") or not source.get("photon"):
        raise ValueError("parameters_source must name physics and photon sources")
    if doc.get("calibration_status") != "uncalibrated":
        raise ValueError("this integration requires calibration_status='uncalibrated'")
    control_hz = _finite_positive(doc.get("control_hz", 30), "control_hz")
    if not float(control_hz).is_integer():
        # The physics clock is an integer frame rate, so a fractional control rate could only be
        # rounded away -- and the recorded timestamps would then measure a different duration
        # than the replay integrated.
        raise ValueError(
            "control_hz must be a whole number of frames per second: the physics clock cannot "
            "run at a fractional rate"
        )
    control_hz = int(control_hz)
    chunk_frames = int(doc.get("chunk_frames", 64))
    if chunk_frames < 1:
        raise ValueError("chunk_frames must be positive")
    existing = doc.get("existing_contact_face_size_m")
    if existing is not None:
        existing = np.asarray(existing, dtype=float)
        if existing.shape != (2,) or not np.isfinite(existing).all() or np.any(existing <= 0):
            raise ValueError("existing_contact_face_size_m must be two positive metres")
        existing = existing.tolist()
    out = dict(doc)
    out.update(
        mounts=normalized_mounts,
        control_hz=control_hz,
        chunk_frames=chunk_frames,
        photon_gel_size_m=list(PHOTON_GEL_SIZE_M),
        engine=mapped,
        cuda_graph=False,
    )
    if existing is not None:
        out["existing_contact_face_size_m"] = existing
        out["contact_face_size_delta_m"] = [
            PHOTON_GEL_SIZE_M[0] - existing[0], PHOTON_GEL_SIZE_M[1] - existing[1]
        ]
    return out


def is_quaternion_field(name: str) -> bool:
    """Whether a frame-indexed array holds rotations, so it must be sampled along the arc."""
    if name == QUATERNION_SUFFIX or name.endswith("_" + QUATERNION_SUFFIX):
        return True
    if "quat" in name:
        raise ValueError(
            f"{name!r} looks like a quaternion but is not {QUATERNION_SUFFIX!r}; convert it "
            "to the episode order before resampling rather than blending its components"
        )
    return False


def _slerp(value: np.ndarray, time: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Sample a rotation track on the control clock, always along the shortest arc.

    Blending quaternions component by component is not interpolation: two encodings of one
    attitude (q and -q) cancel at the midpoint, and the result is not a rotation at all.
    """
    quat = value.reshape(len(time), 4).astype(float)
    if not np.isfinite(quat).all() or np.any(np.linalg.norm(quat, axis=1) <= 0):
        raise ValueError("quaternion track must be finite and non-zero")
    # Slerp refuses to extrapolate.  The completed final period is a held sample by
    # construction, so clamping the query is the same answer the interpolator would give.
    sampled = Slerp(time, Rotation.from_quat(quat))(
        np.clip(target, time[0], time[-1])
    ).as_quat()
    return sampled.reshape((len(target),) + value.shape[1:])


def resample_trajectory(arrays: dict[str, np.ndarray], control_hz: float) -> tuple[dict, dict]:
    """Resample every frame-indexed numeric array to a uniform control clock.

    Rotations are sampled along the shortest arc; every other numeric field is linear.

    The grid covers a whole number of control periods.  The replay advances exactly one period
    per frame, so a short trailing frame would be recorded as a fraction of the time that was
    actually integrated.  The final period is therefore completed by holding the last sample,
    and the report states how much was held rather than leaving it to be inferred.
    """
    if "time" not in arrays:
        raise ValueError("trajectory is missing time")
    time = np.asarray(arrays["time"], dtype=float)
    if time.ndim != 1 or len(time) < 2 or not np.isfinite(time).all() or np.any(np.diff(time) <= 0):
        raise ValueError("trajectory time must be finite and strictly increasing")
    dt = 1.0 / _finite_positive(control_hz, "control_hz")
    periods = int(np.ceil((time[-1] - time[0]) / dt - 1e-9))
    target = time[0] + np.arange(periods + 1) * dt
    uniform_input = bool(np.allclose(np.diff(time), np.diff(time)[0], rtol=1e-6, atol=1e-9))
    out: dict[str, np.ndarray] = {}
    for name, value in arrays.items():
        value = np.asarray(value)
        if value.ndim == 0 or len(value) != len(time) or not np.issubdtype(value.dtype, np.number):
            out[name] = value.copy()
            continue
        if is_quaternion_field(name):
            if value.shape[1:] != (4,):
                raise ValueError(f"{name} must hold four components per frame, got {value.shape}")
            out[name] = _slerp(value, time, target)
            continue
        flat = value.reshape(len(time), -1)
        sampled = np.stack([np.interp(target, time, flat[:, i]) for i in range(flat.shape[1])], axis=1)
        out[name] = sampled.reshape((len(target),) + value.shape[1:])
    out["time"] = target
    return out, {
        "input_frames": len(time), "output_frames": len(target),
        "input_uniform": uniform_input, "resampled": not (
            len(target) == len(time) and np.allclose(target, time, rtol=0, atol=1e-9)
        ), "control_hz": float(control_hz), "frame_dt_s": dt,
        "duration_s": float(target[-1] - target[0]),
        "trailing_hold_s": max(0.0, float(target[-1] - time[-1])),
    }


def sha256(path: str | pathlib.Path) -> str:
    h = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def json_safe(value: Any) -> Any:
    """Convert numpy values and reject non-finite JSON numbers."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        raise ValueError("JSON artifacts must not contain NaN or infinity")
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def write_rgb_videos(root: str | pathlib.Path, fps: float) -> dict[str, str]:
    """Encode native Photon RGB chunks without changing their resolution."""
    import cv2

    root = pathlib.Path(root)
    outputs = {}
    for side in SIDES:
        target = root / f"{side}_rgb.mp4"
        writer = cv2.VideoWriter(
            str(target), cv2.VideoWriter_fourcc(*"mp4v"), float(fps),
            (PHOTON_RGB_SHAPE[1], PHOTON_RGB_SHAPE[0]),
        )
        if not writer.isOpened():
            raise RuntimeError(f"failed to open tactile RGB video writer: {target}")
        try:
            for chunk in sorted((root / side).glob("chunk_*.npz")):
                with np.load(chunk) as arrays:
                    for rgb in arrays["rgb"]:
                        writer.write(np.ascontiguousarray(rgb[..., ::-1]))
        finally:
            writer.release()
        outputs[side] = str(target)
    manifest_path = root / "tactile_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest["rgb_videos"] = {
            side: str(pathlib.Path(path).relative_to(root)) for side, path in outputs.items()
        }
        manifest_path.write_text(json.dumps(json_safe(manifest), indent=2) + "\n")
    return outputs


@dataclass
class TactileRunWriter:
    """Chunked two-sensor artifact writer with one update per frame."""
    root: pathlib.Path
    config: dict

    def __post_init__(self):
        self.root = pathlib.Path(self.root)
        self.root.mkdir(parents=True, exist_ok=False)
        self._episode = None
        self._last_frame = None
        self._rows: list[dict] = []
        self._chunks: list[dict[str, list[np.ndarray]]] = []
        self._current = {side: {"depth_m": [], "marker_flow_px": [], "rgb": []} for side in SIDES}

    def append(self, *, episode: int, frame: int, time_s: float, sensors: dict[str, dict]) -> None:
        if set(sensors) != set(SIDES):
            raise ValueError("every tactile frame must contain left and right")
        if self._episode != episode:
            self._episode, self._last_frame = episode, None
        if self._last_frame is not None and frame <= self._last_frame:
            raise ValueError("Photon may be updated only once per increasing frame")
        if not np.isfinite(time_s):
            raise ValueError("time_s must be finite")
        row = {"episode": int(episode), "frame": int(frame), "time_s": float(time_s), "sensors": {}}
        for side in SIDES:
            item = sensors[side]
            # Photon exposes views backed by vendor C++/OpenGL buffers.  A
            # chunk must own its arrays: releasing a list of borrowed views
            # after np.savez can otherwise invalidate the live renderer and
            # crash the next frame in native code.
            depth = np.array(item["depth_m"], copy=True)
            rgb = np.array(item["rgb"], copy=True)
            flow = np.array(item["marker_flow_px"], copy=True)
            if depth.shape != PHOTON_DEPTH_SHAPE or rgb.shape != PHOTON_RGB_SHAPE:
                raise ValueError(
                    f"{side} Photon output shape mismatch: "
                    f"depth={depth.shape}, expected={PHOTON_DEPTH_SHAPE}; "
                    f"rgb={rgb.shape}, expected={PHOTON_RGB_SHAPE}; "
                    f"flow={flow.shape}"
                )
            if flow.shape[-1:] != (2,) or not all(np.isfinite(x).all() for x in (depth, rgb, flow)):
                raise ValueError(f"{side} Photon output is invalid")
            self._current[side]["depth_m"].append(depth)
            self._current[side]["rgb"].append(rgb)
            self._current[side]["marker_flow_px"].append(flow)
            availability = item.get("force_availability", {})
            row["sensors"][side] = {
                "valid": True,
                "contact_object_ids": list(item.get("contact_object_ids", [])),
                "relative_motion_m": json_safe(item.get("relative_motion_m", [0, 0, 0])),
                "T_world_sensor": json_safe(item["T_world_sensor"]),
                "forces": json_safe(item.get("forces", {})),
                "force_availability": {str(k): bool(v) for k, v in availability.items()},
            }
        self._rows.append(row)
        self._last_frame = frame
        if len(self._current["left"]["depth_m"]) >= self.config["chunk_frames"]:
            self._flush()

    def _flush(self):
        if not self._current["left"]["depth_m"]:
            return
        index = len(self._chunks)
        record = {}
        for side in SIDES:
            directory = self.root / side
            directory.mkdir(exist_ok=True)
            path = directory / f"chunk_{index:06d}.npz"
            np.savez_compressed(path, **{k: np.stack(v) for k, v in self._current[side].items()})
            record[side] = str(path.relative_to(self.root))
            self._current[side] = {"depth_m": [], "marker_flow_px": [], "rgb": []}
        self._chunks.append(record)

    def finalize(self, provenance: dict, *, status: str = "complete", error: str | None = None) -> dict:
        self._flush()
        manifest = {
            "schema_version": "1.0", "status": status, "sensor": "photon",
            "contact_backend": self.config["contact_backend"], "engine": self.config["engine"],
            "calibration_status": "uncalibrated", "units": {
                "time": "s", "pose_translation": "m", "depth": "m",
                "marker_flow": "pixel", "contact_force": "N", "mount_reaction": "N",
            },
            "shapes": {"depth": list(PHOTON_DEPTH_SHAPE), "rgb": list(PHOTON_RGB_SHAPE)},
            "frames": self._rows, "chunks": self._chunks, "provenance": provenance,
        }
        if error is not None:
            manifest["error"] = str(error)
        manifest = json_safe(manifest)
        (self.root / "tactile_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        (self.root / "config_snapshot.json").write_text(json.dumps(json_safe(self.config), indent=2) + "\n")
        return manifest
