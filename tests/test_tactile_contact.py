import json

import numpy as np
import pytest

from real2sim.tactile.run import (
    PHOTON_GEL_SIZE_M,
    TactileRunWriter,
    json_safe,
    resample_trajectory,
    validate_tactile_config,
)


def config(backend="hydroelastic"):
    return {
        "schema_version": "1.0", "sensor": "photon", "contact_backend": backend,
        "mounts": {
            "left": {"body": "left_finger", "T_body_sensor": np.eye(4).tolist()},
            "right": {"body": "right_finger", "T_body_sensor": np.eye(4).tolist()},
        },
        "existing_contact_face_size_m": [0.0165, 0.02834],
        "physics": {"preset": "uncalibrated"},
        "photon": {"preset": "vendor", "outputs": ["rgb", "depth", "marker_flow"]},
        "parameters_source": {"physics": "engineering preset", "photon": "vendor bundle"},
        "calibration_status": "uncalibrated", "control_hz": 20, "chunk_frames": 2,
    }


def test_dual_mount_units_size_and_backend_contract():
    cfg = validate_tactile_config(config(), engine="mujoco")
    assert cfg["engine"] == "mujoco" and cfg["cuda_graph"] is False
    assert cfg["photon_gel_size_m"] == list(PHOTON_GEL_SIZE_M)
    assert np.allclose(cfg["contact_face_size_delta_m"], [0.0008, 0.0008])
    with pytest.raises(ValueError, match="requires engine"):
        validate_tactile_config(config(), engine="mujoco_fast")
    bad = config(); bad["mounts"]["right"]["body"] = ""
    with pytest.raises(ValueError): validate_tactile_config(bad)
    bad = config("unsupported")
    with pytest.raises(ValueError, match="hydroelastic"): validate_tactile_config(bad)


def test_nonuniform_input_is_explicitly_resampled():
    arrays = {"time": np.array([0.0, .03, .11]), "q": np.array([[0.], [3.], [11.]])}
    out, report = resample_trajectory(arrays, 20)
    assert report == {"input_frames": 3, "output_frames": 4, "input_uniform": False, "resampled": True, "control_hz": 20.0}
    assert np.allclose(out["time"], [0, .05, .1, .11])
    assert np.allclose(out["q"].ravel(), [0, 5, 10, 11])


def sensor_frame(value=0):
    return {
        "depth_m": np.full((100, 64), value, np.float32),
        "rgb": np.full((700, 400, 3), value, np.uint8),
        "marker_flow_px": np.zeros((2, 220, 2), np.float32),
        "T_world_sensor": np.eye(4), "contact_object_ids": ["bar"],
        "forces": {"contact_force_N": [1, 0, 0]},
        "force_availability": {"contact_force_N": True, "mount_reaction_N": False},
    }


def test_writer_separates_sides_chunks_and_rejects_duplicate_update(tmp_path):
    cfg = validate_tactile_config(config())
    writer = TactileRunWriter(tmp_path / "run", cfg)
    writer.append(episode=0, frame=0, time_s=0, sensors={"left": sensor_frame(1), "right": sensor_frame(2)})
    with pytest.raises(ValueError, match="only once"):
        writer.append(episode=0, frame=0, time_s=0, sensors={"left": sensor_frame(), "right": sensor_frame()})
    writer.append(episode=0, frame=1, time_s=.05, sensors={"left": sensor_frame(3), "right": sensor_frame(4)})
    manifest = writer.finalize({"trajectory_sha256": "abc"})
    assert manifest["frames"][0]["sensors"]["left"]["force_availability"]["mount_reaction_N"] is False
    left = np.load(tmp_path / "run" / manifest["chunks"][0]["left"])
    right = np.load(tmp_path / "run" / manifest["chunks"][0]["right"])
    assert left["rgb"][0, 0, 0, 0] == 1 and right["rgb"][0, 0, 0, 0] == 2
    assert "NaN" not in (tmp_path / "run" / "tactile_manifest.json").read_text()


def test_json_never_encodes_missing_force_as_nan():
    with pytest.raises(ValueError, match="NaN"):
        json_safe({"force": float("nan")})
