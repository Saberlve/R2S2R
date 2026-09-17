import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

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


def test_nonuniform_input_is_resampled_onto_whole_control_periods():
    arrays = {"time": np.array([0.0, .03, .11]), "q": np.array([[0.], [3.], [11.]])}
    out, report = resample_trajectory(arrays, 20)
    assert report["input_frames"] == 3 and report["output_frames"] == 4
    assert report["input_uniform"] is False and report["resampled"] is True
    assert report["control_hz"] == 20.0 and report["frame_dt_s"] == pytest.approx(.05)
    # A replay advances one whole period per frame, so the grid may not stop mid-period: the
    # trailing 0.04 s is held at the last commanded sample and reported as such.
    assert report["duration_s"] == pytest.approx(.15)
    assert report["trailing_hold_s"] == pytest.approx(.04)
    assert np.allclose(out["time"], [0, .05, .1, .15])
    assert np.allclose(out["q"].ravel(), [0, 5, 10, 11])


def test_input_already_on_the_control_clock_gains_no_extra_frame():
    arrays = {"time": np.arange(4) * .05, "q": np.arange(4.0)[:, None]}
    out, report = resample_trajectory(arrays, 20)
    assert report["resampled"] is False and report["trailing_hold_s"] == 0
    assert np.allclose(out["time"], arrays["time"])


def test_fractional_control_rate_is_refused_not_rounded():
    # cfg.fps is an integer, so 29.97 would be silently replayed as 30 while the recorded
    # timestamps kept the fractional spacing.
    bad = config(); bad["control_hz"] = 29.97
    with pytest.raises(ValueError, match="whole number"):
        validate_tactile_config(bad)
    assert validate_tactile_config({**config(), "control_hz": 60})["control_hz"] == 60


def test_quaternions_are_slerped_not_blended_componentwise():
    # q and -q describe one attitude; a component-wise blend collapses to zero at the midpoint.
    quat = np.array([[0, 0, 0, 1.], [0, 0, 0, -1.]])
    arrays = {"time": np.array([0., 1.]), "tcp_quat_xyzw": quat}
    out, _ = resample_trajectory(arrays, 2)
    assert np.allclose(np.linalg.norm(out["tcp_quat_xyzw"], axis=1), 1)
    assert np.allclose(np.abs(out["tcp_quat_xyzw"]), [[0, 0, 0, 1]] * 3)


def test_slerp_keeps_a_constant_angular_rate():
    quat = np.array([[0, 0, 0, 1.], [0, 0, np.sqrt(.5), np.sqrt(.5)]])  # 0 -> 90 deg about z
    out, _ = resample_trajectory({"time": np.array([0., 1.]), "tcp_quat_xyzw": quat}, 2)
    assert np.allclose(Rotation.from_quat(out["tcp_quat_xyzw"]).magnitude(), [0, np.pi / 4, np.pi / 2])


def test_an_unrecognised_quaternion_field_is_refused():
    arrays = {"time": np.array([0., 1.]), "tcp_quat_wxyz": np.array([[1., 0, 0, 0], [1., 0, 0, 0]])}
    with pytest.raises(ValueError, match="looks like a quaternion"):
        resample_trajectory(arrays, 10)


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


def test_a_failed_run_keeps_its_partial_chunk_and_frame_rows(tmp_path):
    cfg = validate_tactile_config(config())  # chunk_frames=2, so frame 0 is still short
    writer = TactileRunWriter(tmp_path / "run", cfg)
    writer.append(episode=0, frame=0, time_s=0, sensors={"left": sensor_frame(1), "right": sensor_frame(2)})
    manifest = writer.finalize(
        {"trajectory_sha256": "abc"}, status="failed", error="RuntimeError('non-finite at frame 1')"
    )
    assert manifest["status"] == "failed" and "non-finite" in manifest["error"]
    assert [row["frame"] for row in manifest["frames"]] == [0]
    assert np.load(tmp_path / "run" / manifest["chunks"][0]["left"])["rgb"].shape[0] == 1


def test_json_never_encodes_missing_force_as_nan():
    with pytest.raises(ValueError, match="NaN"):
        json_safe({"force": float("nan")})
