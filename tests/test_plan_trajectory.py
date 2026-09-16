import numpy as np
import pytest

from real2sim.traj.planner import (
    Infeasible, JointTrajectory, Waypoint, interpolate_waypoints,
    quintic_scale, validate_limits, validate_task,
)

START = {"position_m": [0.4, 0.0, 0.5], "quaternion_xyzw": [0, 0, 0, 1],
         "gripper": 0.0, "duration_s": 1.0}
TARGET = {"position_m": [0.5, 0.1, 0.6], "quaternion_xyzw": [0, 0, 0.0, 1],
          "gripper": 1.0, "duration_s": 2.0}
TASK = {"schema_version": "1.0", "waypoints": [START, TARGET], "fps": 30}


def test_task_validation_accepts_and_normalizes():
    task = validate_task(TASK)
    assert len(task["waypoints"]) == 2
    assert task["fps"] == 30
    assert task["waypoints"][1].gripper == 1.0
    with pytest.raises(ValueError):
        validate_task({**TASK, "schema_version": "2.0"})
    with pytest.raises(ValueError):
        validate_task({**TASK, "waypoints": []})
    with pytest.raises(ValueError):
        validate_task({**TASK, "waypoints": [{**START, "quaternion_xyzw": [0, 0, 0, 2]}]})
    with pytest.raises(ValueError):
        validate_task({**TASK, "waypoints": [{**START, "gripper": 1.5}]})
    with pytest.raises(ValueError):
        validate_task({**TASK, "waypoints": [{**START, "duration_s": 0}]})
    with pytest.raises(ValueError):
        validate_task({**TASK, "waypoints": [{**START, "position_m": [0.4, float("nan"), 0.5]}]})
    with pytest.raises(ValueError):
        validate_task({**TASK, "fps": 0})


def test_quintic_scale_endpoints_and_monotonicity():
    t = np.linspace(0, 1, 101)
    s = quintic_scale(t)
    assert s[0] == pytest.approx(0.0)
    assert s[-1] == pytest.approx(1.0)
    assert np.all(np.diff(s) >= -1e-12)


def test_interpolation_hits_endpoints_and_fps():
    task = validate_task(TASK)
    times, positions, quats, grippers = interpolate_waypoints(task["waypoints"], task["fps"])
    assert times[0] == 0.0 and np.all(np.diff(times) > 0)
    assert times[-1] == pytest.approx(2.0, abs=1.0 / 30 + 1e-6)
    np.testing.assert_allclose(positions[0], START["position_m"], atol=1e-12)
    np.testing.assert_allclose(positions[-1], TARGET["position_m"], atol=1e-12)
    np.testing.assert_allclose(quats[-1], TARGET["quaternion_xyzw"], atol=1e-12)
    assert grippers[0] == pytest.approx(0.0) and grippers[-1] == pytest.approx(1.0)
    assert len(times) == max(2, int(round(2.0 * 30)) + 1)
    with pytest.raises(ValueError):
        interpolate_waypoints(task["waypoints"][:1], 30)


def test_interpolation_uses_uniform_time_grid():
    # Short segments still keep a strict 1/fps frame gap after quantization (durations snap to whole frames)
    task = validate_task({**TASK, "waypoints": [
        START,
        {**TARGET, "duration_s": 0.05},
        {**TARGET, "duration_s": 0.04},
    ]})
    times, _, _, _ = interpolate_waypoints(task["waypoints"], 30)
    np.testing.assert_allclose(np.diff(times), 1.0 / 30, rtol=0, atol=1e-12)
    assert len(times) == 1 + 2 + 1  # 0.05s -> 2 frames, 0.04s -> 1 frame
    assert times[-1] == pytest.approx(3.0 / 30)


def _traj(times, positions):
    return JointTrajectory(times=np.asarray(times, float), positions=np.asarray(positions, float),
                           joint_names=("j1", "j2"))


def test_validate_limits_pass_and_violation():
    times = np.arange(101) / 100.0
    smooth = np.column_stack([np.sin(times), np.cos(times)])
    assert validate_limits(_traj(times, smooth), [2.0, 2.0], [50.0, 50.0]) == []
    spike = smooth.copy()
    spike[50, 0] += 1.0
    issues = validate_limits(_traj(times, spike), [2.0, 2.0])
    assert issues and "velocity" in issues[0]
    with pytest.raises(ValueError):
        validate_limits(_traj(times, smooth), [2.0])
    assert validate_limits(_traj([0.0, 0.0], [[0, 0], [0, 0]]), [1.0, 1.0]) != []


def test_infeasible_is_a_result_not_an_exception():
    refusal = Infeasible("no_ik", "detail")
    assert refusal.reason == "no_ik"
