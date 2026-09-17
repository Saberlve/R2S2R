"""The lift gate is a duration, not a frame count: the control rate is configurable."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real2sim.traj.replay_metrics import bar_lift_report, longest_hold_s, playback_fps


def held_run(height_m, hold_s, hz, reach_s=0.5):
    """A run whose bar reaches height_m after reach_s and stays there for hold_s."""
    frames = int(round((reach_s + hold_s) * hz)) + 1
    time = np.arange(frames) / hz
    bar = np.zeros((frames, 7))
    bar[:, 2] = np.where(time >= reach_s - 1e-9, height_m, 0.0)
    return bar, time


def test_a_two_second_hold_is_accepted_at_a_low_control_rate():
    # 2.0 s at 20 Hz is 40 frames; a fixed 60-frame window would have rejected a real hold.
    bar, time = held_run(0.06, 2.0, 20)
    report = bar_lift_report(bar, np.eye(4), time)
    assert report["held_5cm_for_2s"] is True
    assert report["longest_clearance_hold_s"] == pytest.approx(2.0)
    assert report["max_bar_lift_m"] == pytest.approx(0.06)


def test_a_one_second_hold_is_not_accepted_at_a_high_control_rate():
    # 1.0 s at 60 Hz is 60 frames; a fixed 60-frame window would have called that a 2 s hold.
    bar, time = held_run(0.06, 1.0, 60)
    report = bar_lift_report(bar, np.eye(4), time)
    assert report["held_5cm_for_2s"] is False
    assert report["longest_clearance_hold_s"] == pytest.approx(1.0)


def test_height_below_clearance_never_counts_as_held():
    bar, time = held_run(0.04, 5.0, 30)
    report = bar_lift_report(bar, np.eye(4), time)
    assert report["held_5cm_for_2s"] is False
    assert report["longest_clearance_hold_s"] == 0.0


def test_two_short_lifts_do_not_add_up_to_one_hold():
    time = np.arange(0.0, 3.5, 1 / 50)
    bar = np.zeros((len(time), 7))
    bar[(time >= 0.5) & (time < 2.0), 2] = 0.06  # 1.48 s held
    bar[time >= 2.5, 2] = 0.06  # 0.98 s held again, after the bar sat back down
    report = bar_lift_report(bar, np.eye(4), time)
    assert report["longest_clearance_hold_s"] == pytest.approx(1.48)
    assert report["held_5cm_for_2s"] is False


def test_lift_is_projected_on_the_measured_table_normal():
    table = np.eye(4)
    table[:3, :3] = Rotation.from_euler("x", 30, degrees=True).as_matrix()
    normal = table[:3, 2]
    time = np.arange(121) / 30
    bar = np.zeros((len(time), 7))
    bar[:, :3] = np.where(time[:, None] >= 0.5 - 1e-9, normal * 0.06, 0.0)
    report = bar_lift_report(bar, table, time)
    assert report["max_bar_lift_m"] == pytest.approx(0.06)
    # The world-Z displacement is shorter than the lift along the table normal.
    assert abs(bar[-1, 2]) == pytest.approx(0.06 * np.cos(np.radians(30)))
    assert report["held_5cm_for_2s"] is True


def test_thresholds_are_reported_with_the_verdict():
    bar, time = held_run(0.03, 0.1, 30)
    report = bar_lift_report(bar, np.eye(4), time, clearance_m=0.02, hold_s=0.05)
    assert report["lift_clearance_m"] == 0.02 and report["lift_hold_required_s"] == 0.05
    assert report["held_5cm_for_2s"] is True


def test_longest_hold_measures_seconds_between_time_stamps():
    assert longest_hold_s([True, True, True], [0.0, 1.5, 2.0]) == pytest.approx(2.0)
    assert longest_hold_s([False, True, True, False, True], [0, 1, 2, 3, 4]) == pytest.approx(1.0)
    assert longest_hold_s([], []) == 0.0


def test_playback_rate_follows_the_recorded_timestamps():
    assert playback_fps(np.arange(331) / 30) == pytest.approx(30.0)
    assert playback_fps(np.arange(331) / 60) == pytest.approx(60.0)


def test_stride_subsamples_without_changing_physical_time():
    # A 60 Hz run rendered every fifth state plays at 12 Hz, not at 30/5: the video has to
    # cover the same number of seconds the replay did.
    assert playback_fps(np.arange(661) / 60, stride=5) == pytest.approx(12.0)
    assert playback_fps(np.arange(331) / 30, stride=15) == pytest.approx(2.0)


def test_a_single_frame_has_no_playback_rate():
    assert playback_fps([0.0]) is None


def test_a_non_uniform_timeline_is_refused_rather_than_averaged():
    with pytest.raises(ValueError, match="uniform"):
        playback_fps([0.0, 0.05, 0.11])
    with pytest.raises(ValueError, match="strictly increasing"):
        playback_fps([0.0, 0.0])
    with pytest.raises(ValueError, match="positive"):
        playback_fps([0.0, 0.1], stride=0)


def test_invalid_input_is_refused_rather_than_guessed():
    with pytest.raises(ValueError, match="one value per frame"):
        longest_hold_s([True, True], [0.0])
    with pytest.raises(ValueError, match="non-decreasing"):
        longest_hold_s([True, True], [1.0, 0.0])
    with pytest.raises(ValueError, match="one pose per frame"):
        bar_lift_report(np.zeros((3, 7)), np.eye(4), np.arange(2.0))
    table = np.eye(4)
    table[:3, 2] *= 2.0
    with pytest.raises(ValueError, match="unit length"):
        bar_lift_report(np.zeros((3, 7)), table, np.arange(3.0))
