import json
import pathlib

import numpy as np
import pytest

from real2sim import cli
from real2sim.align import base, wrist
from real2sim.scene import spatial
from real2sim.scene.modeling import interface as modeling
from real2sim.tactile import interface as tactile

ROOT = pathlib.Path(__file__).resolve().parents[1]
MINIMAL = ROOT / 'examples' / 'minimal' / 'scene.json'


def test_legacy_verbs_normalize_to_grouped_form():
    assert cli.normalize_command(['scan-bake', 'cfg.json']) == ['scene', 'scan-bake', 'cfg.json']
    assert cli.normalize_command(['calibrate', '--scene', 's']) == ['align', 'calibrate', '--scene', 's']
    assert cli.normalize_command(['xarm7', 'generate_grasp']) == ['traj', 'xarm7', 'generate_grasp']
    assert cli.normalize_command(['convert', '--input', 'x']) == ['traj', 'convert', '--input', 'x']
    unchanged = ['scene', 'validate', 's']
    assert cli.normalize_command(unchanged) == unchanged
    assert cli.normalize_command(['freeze', '--scene', 's']) == ['freeze', '--scene', 's']


def test_group_verbs_are_unique_across_groups():
    # LEGACY flattens every group's verbs into one mapping, so a verb declared by two groups
    # would silently rebind the earlier group's flat alias (e.g. `r2s validate`).
    seen = {}
    for group, verbs in cli.GROUP_COMMANDS.items():
        for verb in verbs:
            assert verb not in seen, '%s is declared by both %s and %s' % (verb, seen.get(verb), group)
            seen[verb] = group


def test_allowlist_accepts_grouped_and_top_level_only():
    assert cli.allowlisted(['scene', 'render'])
    assert cli.allowlisted(['align', 'calibrate'])
    assert cli.allowlisted(['traj', 'convert'])
    assert cli.allowlisted(['asset', 'check'])
    assert cli.allowlisted(['freeze'])
    assert not cli.allowlisted(['render'])
    assert not cli.allowlisted(['scene', 'draft-model'])
    assert not cli.allowlisted(['tactile', 'describe'])
    assert not cli.allowlisted(['asset', 'unknown'])
    assert not cli.allowlisted(['scene', 'unknown'])
    assert not cli.allowlisted([])


def test_legacy_and_grouped_validate_produce_identical_output(capsys):
    cli.main(['scene', 'validate', str(MINIMAL)])
    grouped = json.loads(capsys.readouterr().out)
    cli.main(['validate', str(MINIMAL)])
    captured = capsys.readouterr()
    assert json.loads(captured.out) == grouped
    assert 'Deprecated' in captured.err


def test_spatial_constraint_validation_accepts_and_rejects():
    ids = {'TableTop', 'TestBox'}
    good = [{'entity_a': 'TableTop', 'entity_b': 'TestBox', 'distance_m': 0.5,
             'sigma_m': 0.002, 'evidence': 'raw/measure/photo1.jpg'}]
    assert spatial.validate_constraints(good, ids) == good
    with pytest.raises(ValueError):
        spatial.validate_constraints([{**good[0], 'entity_b': 'Ghost'}], ids)
    with pytest.raises(ValueError):
        spatial.validate_constraints([{**good[0], 'distance_m': -1.0}], ids)
    with pytest.raises(ValueError):
        spatial.validate_constraints([{**good[0], 'sigma_m': float('nan')}], ids)
    with pytest.raises(ValueError):
        spatial.validate_constraints([{**good[0], 'entity_b': 'TableTop'}], ids)
    with pytest.raises(ValueError):
        spatial.validate_constraints([{**good[0], 'evidence': ''}], ids)


def test_reserved_interfaces_raise_not_implemented():
    with pytest.raises(NotImplementedError):
        spatial.optimize_spatial({}, [])
    with pytest.raises(NotImplementedError):
        modeling.build_draft({}, '/tmp/anywhere')


def test_tactile_describe_contract_marks_photon_offline_integration():
    contract = tactile.describe()
    assert contract['schema_version'] == '1.0-draft'
    assert contract['status'] == 'photon_integrated_offline_only'
    backend = contract['sensor']['photon']['backend']
    assert 'tacsim' in backend
    # The invariant, not the wording: tacsim is consumed from the target interpreter, so this
    # repository must not describe itself as carrying a copy.
    assert 'submodule' not in backend
    assert contract['sensor']['mount'].startswith('fixed')
    assert any('tacsim' in b for b in contract['backends_planned'])


def test_modeling_contract_marks_drafts_estimated_only():
    contract = modeling.describe()
    assert contract['status'] == 'reserved_not_implemented'
    assert 'estimated' in contract['notes']


def test_base_check_validates_scene_config():
    cfg = {
        'T_sim_base': np.eye(4).tolist(),
        'tcp_offset_m': [0, 0, 0.172],
        'table_matrix': np.eye(4).tolist(),
        'table_size_m': [1.2, 0.8, 0.75],
        'bar': {'position_m': [0.4, 0.0, 0.76], 'quaternion_xyzw': [0, 0, 0, 1],
                'size_m': [0.3, 0.02, 0.02], 'friction': 0.6},
    }
    report = base.validate_scene_config(cfg)
    assert report['valid'] is True
    bad = np.eye(4).tolist()
    bad[0][0] = 2.0
    with pytest.raises(ValueError):
        base.validate_scene_config({**cfg, 'T_sim_base': bad})
    with pytest.raises(ValueError):
        base.validate_scene_config({k: v for k, v in cfg.items() if k != 'bar'})
    with pytest.raises(ValueError):
        base.validate_scene_config({**cfg, 'bar': {**cfg['bar'], 'quaternion_xyzw': [0, 0, 0, 2]}})


def test_controller_snapshot_core_converts_mm_and_validates():
    raw = {
        'end_transform': np.eye(4).tolist(),
        'joint_origins': [[0, 0, 0.1, 0, 0, 0]],
        'world_offset': [0, 0, 0, 0, 0, 0],
    }
    raw['end_transform'][2][3] = 172.0
    cal = base.snapshot_to_calibration(raw)
    assert cal['T_flange_tcp'][2][3] == pytest.approx(0.172)
    assert cal['schema_version'] == '1.0'
    assert len(cal['joint_axes']) == 1
    broken = {'end_transform': (np.eye(4) * 2).tolist(), 'joint_origins': [], 'world_offset': []}
    with pytest.raises(ValueError):
        base.snapshot_to_calibration(broken)


def test_wrist_module_composes_tcp_camera_transform():
    measurement = {
        'post_rotation_degrees': 180,
        'T_tcp_camera_cv': [
            [-0.008212, 0.999961, 0.003294, 0.069371],
            [-0.999880, -0.008255, 0.013139, 0.025467],
            [0.013165, -0.003186, 0.999908, -0.158734],
            [0, 0, 0, 1],
        ],
    }
    controller = {'end_transform': np.eye(4).tolist()}
    controller['end_transform'][2][3] = 172.0
    previous = {
        'K': np.eye(3).tolist(), 'resolution': [640, 480],
        'distortion_coeffs': [0] * 5, 'image_orientation': 'raw',
        'T_flange_camera_cv': np.eye(4).tolist(),
    }
    record = wrist.build_record(measurement, controller, previous)
    actual = np.asarray(record['T_flange_camera_cv'])
    assert np.allclose(actual[:3, 3], [0.069371, 0.025467, 0.013266])
    assert record['adoption_status'] == 'candidate_pending_independent_scene_validation'
    bad = np.eye(4)
    bad[0, 0] = 2
    with pytest.raises(ValueError):
        wrist.rigid(bad, 'bad')
