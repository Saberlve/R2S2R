import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from real2sim import cases
from real2sim.contracts import load


def fixture(tmp_path):
    case = cases.init(tmp_path / 'case', '/existing/bpy/python')
    (case / 'config/scene.json').write_text('{}')
    cases.atomic(case / 'workflow.json', {'schema_version': '1.0', 'stages': [
        {'id': 'source', 'kind': 'files', 'requires': [], 'inputs': ['{case}/config/scene.json']},
        {'id': 'build', 'kind': 'command', 'requires': ['source'], 'argv': ['build', '--out', '{out}/scene.blend'], 'outputs': ['scene.blend']},
        {'id': 'review', 'kind': 'review', 'requires': ['build']},
        {'id': 'done', 'kind': 'files', 'requires': ['review'], 'inputs': []}]})
    return case


def fake_run(calls):
    def execute(cmd, **kwargs):
        calls.append(cmd)
        Path(cmd[cmd.index('--out') + 1]).write_text('rendered')
        return SimpleNamespace(returncode=0)
    return execute


def test_resume_review_and_invalidation(tmp_path, monkeypatch):
    case = fixture(tmp_path); calls = []
    monkeypatch.setattr(cases.subprocess, 'run', fake_run(calls))
    first = cases.run(case)
    assert first['status'] == 'needs_review' and len(calls) == 1
    token = first['stages']['review']['token']
    assert cases.run(case)['status'] == 'needs_review' and len(calls) == 1
    cases.review(case, 'review', token, 'approve', 'integration_test', 'synthetic only')
    assert cases.run(case)['status'] == 'complete'
    assert cases.run(case)['status'] == 'complete' and len(calls) == 1
    (case / 'config/scene.json').write_text('{"changed":true}')
    changed = cases.run(case)
    assert changed['status'] == 'needs_review' and len(calls) == 2
    with pytest.raises(ValueError):
        cases.review(case, 'review', token, 'approve', 'test', 'stale')


def test_output_tamper_and_missing_input(tmp_path, monkeypatch):
    case = fixture(tmp_path); calls = []
    monkeypatch.setattr(cases.subprocess, 'run', fake_run(calls))
    first = cases.run(case)
    (Path(first['stages']['build']['attempt']) / 'scene.blend').write_text('tamper')
    second = cases.run(case)
    assert len(calls) == 2 and second['stages']['review']['token'] != first['stages']['review']['token']
    (case / 'config/scene.json').unlink()
    assert cases.run(case)['status'] == 'needs_agent'


def test_failure_requires_explicit_retry(tmp_path, monkeypatch):
    case = fixture(tmp_path); calls = []
    def fail(*args, **kwargs):
        calls.append(1); return SimpleNamespace(returncode=1)
    monkeypatch.setattr(cases.subprocess, 'run', fail)
    assert cases.run(case)['status'] == 'failed'
    assert cases.run(case)['status'] == 'failed' and len(calls) == 1
    assert cases.run(case, retry_failed=True)['status'] == 'failed' and len(calls) == 2


def test_escape_and_lock(tmp_path):
    case = fixture(tmp_path)
    w = load(case / 'workflow.json')
    w['stages'][1]['argv'][-1] = '{case}/escaped.blend'
    cases.atomic(case / 'workflow.json', w)
    result = cases.run(case)
    assert result['status'] == 'failed' and not (case / 'escaped.blend').exists()
    (case / 'runs/.lock').write_text('12345')
    with pytest.raises(FileExistsError):
        cases.run(case)


def test_intake_pause(tmp_path):
    case = cases.init(tmp_path / 'fresh', '/bpy')
    result = cases.run(case)
    assert result['status'] == 'needs_user'
    assert 'table_size_m' in (case / 'TODO.md').read_text(encoding='utf-8')
    assert not (case / 'runs/.lock').exists()


def test_directory_inputs_and_template_expansion(tmp_path):
    case = fixture(tmp_path)
    (case / 'assets/a.txt').write_text('original')
    first = cases.tree(case / 'assets')
    (case / 'assets/a.txt').write_text('changed')
    assert cases.tree(case / 'assets') != first
    assert cases.expand('{stage.fit}/scene.json', {'stage.fit': '/run/fitted'}) == '/run/fitted/scene.json'


def test_json_template_materialization(tmp_path, monkeypatch):
    case = fixture(tmp_path)
    cases.atomic(case / 'config/fit.template.json', {'scene': '{case}/config/scene.json'})
    w = load(case / 'workflow.json')
    w['stages'][1]['templates'] = {'fit': '{case}/config/fit.template.json'}
    w['stages'][1]['argv'] = ['fit-camera', '{template.fit}', '--out', '{out}/scene.blend']
    cases.atomic(case / 'workflow.json', w)
    calls = []
    def execute(cmd, **kwargs):
        config = load(cmd[cmd.index('fit-camera') + 1])
        assert config['scene'] == str(case / 'config/scene.json')
        return fake_run(calls)(cmd, **kwargs)
    monkeypatch.setattr(cases.subprocess, 'run', execute)
    assert cases.run(case)['status'] == 'needs_review'
    assert load(case / 'config/fit.template.json')['scene'].startswith('{case}')
    cases.atomic(case / 'config/fit.template.json', {'scene': '{case}/config/scene.json', 'changed': True})
    assert cases.run(case)['status'] == 'needs_review' and len(calls) == 2


def test_undeclared_stage_dependency_rejected(tmp_path):
    case = fixture(tmp_path)
    w = load(case / 'workflow.json')
    w['stages'][1]['argv'] = ['build', '--scene', '{stage.unrelated}/scene.json', '--out', '{out}/scene.blend']
    cases.atomic(case / 'workflow.json', w)
    with pytest.raises(ValueError, match='ancestor'):
        cases.run(case)
    assert not (case / 'runs/.lock').exists()
