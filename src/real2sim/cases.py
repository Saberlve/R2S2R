"""Resumable, configuration-driven offline visual workflow. No hardware commands."""
import hashlib
import json
import math
import os
from pathlib import Path
from datetime import datetime, timezone
import re
import subprocess
import sys
import uuid
from .contracts import load, sha
from .cli import normalize_command, allowlisted


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def tree(path):
    p = Path(path)
    if p.is_symlink():
        raise ValueError('Symlink input/output unsupported: ' + str(p))
    if p.is_file():
        return sha(p)
    if not p.is_dir():
        raise FileNotFoundError(p)
    return {str(f.relative_to(p)): tree(f) for f in sorted(p.rglob('*'))
            if f.is_file() or f.is_symlink()}


def expand(value, ctx):
    if isinstance(value, str):
        return re.sub(r'\{([a-zA-Z0-9_.-]+)\}', lambda m: str(ctx[m[1]]), value)
    if isinstance(value, list):
        return [expand(v, ctx) for v in value]
    if isinstance(value, dict):
        return {k: expand(v, ctx) for k, v in value.items()}
    return value


def intake(case):
    data = load(case / 'intake.json')
    missing, warnings, paths = [], [], []
    def require(path, label):
        if not path or not (case / path).is_file():
            missing.append(label)
        else:
            paths.append((case / path).resolve())
    cameras = data.get('cameras', [])
    if not cameras:
        missing.append('At least one fixed camera with a raw reference image')
    for c in cameras:
        require(c.get('image'), 'camera image: ' + c.get('id', '?'))
        if not c.get('native_wh') or c.get('pixel_ops') is None:
            warnings.append('camera resolution/crop pipeline not yet verified: ' + c.get('id', '?'))
        if not c.get('intrinsics_file'):
            warnings.append('intrinsics unknown; needs the image_fitted path and independent validation: ' + c.get('id', '?'))
        else:
            require(c['intrinsics_file'], 'intrinsics file')
    require(data.get('room_video'), 'room walkthrough video')
    scans = data.get('scanner_exports', [])
    if not scans:
        missing.append('Scanner raw export package with model and texture')
    for s in scans:
        require(s, 'Scanner export package: ' + s)
    dims = data.get('table_size_m')
    if not isinstance(dims, list) or len(dims) != 3 or not all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v > 0 for v in dims):
        missing.append('table_size_m (table length, width and height) in metres')
    if not data.get('anchors'):
        warnings.append('no distance anchors from the table to walls or door frames; the background layout may be ambiguous')
    return {'missing': missing, 'warnings': warnings,
            'raw_sha256': {str(p): sha(p) for p in paths}}


def init(case, cycles_python):
    case = Path(case).resolve()
    case.mkdir(parents=True, exist_ok=False)
    for name in ['raw', 'config', 'assets', 'runs']:
        (case / name).mkdir()
    atomic(case / 'intake.json', {'schema_version': '1.0', 'table_size_m': [None, None, None],
        'room_video': '', 'scanner_exports': [], 'anchors': [],
        'cameras': [{'id': 'CameraB', 'image': '', 'native_wh': None,
                     'pixel_ops': None, 'intrinsics_file': None}], 'excluded_entities': []})
    atomic(case / 'runtime.json', {'cycles_python': cycles_python, 'cuda_visible_devices': ''})
    stages = [
        {'id': 'intake', 'kind': 'intake', 'requires': []},
        {'id': 'scene_ready', 'kind': 'files', 'requires': ['intake'],
         'inputs': ['{case}/config/scene.json', '{case}/assets', '{case}/config/evidence.md'],
         'instruction': 'Agent: prepare independent assets, measured scene, scanner registration and camera fitting evidence. Do not modify common code.'},
        {'id': 'validate', 'kind': 'command', 'requires': ['scene_ready'],
         'argv': ['validate', '{case}/config/scene.json'], 'outputs': []},
        {'id': 'build', 'kind': 'command', 'requires': ['validate'],
         'argv': ['build', '--scene', '{case}/config/scene.json', '--python', '{runtime.cycles_python}', '--out', '{out}/scene.blend'],
         'outputs': ['scene.blend']},
        {'id': 'audit', 'kind': 'command', 'requires': ['build'],
         'argv': ['audit', '--scene', '{case}/config/scene.json', '--blend', '{stage.build}/scene.blend', '--python', '{runtime.cycles_python}', '--out', '{out}/audit.json'],
         'outputs': ['audit.json']},
        {'id': 'render', 'kind': 'command', 'requires': ['audit'],
         'argv': ['render', '--scene', '{case}/config/scene.json', '--blend', '{stage.build}/scene.blend', '--python', '{runtime.cycles_python}', '--samples', '16', '--out', '{out}/images'],
         'outputs': ['images']},
        {'id': 'review', 'kind': 'review', 'requires': ['render'],
         'instruction': 'Review camera alignment, scanner seams, independent geometry, exposure and evidence.md. This approves a visual baseline only.'},
        {'id': 'freeze', 'kind': 'command', 'requires': ['review'],
         'argv': ['freeze', '--scene', '{case}/config/scene.json', '--blend', '{stage.build}/scene.blend', '--out', '{out}/baseline'],
         'outputs': ['baseline']}
    ]
    atomic(case / 'workflow.json', {'schema_version': '1.0', 'stages': stages})
    (case / 'AGENT_HANDOFF.md').write_text(
        'Run case-run and read runs/status.json and TODO.md. Fill intake from original evidence; never invent missing measurements. '
        'Prepare config/scene.json, assets and config/evidence.md using repository scanner and camera fitting procedures. '
        'Add configured stages to workflow.json for scan-bake, fit-camera and fit-appearance as needed. '
        'Declare all external input files/directories in inputs; use absolute paths in referenced configs. '
        'Do not change shared code per scene. Do not capture or move hardware. Do not approve on behalf of a human. '
        'Unknown intrinsics remain image_fitted. Missing data, uncertainty and excluded entities must be recorded.\n', encoding='utf-8')
    return case


def run(case, retry_failed=False):
    case = Path(case).resolve()
    runs = case / 'runs'
    runs.mkdir(exist_ok=True)
    lock = runs / '.lock'
    fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    os.write(fd, str(os.getpid()).encode())
    os.close(fd)
    try:
        return _run(case, retry_failed)
    finally:
        lock.unlink()


def _run(case, retry_failed):
    runs = case / 'runs'
    runtime, workflow = load(case / 'runtime.json'), load(case / 'workflow.json')
    if workflow.get('schema_version') != '1.0':
        raise ValueError('Unsupported workflow version')
    old = load(runs / 'status.json') if (runs / 'status.json').exists() else {'stages': {}}
    state = {'status': 'running', 'stages': {}}
    ctx = {'case': str(case), **{'runtime.' + k: v for k, v in runtime.items()}}
    code = tree(Path(__file__).parent)
    # Exclude interpreter-generated caches from provenance/cache keys.
    code = {k: v for k, v in code.items() if '__pycache__' not in k and not k.endswith('.pyc')}
    seen = set()
    ancestors = {}
    def persist(message=''):
        atomic(runs / 'status.json', state)
        (case / 'TODO.md').write_text(message + '\n', encoding='utf-8')
    for s in workflow['stages']:
        sid = s['id']
        if not re.fullmatch('[a-zA-Z0-9_-]+', sid) or sid in seen or any(d not in seen for d in s['requires']):
            raise ValueError('IDs must be unique; dependencies must precede stage: ' + sid)
        if s['kind'] not in {'intake', 'files', 'command', 'review'}:
            raise ValueError('Unknown stage kind: ' + s['kind'])
        if s['kind'] == 'command' and (not isinstance(s.get('argv'), list) or not isinstance(s.get('outputs'), list)):
            raise ValueError('Command requires argv and outputs arrays')
        ancestors[sid] = set(s['requires'])
        for d in s['requires']:
            ancestors[sid].update(ancestors[d])
        references = re.findall(r'\{stage\.([a-zA-Z0-9_-]+)\}', json.dumps(s))
        template_sources = expand(s.get('templates', {}), ctx)
        for source in template_sources.values():
            if Path(source).is_file():
                references += re.findall(r'\{stage\.([a-zA-Z0-9_-]+)\}', Path(source).read_text(encoding='utf-8'))
        if any(ref not in ancestors[sid] for ref in references):
            raise ValueError('stage placeholder must name an ancestor: ' + sid)
        seen.add(sid)
        deps = {d: {k: state['stages'][d].get(k) for k in ['fingerprint', 'attempt', 'output_hashes']} for d in s['requires']}
        templates = expand(s.get('templates', {}), ctx)
        paths = [Path(p) for p in [*expand(s.get('inputs', []), ctx), *templates.values()]]
        if any(not p.is_absolute() for p in paths):
            raise ValueError('Use {case} or absolute input/template paths')
        missing = [str(p) for p in paths if not p.exists()]
        report = intake(case) if s['kind'] == 'intake' else None
        if report:
            missing += report['missing']
        fingerprint = digest({'stage': s, 'runtime': runtime, 'code': code, 'deps': deps,
                              'inputs': {str(p): tree(p) for p in paths if p.exists()},
                              'intake': [load(case / 'intake.json'), report] if report else None})
        row = {'fingerprint': fingerprint, 'kind': s['kind']}
        state['stages'][sid] = row
        if missing:
            row.update(status='needs_user' if s['kind'] == 'intake' else 'needs_agent', missing=missing)
            state['status'] = row['status']; persist(sid + ': ' + '\n'.join(missing) + '\n' + s.get('instruction', ''))
            return state
        prev = old['stages'].get(sid, {})
        valid = prev.get('fingerprint') == fingerprint
        if valid and prev.get('status') == 'passed' and s['kind'] != 'review':
            try:
                valid = all(tree(p) == h for p, h in prev.get('output_hashes', {}).items())
            except FileNotFoundError:
                valid = False
            if valid:
                state['stages'][sid] = {**prev, 'cached': True}
                if prev.get('attempt'):
                    ctx['stage.' + sid] = prev['attempt']
                continue
        if valid and prev.get('status') == 'failed' and not retry_failed:
            state['stages'][sid] = prev
            state['status'] = 'failed'; persist(sid + ': inspect log, fix configuration or use --retry-failed')
            return state
        if s['kind'] == 'review':
            decision = runs / 'reviews' / (sid + '.json')
            approval = load(decision) if decision.exists() else {}
            if approval.get('token') != fingerprint or approval.get('decision') != 'approve':
                row.update(status='needs_review', token=fingerprint, instruction=s.get('instruction', ''),
                           artifacts={k: v for k, v in ctx.items() if k.startswith('stage.')})
                state['status'] = 'needs_review'; persist(sid + ': ' + s.get('instruction', '') + '\nReview token: ' + fingerprint)
                return state
            row['approval'] = approval
        elif s['kind'] == 'command':
            attempt = runs / sid / uuid.uuid4().hex
            attempt.mkdir(parents=True)
            row['attempt'] = str(attempt)
            ctx['stage.' + sid] = str(attempt)
            try:
                local = {**ctx, 'out': str(attempt)}
                for name, source in templates.items():
                    if not re.fullmatch('[a-zA-Z0-9_-]+', name):
                        raise ValueError('Invalid template name')
                    target = attempt / 'configs' / (name + '.json')
                    atomic(target, expand(load(source), local))
                    local['template.' + name] = str(target)
                args = normalize_command(expand(s['argv'], local))
                if not allowlisted(args):
                    raise ValueError('Command not allowlisted')
                if '--out' in args:
                    dest = Path(args[args.index('--out') + 1]).resolve()
                    if not dest.is_relative_to(attempt) or dest == attempt:
                        raise ValueError('--out must be a child of the fresh attempt')
                elif args[:2] != ['scene', 'validate']:
                    raise ValueError('Command requires --out')
                outputs = [(attempt / p).resolve() for p in s.get('outputs', [])]
                if any(not p.is_relative_to(attempt) or p == attempt for p in outputs):
                    raise ValueError('Output escapes attempt directory')
                env = os.environ.copy()
                env['CUDA_VISIBLE_DEVICES'] = runtime.get('cuda_visible_devices', '')
                cmd = [sys.executable, '-m', 'real2sim.cli', *args]
                atomic(attempt / 'command.json', {'argv': cmd, 'fingerprint': fingerprint})
                with (attempt / 'stage.log').open('w', encoding='utf-8') as log:
                    result = subprocess.run(cmd, cwd=case, env=env, stdout=log, stderr=subprocess.STDOUT)
                if result.returncode:
                    raise RuntimeError('Exit code ' + str(result.returncode))
                row['output_hashes'] = {str(p): tree(p) for p in [*outputs, attempt / 'stage.log']}
            except Exception as e:
                row.update(status='failed', error=str(e))
                state['status'] = 'failed'; persist(sid + ': ' + str(e) + '\n' + str(attempt / 'stage.log'))
                return state
        elif s['kind'] not in {'intake', 'files'}:
            raise ValueError('Unknown stage kind: ' + s['kind'])
        row['status'] = 'passed'
        if report:
            row['report'] = report
        persist()
    state['status'] = 'complete'; persist('Complete. See runs/status.json for current artifacts and provenance.')
    return state


def review(case, stage, token, decision, reviewer, note):
    case = Path(case).resolve()
    if (case / 'runs/.lock').exists():
        raise RuntimeError('Workflow is running')
    state = load(case / 'runs/status.json')
    row = state['stages'][stage]
    if row.get('status') != 'needs_review' or row.get('token') != token:
        raise ValueError('Not the pending review token; rerun case-run first')
    atomic(case / 'runs/reviews' / (stage + '.json'),
           {'token': token, 'decision': decision, 'reviewer': reviewer, 'note': note, 'created_utc': datetime.now(timezone.utc).isoformat()})
