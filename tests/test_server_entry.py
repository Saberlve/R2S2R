import importlib.util,json,pathlib
import pytest
spec=importlib.util.spec_from_file_location('server_entry',pathlib.Path(__file__).resolve().parents[1]/'tools/server.py');server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)
def fixture_template(tmp_path):
 source=tmp_path/'original'/'runtime';source.mkdir(parents=True)
 for name in ['Baseline.blend','camera_config.json','render.py']:(source/name).write_text('source evidence')
 (source/'runtime_manifest.json').write_text(json.dumps({'wrist_calibration':'../calibration.json'}));(source.parent/'calibration.json').write_text('{"resolution":[640,480]}')
 return source

def test_template_export_is_self_contained_and_does_not_modify_source(tmp_path):
 source=fixture_template(tmp_path);before={p.name:p.read_bytes() for p in source.iterdir()};dest=tmp_path/'export';server.copy_template(source,dest)
 m=json.loads((dest/'runtime_manifest.json').read_text());assert m['wrist_calibration']=='wrist_calibration.json';assert json.loads((dest/m['wrist_calibration']).read_text())['resolution']==[640,480]
 assert {p.name:p.read_bytes() for p in source.iterdir()}==before
 # Editing exported bytes must not mutate the source via hard links.
 (dest/'Baseline.blend').write_text('changed');assert (source/'Baseline.blend').read_text()=='source evidence'

def test_existing_output_is_not_overwritten(tmp_path):
 source=fixture_template(tmp_path);dest=tmp_path/'export';dest.mkdir();sentinel=dest/'Baseline.blend';sentinel.write_text('keep')
 with pytest.raises(FileExistsError):server.copy_template(source,dest)
 assert sentinel.read_text()=='keep'

def test_missing_declared_calibration_is_not_silently_accepted(tmp_path):
 source=fixture_template(tmp_path);(source.parent/'calibration.json').unlink()
 with pytest.raises(FileNotFoundError):server.copy_template(source,tmp_path/'export')

def scratch_clone(tmp_path,monkeypatch,populated=True):
 """Repoint every repository default at a scratch tree, so a test describes a fresh clone."""
 sub=tmp_path/'Data-MechanicSim'
 (sub/'newton_gen').mkdir(parents=True) if populated else sub.mkdir()
 monkeypatch.setattr(server,'SUBMODULE_NEWTON_PROJECT',sub);monkeypatch.setattr(server,'ROOT',tmp_path)
 for name in ['DEFAULT_MAIN_PYTHON','DEFAULT_CYCLES_PYTHON']:
  exe=tmp_path/name.lower()/'bin/python';exe.parent.mkdir(parents=True);exe.write_text('')
  monkeypatch.setattr(server,name,exe)
 return sub

def test_a_prepared_clone_needs_no_environment_variables(tmp_path,monkeypatch):
 sub=scratch_clone(tmp_path,monkeypatch);site=server.site_from_env({})
 assert site['newton_project']==str(sub) and site['runs_root']==str(tmp_path/'runs')
 server.require_site(site,'smoke')  # the claim: clone + `uv sync` twice and the smoke path runs

def test_an_explicit_variable_still_wins(tmp_path,monkeypatch):
 scratch_clone(tmp_path,monkeypatch)
 site=server.site_from_env({'R2S_MAIN_PYTHON':'/custom/python','R2S_NEWTON_PROJECT':'/elsewhere'})
 assert site['main_python']=='/custom/python' and site['newton_project']=='/elsewhere'

def test_an_unbuilt_environment_is_reported_as_missing(tmp_path,monkeypatch):
 scratch_clone(tmp_path,monkeypatch);monkeypatch.setattr(server,'DEFAULT_MAIN_PYTHON',tmp_path/'absent/python')
 site=server.site_from_env({});assert site['main_python'] is None
 with pytest.raises(SystemExit) as exc:server.require_site(site,'smoke')
 assert 'R2S_MAIN_PYTHON' in str(exc.value) and 'uv sync' in str(exc.value)

def test_an_uninitialized_submodule_is_not_taken_as_a_newton_project(tmp_path,monkeypatch):
 sub=scratch_clone(tmp_path,monkeypatch,populated=False);site=server.site_from_env({})
 assert server.newton_project_is_populated(sub) is False and site['newton_project'] is None
 with pytest.raises(SystemExit) as exc:server.require_site(site,'simulate')
 # The empty directory is the non-recursive clone, so the error has to name that remedy.
 assert 'submodule update --init --recursive' in str(exc.value)

def test_smoke_does_not_require_the_lab_case_paths(tmp_path,monkeypatch):
 scratch_clone(tmp_path,monkeypatch);site=server.site_from_env({})
 assert site['measured_case'] is None and site['reference_template'] is None and site['ffmpeg'] in (None,server.shutil.which('ffmpeg'))
 server.require_site(site,'smoke')
 # The commands that do read them still have to refuse rather than run against nothing.
 with pytest.raises(SystemExit):server.require_site(site,'render')
 with pytest.raises(SystemExit):server.require_site(site,'simulate')

def test_the_example_env_file_only_sets_paths_that_have_no_default():
 # Copying the template verbatim is what the README tells people to do, and an explicit value
 # always wins -- so an uncommented placeholder here would silently clobber a working default.
 text=(pathlib.Path(__file__).resolve().parents[1]/'examples/site.example.env').read_text()
 exported={line.split('=')[0].replace('export','').strip() for line in text.splitlines() if line.startswith('export')}
 assert exported=={'R2S_MEASURED_CASE','R2S_REFERENCE_TEMPLATE'}

def test_simulate_executes_repository_adapter_not_frozen_case_copy():
 text=(pathlib.Path(__file__).resolve().parents[1]/'tools/server.py').read_text()
 assert "adapter=ROOT/'src/real2sim/traj/adapters/xarm7/simulate_replay.py'" in text
 assert "run([py,case/'src/simulate_replay.py'" not in text

def test_current_robot_is_materialized_with_local_meshes(tmp_path):
 template=tmp_path/'run'/'runtime';template.mkdir(parents=True)
 robot=template.parent/'robot.urdf';mesh=template.parent/'finger.stl'
 mesh.write_bytes(b'mesh')
 robot.write_text('<robot name="r"><link name="finger"><collision name="c"><geometry><mesh filename="finger.stl"/></geometry></collision></link></robot>')
 (template/'runtime_manifest.json').write_text(json.dumps({'custom_g2':{'physics_urdf':'../robot.urdf'}}))
 case=tmp_path/'case';(case/'inputs').mkdir(parents=True)
 (case/'inputs/previous_unstamped_collision_cache.urdf').write_text('legacy')
 target=server.copy_current_physics_robot(template,case)
 assert target==case/'inputs/xarm7_calibrated.urdf'
 text=target.read_text();assert 'current_robot/' in text and str(mesh) not in text
 assert not (case/'inputs/previous_unstamped_collision_cache.urdf').exists()

def test_simulate_never_runs_frozen_renderer_or_default_old_trajectory():
 text=(pathlib.Path(__file__).resolve().parents[1]/'tools/server.py').read_text()
 assert "run([bpy,ROOT/'src/real2sim/traj/adapters/xarm7/render_motion.py'" in text
 assert "run([bpy,case/'src/render_motion.py'" not in text
 assert "else case/'results/grasp_clearance.npz'" not in text
