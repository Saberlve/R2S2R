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

def simulate_fixture(tmp_path):
 """A measured case and a reference template holding only what `simulate` reads."""
 template=tmp_path/'original'/'runtime';template.mkdir(parents=True)
 for name in ['Baseline.blend','camera_config.json','render.py']:(template/name).write_text('source evidence')
 (template.parent/'finger.stl').write_bytes(b'mesh')
 (template.parent/'robot.urdf').write_text('<robot name="r"><link name="finger"><collision name="c"><geometry><mesh filename="finger.stl"/></geometry></collision></link></robot>')
 (template/'runtime_manifest.json').write_text(json.dumps({'wrist_calibration':'../calibration.json','custom_g2':{'physics_urdf':'../robot.urdf'}}))
 (template.parent/'calibration.json').write_text('{"resolution":[640,480]}')
 case=tmp_path/'measured';(case/'inputs'/'render_baseline').mkdir(parents=True)
 (case/'scene_config.json').write_text('{}')
 return case,template

def drive_simulate(tmp_path,monkeypatch,render_fps=12.0):
 """Run the real `simulate` path and return the argument vector of every command it built."""
 commands=[];case,template=simulate_fixture(tmp_path);out=tmp_path/'out'
 site={'main_python':'/main/python','cycles_python':'/cycles/python','ffmpeg':'/usr/bin/ffmpeg',
       'newton_project':'/newton','measured_case':str(case),'reference_template':str(template),
       'runs_root':str(tmp_path/'runs')}
 def fake_run(cmd,env,log=None):
  argv=[str(c) for c in cmd];commands.append(argv)
  if argv[1].endswith('render_motion.py'):
   # A real render records the rate it derived from the timestamps; this stands in for the GPU run.
   render=pathlib.Path(argv[argv.index('--run')+1])/'render';render.mkdir(parents=True,exist_ok=True)
   (render/'manifest.json').write_text(json.dumps({'render_fps':render_fps}))
 monkeypatch.setattr(server,'run',fake_run)
 monkeypatch.setattr(server,'site_from_env',lambda environ:site)
 monkeypatch.setattr(server,'require_site',lambda site,command:None)
 monkeypatch.setattr(server.sys,'argv',['server.py','simulate','--gpu','0','--out',str(out)])
 server.main()
 return commands,out.resolve()

def test_simulate_executes_repository_adapter_not_frozen_case_copy(tmp_path,monkeypatch):
 commands,out=drive_simulate(tmp_path,monkeypatch)
 adapter=[c for c in commands if c[1].endswith('simulate_replay.py')]
 assert len(adapter)==1
 assert pathlib.Path(adapter[0][1])==server.ROOT/'src/real2sim/traj/adapters/xarm7/simulate_replay.py'
 # Every program this run starts lives in the repository: a case's frozen src/ is evidence and
 # must never shadow the adapter the checkout actually fixes.
 assert all(pathlib.Path(c[1]).is_relative_to(server.ROOT) for c in commands if c[1].endswith('.py'))

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

def test_simulate_never_runs_frozen_renderer_or_default_old_trajectory(tmp_path,monkeypatch):
 commands,out=drive_simulate(tmp_path,monkeypatch)
 renderer=[c for c in commands if c[1].endswith('render_motion.py')]
 assert len(renderer)==1
 assert pathlib.Path(renderer[0][1])==server.ROOT/'src/real2sim/traj/adapters/xarm7/render_motion.py'
 assert renderer[0][renderer[0].index('--run')+1]==str(out/'case/results/eef_mujoco_fast_bar_server')
 # The trajectory is the one this run planned, not a frozen baseline result.
 adapter=[c for c in commands if c[1].endswith('simulate_replay.py')][0]
 assert adapter[adapter.index('--trajectory')+1]==str(out/'case/results/generated_grasp.npz')
 assert not any('grasp_clearance' in arg for c in commands for arg in c)

def test_three_view_video_uses_the_rate_the_render_derived(tmp_path,monkeypatch):
 # 12.0 Hz is 60 Hz control sampled every fifth state -- never the 30/stride this used to assume.
 commands,out=drive_simulate(tmp_path,monkeypatch,render_fps=12.0)
 encode=[c for c in commands if c[0].endswith('ffmpeg')]
 assert len(encode)==1 and encode[0][encode[0].index('-framerate')+1]=='12.0'
 assert encode[0][encode[0].index('-i')+1].endswith('three_views_%06d.png')

def test_a_render_without_a_playback_rate_is_reported_not_guessed(tmp_path,monkeypatch):
 commands,out=drive_simulate(tmp_path,monkeypatch,render_fps=None)
 assert not [c for c in commands if c[0].endswith('ffmpeg')]
 assert 'not encoded' in json.loads((out/'receipt.json').read_text())['three_views_video']
