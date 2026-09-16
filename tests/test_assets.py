import importlib.util,pathlib,shutil,subprocess
import pytest

from real2sim import assets
from real2sim.contracts import save

spec=importlib.util.spec_from_file_location('git_payload',pathlib.Path(__file__).resolve().parents[1]/'tools/check_git_payload.py')
payload=importlib.util.module_from_spec(spec);spec.loader.exec_module(payload)
ROOT=pathlib.Path(__file__).resolve().parents[1]
LIBRARY=ROOT/'assets'

def spec_document(**overrides):
 doc={'schema_version':'1.0','id':'Widget','role':'task','geometry':{'kind':'box','size_m':[0.1,0.05,0.02]},
      'provenance':{'confidence':'model','source':'unit test'}}
 doc.update(overrides);return doc

def object_dir(tmp_path,doc=None):
 directory=tmp_path/(doc or spec_document())['id'];directory.mkdir(parents=True)
 save(directory/'asset.json',doc or spec_document());return directory

def test_the_committed_library_validates():
 report=assets.validate_library(LIBRARY)
 assert report['passed'] is True and report['errors']==[]
 assert 'TestBox' in {row['id'] for row in report['objects']}

def test_unknown_fields_are_rejected(tmp_path):
 directory=object_dir(tmp_path)
 for mutated in [spec_document(extra=1),spec_document(geometry={'kind':'box','size_m':[1,1,1],'extra':1})]:
  save(directory/'asset.json',mutated)
  with pytest.raises(Exception):assets.load_asset(directory/'asset.json',directory)

def test_id_must_match_its_directory(tmp_path):
 directory=object_dir(tmp_path)
 with pytest.raises(ValueError):assets.validate_asset(spec_document(id='Other'),directory)

def test_box_geometry_needs_a_positive_size():
 for geometry in [{'kind':'box'},{'kind':'box','size_m':[0.1,0,-0.1]}]:
  with pytest.raises(Exception):assets.validate_asset(spec_document(geometry=geometry))

def test_mesh_geometry_must_stay_inside_the_object_directory(tmp_path):
 directory=object_dir(tmp_path,spec_document(geometry={'kind':'mesh','path':'../escape.glb'}))
 with pytest.raises(ValueError):assets.validate_asset(spec_document(geometry={'kind':'mesh','path':'../escape.glb'}),directory)
 # A format the build worker cannot import must not be committed either.
 with pytest.raises(ValueError):assets.validate_asset(spec_document(geometry={'kind':'mesh','path':'body.obj'}),directory)
 with pytest.raises(ValueError):assets.validate_asset(spec_document(geometry={'kind':'mesh','path':'body.glb'}),directory)

def test_mesh_asset_resolves_and_hashes(tmp_path):
 directory=object_dir(tmp_path,spec_document(geometry={'kind':'mesh','path':'body.glb'}))
 (directory/'body.glb').write_bytes(b'glTF fixture')
 doc=assets.load_asset(directory/'asset.json',directory)
 assert assets.mesh_path(doc,directory,must_exist=True).name=='body.glb'
 report=assets.validate_library(tmp_path)
 assert report['passed'] and report['objects'][0]['mesh']=='Widget/body.glb' and len(report['objects'][0]['sha256'])==64

def test_physics_and_visual_ranges():
 with pytest.raises(Exception):assets.validate_asset(spec_document(physics={'mass_kg':0,'friction':0.5}))
 with pytest.raises(Exception):assets.validate_asset(spec_document(physics={'mass_kg':1,'friction':-1}))
 with pytest.raises(Exception):assets.validate_asset(spec_document(physics={'mass_kg':1,'friction':0.5,'inertia_diagonal_kg_m2':[1,0,1]}))
 with pytest.raises(ValueError):assets.validate_asset(spec_document(visual={'color_linear':[1.5,0,0]}))
 with pytest.raises(ValueError):assets.validate_asset(spec_document(visual={'roughness':2}))

def test_a_broken_object_is_reported_not_raised(tmp_path):
 object_dir(tmp_path);broken=tmp_path/'Broken';broken.mkdir();(broken/'asset.json').write_text('{}')
 report=assets.validate_library(tmp_path)
 assert report['passed'] is False and len(report['errors'])==1 and report['errors'][0].startswith('Broken')

def test_a_missing_library_root_is_reported(tmp_path):
 report=assets.validate_library(tmp_path/'absent')
 assert report['passed'] is False and 'missing' in report['errors'][0]

def test_symlinked_object_directories_are_rejected(tmp_path):
 target=object_dir(tmp_path/'target');(tmp_path/'library').mkdir()
 try:(tmp_path/'library'/'Widget').symlink_to(target,target_is_directory=True)
 except OSError:pytest.skip('symlinks unavailable on this filesystem')
 report=assets.validate_library(tmp_path/'library')
 assert report['passed'] is False and 'symlink' in report['errors'][0]

def test_the_payload_gate_scopes_the_asset_exception():
 assert payload.classify('assets/Widget/body.glb',1000) is None
 assert payload.classify('assets/Widget/textures/albedo.png',1000) is None
 assert payload.classify('assets/Widget/body.glb',3_000_000)=='object asset larger than 2MB'
 assert payload.classify('assets/Widget/render.png',1000)=='object asset raster outside a textures/ directory'
 assert payload.classify('assets/Widget/body.blend',1000)=='unsupported object asset format'
 assert payload.classify('assets/Widget/notes.md',1000) is None
 assert payload.classify('renders/body.glb',1000)=='resource binary'
 assert payload.classify('renders/small.txt',1000) is None
 assert payload.classify('examples/lab_reference/scanner/door_erase_mask.png',10) is None

def git_ignores(relative):
 """True when .gitignore would drop this path, so `git add` silently skips it."""
 return subprocess.run(['git','-C',str(ROOT),'check-ignore','-q','--',relative],capture_output=True).returncode==0

@pytest.mark.skipif(shutil.which('git') is None,reason='git is required to test the ignore rules')
def test_gitignore_and_the_payload_gate_agree():
 # The two files must not drift: a path the gate permits has to be committable, and a raster
 # outside textures/ has to stay ignored even though the directory is open for meshes.
 for relative,allowed in [('assets/Widget/body.glb',True),('assets/Widget/model.gltf',True),
                          ('assets/Widget/textures/albedo.png',True),('assets/Widget/textures/albedo.jpg',True),
                          ('assets/Widget/render.png',False),('assets/Widget/body.blend',False)]:
  assert (payload.classify(relative,1000) is None)==allowed,relative
  assert git_ignores(relative) is not allowed,relative

@pytest.mark.skipif(shutil.which('git') is None,reason='git is required to test the ignore rules')
def test_the_committed_library_respects_the_git_budget():
 # Pre-commit feedback: the payload gate scans all reachable history, so a violation found
 # here is cheap and the same violation found after a commit is not.
 for path in sorted(p for p in LIBRARY.rglob('*') if p.is_file()):
  relative=str(path.relative_to(ROOT))
  assert path.stat().st_size<=payload.asset_max_bytes,relative
  assert not git_ignores(relative),relative
  if path.suffix.lower() in payload.asset_mesh_suffixes|payload.asset_raster_suffixes:
   assert payload.classify(relative,path.stat().st_size) is None,relative
