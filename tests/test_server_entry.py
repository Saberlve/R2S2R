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
