"""Normalize an existing read-only xArm controller snapshot; no SDK connection."""
import argparse,pathlib,json,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'))
from real2sim.align.base import snapshot_to_calibration
p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--out',required=True);a=p.parse_args();raw=json.loads(pathlib.Path(a.input).read_text(encoding='utf-8-sig'))
out=pathlib.Path(a.out)
if out.exists():raise FileExistsError(out)
out.write_text(json.dumps(snapshot_to_calibration(raw),indent=2))
