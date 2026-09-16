"""Photon (Xense G1-WS) tactile sensor integration: runtime checks and render config validation.

The backend comes from the git submodule external/Data-TacSim (the tacsim library) plus the
vendor-proprietary simulation package xense-sim4.5, deployed under that submodule's
third_party/ and not tracked by git. This module is pure logic: it does not import tacsim.
The heavy work lives in photon_worker.py, a separate process that needs CUDA and an OpenGL
context.
"""
from __future__ import annotations

import json
import math
import pathlib
import subprocess

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
TACSIM_ROOT = REPO_ROOT / "external" / "Data-TacSim"
BUNDLE_GLOB = "third_party/*/**/pip_prebundle/xensim"
VALID_OUTPUTS = ("depth", "rgb", "marker_flow")


def find_bundle(tacsim_root=TACSIM_ROOT):
    """Locate pip_prebundle/xensim using tacsim's vendor bundle convention; returns None if not deployed."""
    root = pathlib.Path(tacsim_root)
    matches = sorted(p for p in root.glob(BUNDLE_GLOB) if p.is_dir())
    return matches[0] if matches else None


def validate_render_config(cfg: dict) -> dict:
    """Validate a photon-render config; returns a normalized copy when it passes. The synthetic stimulus must be declared explicitly."""
    if cfg.get("schema_version") != "1.0":
        raise ValueError("schema_version must be '1.0'")
    if cfg.get("sensor") != "photon":
        raise ValueError("sensor must be 'photon' (the only integrated tactile sensor)")
    outputs = cfg.get("outputs", ["depth", "rgb"])
    if not isinstance(outputs, list) or not outputs or any(o not in VALID_OUTPUTS for o in outputs):
        raise ValueError("outputs must be a non-empty subset of " + ", ".join(VALID_OUTPUTS))
    if cfg.get("declared_synthetic") is not True:
        raise ValueError("declared_synthetic: true is required; simulated tactile output is not real-contact validation")
    stimulus = cfg.get("stimulus", {})
    if stimulus.get("type") != "gaussian":
        raise ValueError("stimulus.type must be 'gaussian' (the only implemented synthetic stimulus)")
    for key in ("amplitude_m", "sigma_m"):
        value = stimulus.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("stimulus." + key + ": expected a positive finite value in metres")
    return {
        "schema_version": "1.0",
        "sensor": "photon",
        "outputs": list(outputs),
        "declared_synthetic": True,
        "stimulus": {
            "type": "gaussian",
            "amplitude_m": float(stimulus["amplitude_m"]),
            "sigma_m": float(stimulus["sigma_m"]),
        },
        "device": cfg.get("device", "cuda"),
    }


def _probe_python(python: str) -> dict:
    """Probe the target interpreter for the conditions tactile needs; everything is reported and nothing fails hard."""
    code = (
        "import json,sys\n"
        "r={'version':sys.version.split()[0],'py310':sys.version_info[:2]==(3,10)}\n"
        "for m in ['cffi','pyudev','torch']:\n"
        " try:\n"
        "  mod=__import__(m);r[m]=getattr(mod,'__version__','present')\n"
        " except Exception as e:r[m]='missing: '+type(e).__name__\n"
        "try:\n"
        " import torch;r['cuda']=bool(torch.cuda.is_available())\n"
        "except Exception:r['cuda']=False\n"
        "print(json.dumps(r))\n"
    )
    try:
        result = subprocess.run([python, "-c", code], capture_output=True, text=True, timeout=120)
        if result.returncode != 0:
            return {"error": (result.stderr or result.stdout).strip()[-400:]}
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def check_photon_runtime(python: str = "python3", tacsim_root=TACSIM_ROOT) -> dict:
    """Item-by-item Photon runtime check report; doctor exits with code 2 if any critical check is False."""
    root = pathlib.Path(tacsim_root)
    bundle = find_bundle(root)
    probe = _probe_python(python)
    checks = {
        "tacsim_submodule_present": (root / "tacsim" / "__init__.py").is_file(),
        "xense_bundle_deployed": bundle is not None,
        "python_is_3.10": probe.get("py310", False),
        "cffi_importable": isinstance(probe.get("cffi"), str) and not probe.get("cffi", "").startswith("missing"),
        "pyudev_importable": isinstance(probe.get("pyudev"), str) and not probe.get("pyudev", "").startswith("missing"),
        "cuda_available": bool(probe.get("cuda", False)),
    }
    critical = ["tacsim_submodule_present", "xense_bundle_deployed", "python_is_3.10",
                "cffi_importable", "pyudev_importable"]
    return {
        "python": python,
        "tacsim_root": str(root),
        "xense_bundle": str(bundle) if bundle else None,
        "checks": checks,
        "probe": probe,
        "ready": all(checks[k] for k in critical),
        "notes": [
            "cuda_available is only required for the offline render_tensor path; in-scene integration is not implemented yet.",
            "Running the Photon backend on a headless host needs xvfb-run -a for the vendor OpenGL context.",
            "Simulated tactile output is not real-contact validation.",
        ],
    }
