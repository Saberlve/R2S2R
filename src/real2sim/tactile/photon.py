"""Photon (Xense G1-WS) tactile sensor integration: runtime checks and render config validation.

tacsim (the Data-TacSim library) is consumed from whatever interpreter is passed to --python;
this repository does not carry a copy and does not inject one into sys.path. The vendor
runtime package xense-sim4.5 is committed upstream in that checkout's third_party/xense_photon/.
Everything this module reports is therefore read *from the target interpreter*, never from a
path relative to this file. The module itself is pure logic: it does not import tacsim. The
heavy work lives in photon_worker.py, a separate process that needs CUDA and an OpenGL context.
"""
from __future__ import annotations

import json
import math
import pathlib
import subprocess

BUNDLE_GLOB = "third_party/*/**/pip_prebundle/xensim"
VALID_OUTPUTS = ("depth", "rgb", "marker_flow")


def find_bundle(tacsim_root=None):
    """Locate pip_prebundle/xensim using tacsim's vendor bundle convention; returns None if the root is unknown or not deployed."""
    if tacsim_root is None:
        return None
    root = pathlib.Path(tacsim_root)
    matches = sorted(p for p in root.glob(BUNDLE_GLOB) if p.is_dir())
    return matches[0] if matches else None


def tacsim_root_from_module(module_file):
    """Derive the tacsim checkout root from an imported tacsim.__init__ path; None if unusable.

    tacsim's own bundle discovery (tacsim/backends/photon/_bootstrap.py) resolves relative to
    the imported module, so deriving the root the same way guarantees we report the bundle that
    the worker will actually load rather than one next to this file.
    """
    if not isinstance(module_file, str):
        return None
    path = pathlib.Path(module_file)
    if path.name != "__init__.py" or path.parent.name != "tacsim":
        return None
    return path.parent.parent


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
        # A real import, not find_spec: tacsim/__init__.py is lazy (it registers names in a
        # _LAZY dict), so this stays cheap while still catching a checkout that resolves but
        # is broken. That distinction is exactly what the worker needs to know.
        "try:\n"
        " import tacsim;r['tacsim']=tacsim.__file__\n"
        "except Exception as e:r['tacsim']='missing: '+type(e).__name__\n"
        "print(json.dumps(r))\n"
    )
    try:
        result = subprocess.run([python, "-c", code], capture_output=True, text=True, timeout=120)
        if result.returncode != 0:
            return {"error": (result.stderr or result.stdout).strip()[-400:]}
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def check_photon_runtime(python: str = "python3", tacsim_root=None) -> dict:
    """Item-by-item Photon runtime check report; doctor exits with code 2 if any critical check is False.

    Every check describes the interpreter named by `python`. Pass tacsim_root only to inspect a
    specific checkout; by default the root is derived from what that interpreter actually imports.
    """
    probe = _probe_python(python)
    imported = probe.get("tacsim")
    tacsim_importable = isinstance(imported, str) and not imported.startswith("missing")
    root = pathlib.Path(tacsim_root) if tacsim_root is not None else tacsim_root_from_module(imported)
    bundle = find_bundle(root)
    checks = {
        "tacsim_importable": tacsim_importable,
        "tacsim_repo_root_found": root is not None and (root / "tacsim" / "__init__.py").is_file(),
        "xense_bundle_deployed": bundle is not None,
        "python_is_3.10": probe.get("py310", False),
        "cffi_importable": isinstance(probe.get("cffi"), str) and not probe.get("cffi", "").startswith("missing"),
        "pyudev_importable": isinstance(probe.get("pyudev"), str) and not probe.get("pyudev", "").startswith("missing"),
        "cuda_available": bool(probe.get("cuda", False)),
    }
    critical = ["tacsim_importable", "tacsim_repo_root_found", "xense_bundle_deployed", "python_is_3.10",
                "cffi_importable", "pyudev_importable"]
    return {
        "python": python,
        # None means this interpreter has no importable tacsim at all -- which is the correct
        # encoding of "this repository does not carry one", not an error.
        "tacsim_root": str(root) if root is not None else None,
        # Reported outside `checks` so that every value in `checks` stays a bool: `ready` is
        # all(checks[k] for k in critical), and a non-empty string would truthy-pass.
        "tacsim_root_kind": ("git_checkout" if (root / ".git").exists() else "directory") if root is not None else None,
        "xense_bundle": str(bundle) if bundle else None,
        "checks": checks,
        "probe": probe,
        "ready": all(checks[k] for k in critical),
        "notes": [
            "cuda_available is only required for the offline render_tensor path; in-scene integration is not implemented yet.",
            "Running the Photon backend on a headless host needs xvfb-run -a for the vendor OpenGL context.",
            "Simulated tactile output is not real-contact validation.",
            "tacsim must be importable by --python itself; this repository neither carries a copy nor injects one into sys.path.",
        ],
    }
