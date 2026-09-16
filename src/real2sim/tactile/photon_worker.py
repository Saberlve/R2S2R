"""Photon offline render worker: runs under a Python that has the tacsim dependencies (3.10 + CUDA + OpenGL).

Usage: python photon_worker.py <config.json> <out_dir>
Environment: R2S_TACSIM_ROOT points at the Data-TacSim submodule root (defaults to this repo's
external/Data-TacSim). Headless hosts need xvfb-run -a. The output is a simulated observation
and must not be read as real-contact validation.
"""
import os
import pathlib
import sys

_ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT / "src"))
sys.path.insert(0, os.environ.get("R2S_TACSIM_ROOT", str(_ROOT / "external" / "Data-TacSim")))

import numpy as np  # noqa: E402

from real2sim.contracts import save, sha  # noqa: E402
from real2sim.tactile.photon import validate_render_config  # noqa: E402


def save_requested_outputs(frame, outputs, out):
    """Save the requested channels; raises RuntimeError when the backend did not produce one that was asked for. Returns the file list."""
    files = {}
    if "depth" in outputs:
        if getattr(frame, "depth_m", None) is None:
            raise RuntimeError("backend did not produce requested output: depth")
        depth = frame.depth_m.detach().cpu().numpy().astype(np.float32)
        np.save(out / "depth_m.npy", depth)
        files["depth_m.npy"] = {"shape": list(depth.shape), "unit": "m"}
    if "rgb" in outputs:
        if getattr(frame, "rgb", None) is None:
            raise RuntimeError("backend did not produce requested output: rgb")
        from PIL import Image
        rgb = frame.rgb.detach().cpu().numpy()
        Image.fromarray(rgb).save(out / "rgb.png")
        files["rgb.png"] = {"shape": list(rgb.shape)}
    if "marker_flow" in outputs:
        if getattr(frame, "marker_flow", None) is None:
            raise RuntimeError("backend did not produce requested output: marker_flow")
        flow = frame.marker_flow.detach().cpu().numpy().astype(np.float32)
        np.save(out / "marker_flow.npy", flow)
        files["marker_flow.npy"] = {"shape": list(flow.shape), "unit": "px"}
    return files


def main():
    config_path, out = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2]).resolve()
    if out.exists():
        raise FileExistsError("A run directory must be new: " + str(out))
    cfg = validate_render_config(__import__("json").loads(config_path.read_text(encoding="utf-8")))

    import torch
    from tacsim.runtime import NewtonTactileSensor, TactileLatentTensor

    tacsim_outputs = tuple({"marker_flow": "marker"}.get(o, o) for o in cfg["outputs"])
    runtime = NewtonTactileSensor("photon", outputs=tacsim_outputs, device=cfg["device"])
    runtime.prepare_renderers()
    try:
        rest = torch.as_tensor(runtime.gel.contact_rest_local, device=cfg["device"], dtype=torch.float32)
        radius2 = rest[:, :2].square().sum(dim=1)
        indentation = cfg["stimulus"]["amplitude_m"] * torch.exp(
            -radius2 / (2.0 * cfg["stimulus"]["sigma_m"] ** 2))
        latent = TactileLatentTensor(
            surface_local=rest,
            indentation_m=indentation,
            force_local=torch.zeros_like(rest),
            total_force_local=torch.zeros(3, device=cfg["device"]),
            max_indent_m=indentation.max(),
            in_contact=torch.tensor(True, device=cfg["device"]),
        )
        frame = runtime.render_tensor(latent)
    finally:
        if runtime.renderer is not None and hasattr(runtime.renderer, "close"):
            runtime.renderer.close()

    out.mkdir(parents=True)
    files = save_requested_outputs(frame, cfg["outputs"], out)
    manifest = {
        "schema_version": "1.0",
        "sensor": "photon",
        "backend": "Data-TacSim tacsim + xense-sim4.5 vendor bundle",
        "declared_synthetic": True,
        "stimulus": cfg["stimulus"],
        "device": cfg["device"],
        "outputs": {name: {**meta, "sha256": sha(out / name)} for name, meta in files.items()},
        "config_sha256": sha(config_path),
        "notes": [
            "Synthetic Gaussian indentation stimulus; simulated tactile output is not real-contact validation.",
            "depth is in metres; marker_flow is in pixels; rgb is the vendor's optical simulation image.",
        ],
    }
    save(out / "tactile_manifest.json", manifest)
    print("PHOTON_RENDER_COMPLETE", out)


if __name__ == "__main__":
    main()
