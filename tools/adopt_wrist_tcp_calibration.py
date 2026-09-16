#!/usr/bin/env python3
"""Create a fresh wrist-camera runtime from a measured TCP-to-camera transform.

Run this with the configured Blender/Cycles Python because the copied .blend
must be updated together with its JSON bindings.

This tool writes only inside --output-case. Switching the pipeline to the new
runtime is a manual operator step: set R2S_REFERENCE_TEMPLATE (and the scene
paths) yourself. The values to use are printed at the end.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np

from real2sim.align.wrist import (
    CV_TO_BLENDER,
    build_record,
    controller_flange_tcp,
    rigid,
    rotation_distance_deg,
)

__all__ = [
    "CV_TO_BLENDER",
    "build_record",
    "controller_flange_tcp",
    "rigid",
    "rotation_distance_deg",
    "load",
    "save",
    "update_blend",
    "main",
]


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def save(path: Path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def update_blend(runtime: Path, record: dict) -> tuple[list, list]:
    try:
        import bpy
        from mathutils import Matrix
    except ImportError as exc:
        raise RuntimeError("Run with the configured Blender/Cycles Python") from exc

    blend = runtime / "Baseline.blend"
    bpy.ops.wm.open_mainfile(filepath=str(blend))
    scene = bpy.context.scene
    camera = scene.objects["Camera_RealSense_A_Wrist.001"]
    flange = scene.objects["RobotArm_Joint_07.001"]
    local = Matrix(record["T_flange_camera_blender"])
    camera.matrix_world = flange.matrix_world @ local
    bpy.context.view_layer.update()
    world = np.asarray(camera.matrix_world, dtype=float).tolist()
    basis = np.asarray(camera.matrix_basis, dtype=float).tolist()
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    return world, basis


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-case", type=Path, required=True)
    parser.add_argument("--output-case", type=Path, required=True)
    parser.add_argument("--measurement", type=Path, required=True)
    parser.add_argument("--controller-calibration", type=Path, required=True)
    args = parser.parse_args()

    source = args.source_case.resolve()
    output = args.output_case.resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite output case: {output}")
    previous = load(source / "calibration.json")
    measurement = load(args.measurement.resolve())
    controller = load(args.controller_calibration.resolve())
    record = build_record(measurement, controller, previous)

    shutil.copytree(source / "runtime", output / "runtime")
    shutil.copy2(args.measurement.resolve(), output / "source_measurement.json")
    save(output / "calibration.json", record)

    runtime = output / "runtime"
    world, basis = update_blend(runtime, record)

    camera_config_path = runtime / "camera_config.json"
    camera_config = load(camera_config_path)
    wrist = camera_config["realsense_A_wrist"]
    wrist["T_world_camera_blender"] = world
    wrist["T_flange_camera_blender"] = record["T_flange_camera_blender"]
    wrist["post_rotation_degrees"] = record["post_rotation_degrees"]
    wrist["extrinsics_status"] = "Independent TCP hand-eye calibration candidate; see calibration.json"
    save(camera_config_path, camera_config)

    manifest_path = runtime / "runtime_manifest.json"
    manifest = load(manifest_path)
    attachment = manifest["attachments"]["Camera_RealSense_A_Wrist.001"]
    attachment["T_body_object"] = record["T_flange_camera_blender"]
    attachment["matrix_basis"] = basis
    manifest["camera_optics"] = (
        "Independent TCP hand-eye calibration candidate; explicit TCP-to-flange composition; "
        f"wrist image post-rotation {record['post_rotation_degrees']} degrees"
    )
    manifest["wrist_calibration"] = "../calibration.json"
    save(manifest_path, manifest)

    report = record["comparison_to_previous_video_fit"]
    (output / "REPORT.md").write_text(
        "# Wrist Camera TCP Calibration Candidate\n\n"
        f"- Against the current video-fit extrinsics: optical centre shift "
        f"{report['optical_center_distance_mm']:.3f} mm, rotation {report['rotation_distance_deg']:.3f} deg.\n"
        f"- Flange-frame translation difference XYZ: {[round(x, 3) for x in report['optical_center_delta_flange_xyz_mm']]} mm.\n"
        f"- Leave-out max error reported by the calibration tool: {record['reported_leaveout_max_error']}.\n"
        "- Status: written as an independent candidate template; it still needs a reprojection "
        "evaluation against synchronised real images and robot states that were not part of the calibration.\n",
        encoding="utf-8",
    )

    print(
        "Manual activation required; this tool does not modify config files.\n"
        "Update these paths yourself, then re-run the server entry points:\n"
        f"  export R2S_REFERENCE_TEMPLATE={runtime}\n"
        f"  scene scene              = {runtime / 'Baseline.blend'}\n"
        f"  scene runtime_manifest   = {runtime / 'runtime_manifest.json'}\n"
        f"  scene camera_config      = {runtime / 'camera_config.json'}\n"
        f"  scene render_script      = {runtime / 'render.py'}\n"
    )
    print(json.dumps(record, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
