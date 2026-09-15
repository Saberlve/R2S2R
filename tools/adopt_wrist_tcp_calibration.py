#!/usr/bin/env python3
"""Create a fresh wrist-camera runtime from a measured TCP-to-camera transform.

Run this with the configured Blender/Cycles Python because the copied .blend
must be updated together with its JSON bindings.
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
    parser.add_argument("--activate-scene", type=Path)
    parser.add_argument("--activate-site", type=Path)
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
        "# 腕部相机 TCP 标定候选\n\n"
        f"- 相对当前录像拟合外参：光心平移 {report['optical_center_distance_mm']:.3f} mm，"
        f"旋转 {report['rotation_distance_deg']:.3f}°。\n"
        f"- 法兰坐标平移差 XYZ：{[round(x, 3) for x in report['optical_center_delta_flange_xyz_mm']]} mm。\n"
        f"- 标定工具报告留出最大误差：{record['reported_leaveout_max_error']}。\n"
        "- 当前状态：已写入独立候选模板；仍需用未参与标定的同步实拍和机器人状态做重投影评估。\n",
        encoding="utf-8",
    )

    if args.activate_scene:
        scene_path = args.activate_scene.resolve()
        scene = load(scene_path)
        rel = output.relative_to(scene_path.parent)
        scene.update({
            "version": "ServerBaseline_WristTcpCalibrationV1_20260914",
            "scene": str(rel / "runtime" / "Baseline.blend"),
            "runtime_manifest": str(rel / "runtime" / "runtime_manifest.json"),
            "camera_config": str(rel / "runtime" / "camera_config.json"),
            "render_script": str(rel / "runtime" / "render.py"),
            "scope": "Server-authoritative scene; wrist optical pose uses independent TCP hand-eye calibration; pending independent scene reprojection validation.",
            "previous_visual_baseline": "WristCameraAlignmentV1/runtime/Baseline.blend",
        })
        save(scene_path, scene)
    if args.activate_site:
        site_path = args.activate_site.resolve()
        site = load(site_path)
        site["reference_template"] = str(runtime)
        save(site_path, site)
    print(json.dumps(record, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
