"""Render Newton states through the frozen Cycles scene; cameras/background stay fixed."""

import argparse, pathlib, json, hashlib, os
import numpy as np, bpy
from mathutils import Matrix
from PIL import Image
from real2sim.traj.replay_metrics import playback_fps

ROOT = pathlib.Path(os.environ["R2S_CASE_ROOT"]).resolve()
p = argparse.ArgumentParser()
p.add_argument("--run", required=True)
p.add_argument("--out")
p.add_argument("--stride", type=int, default=10)
p.add_argument("--samples", type=int, default=16)
p.add_argument("--limit", type=int, default=0)
a = p.parse_args()
run = pathlib.Path(a.run)
run = run if run.is_absolute() else ROOT / "results" / run
data = np.load(run / "states.npz")
# The replay's own clock, never a constant: a 60 Hz control run played at a fixed 30 Hz would
# run at half speed and drift out of step with the tactile videos.  Read before rendering so a
# non-uniform timeline is refused before any GPU work.
source_fps = playback_fps(data["time"])
render_fps = playback_fps(data["time"], a.stride)
geo = json.loads((ROOT / "inputs/baseline_geometry.json").read_text())
settings = json.loads((run / "scene_config.json").read_text())
baseline = ROOT / "inputs/render_baseline"
cfg = json.loads((baseline / "camera_config.json").read_text())
manifest = json.loads((baseline / "runtime_manifest.json").read_text())
bpy.ops.wm.open_mainfile(filepath=str(baseline / "Baseline.blend"))
scene = bpy.data.scenes["RobotSyncV1"]
if bpy.context.window:
    bpy.context.window.scene = scene
fixed = {
    v["object"]: scene.objects[v["object"]].matrix_world.copy()
    for k, v in cfg.items()
    if k != "realsense_A_wrist"
}
prefs = bpy.context.preferences.addons["cycles"].preferences
prefs.compute_device_type = "CUDA"
prefs.refresh_devices()
for d in prefs.devices:
    d.use = d.type == "CUDA"
# Without this Cycles destroys its CUDA context after every render() call, so a run of this
# scene reloads the kernels once per camera per frame and every one of those allocations can
# fail against whatever else holds the card.  Keeping the device alive renders this run about
# 1.6x faster (median 2.67 s -> 1.59 s per camera-frame, measured on the bar-grasp case) and
# leaves far fewer chances of losing it; the image is unchanged, since persistent and one-shot
# differ by at most 1/255, as much as two renders within either mode.
scene.render.use_persistent_data = True
scene.render.engine = "CYCLES"
scene.cycles.device = "GPU"
scene.cycles.samples = a.samples
scene.cycles.use_denoising = True
scene.cycles.seed = 42
scene.render.resolution_percentage = 100
scene.render.use_border = False
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGB"
labels = data["body_labels"].tolist()
indices = {
    name: labels.index("UF_ROBOT/" + name)
    for name in ["link" + str(i) for i in range(1, 8)]
}


def matrix(v):
    q = __import__("mathutils").Quaternion(
        (float(v[6]), float(v[3]), float(v[4]), float(v[5]))
    )
    T = q.to_matrix().to_4x4()
    T.translation = v[:3]
    return T


# Use the accepted scene geometry and bind every moving part to saved bodies.
# No frozen case renderer or reconstructed stock gripper can override these parts.
bindings = manifest["custom_g2"]["bindings"]
custom = [name for name in bindings if name.startswith("Custom_")]
if len(custom) != 6:
    raise ValueError("Expected six custom gripper bindings")
resolved = {}
for name, binding in bindings.items():
    if name not in scene.objects:
        raise ValueError("Missing bound object: " + name)
    label = binding["body_label"]
    if labels.count(label) != 1:
        raise ValueError("Missing or ambiguous body: " + label)
    obj = scene.objects[name]
    obj.parent = None
    obj.constraints.clear()
    resolved[name] = (labels.index(label), Matrix(binding["T_body_object"]))
# Wrist camera optics use the accepted calibration; moving the arm must not
# silently replace it with the scene's earlier parent transform.
cal = json.loads((baseline / manifest["wrist_calibration"]).read_text())
bar = None
if data["bar"].size:
    bpy.ops.mesh.primitive_cube_add()
    bar = bpy.context.object
    bar.name = "WhitePlasticBar"
    bar.dimensions = settings["bar"]["size_m"]
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    mat = bpy.data.materials.new("WhitePlastic")
    mat.diffuse_color = (0.8, 0.8, 0.78, 1)
    mat.use_nodes = True
    mat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (
        0.8,
        0.8,
        0.78,
        1,
    )
    bar.data.materials.append(mat)
out = pathlib.Path(a.out) if a.out else run / "render"
out.mkdir(exist_ok=False)
checks = []
frameids = list(range(0, len(data["time"]), a.stride))
frameids = frameids[: a.limit] if a.limit else frameids
for k, i in enumerate(frameids):
    body = data["body_q"][i]
    for j in range(1, 8):
        scene.objects[f"RobotArm_Joint_{j:02d}.001"].matrix_world = matrix(
            body[indices["link" + str(j)]]
        )
        scene.view_layers[0].update()
    for name, (index, local) in resolved.items():
        scene.objects[name].matrix_world = matrix(body[index]) @ local
    scene.view_layers[0].update()
    wrist = scene.objects[cfg["realsense_A_wrist"]["object"]]
    wrist.matrix_world = (
        matrix(body[indices["link7"]])
        @ Matrix(cal["T_flange_camera_cv"])
        @ Matrix.Diagonal((1, -1, -1, 1))
    )
    scene.view_layers[0].update()
    if bar:
        bar.matrix_world = matrix(data["bar"][i])
    for name, T in fixed.items():
        assert (
            np.max(np.abs(np.array(scene.objects[name].matrix_world) - np.array(T)))
            < 1e-6
        )
    tiles = []
    for cid, c in cfg.items():
        cam = scene.objects[c["object"]]
        scene.camera = cam
        w, h = c["width"], c["height"]
        K = np.array(c["K"])
        fx, fy = K[0][0], K[1][1]
        cx, cy = K[0][2], K[1][2]
        scene.render.resolution_x = w
        scene.render.resolution_y = h
        scene.render.pixel_aspect_x = 1
        scene.render.pixel_aspect_y = fx / fy
        cam.data.sensor_fit = "HORIZONTAL"
        cam.data.sensor_width = 36
        cam.data.lens = 36 * fx / w
        cam.data.shift_x = (w / 2 - cx) / w
        cam.data.shift_y = (cy - h / 2) * (fx / fy) / w
        folder = out / cid
        folder.mkdir(exist_ok=True)
        file = folder / f"{k:06d}.png"
        scene.render.filepath = str(file)
        bpy.ops.render.render(write_still=True)
        im = Image.open(file).convert("RGB")
        if c.get("post_rotation_degrees") == 180:
            im = im.transpose(Image.Transpose.ROTATE_180)
            im.save(file)
        tiles.append(im)
    canvas = Image.new(
        "RGB", (sum(im.width for im in tiles), max(im.height for im in tiles))
    )
    x = 0
    for im in tiles:
        canvas.paste(im, (x, 0))
        x += im.width
    canvas.save(out / f"three_views_{k:06d}.png")
    checks.append(
        {
            "render_frame": k,
            "simulation_frame": i,
            "time_s": float(data["time"][i]),
            "custom_binding_max_error": max(
                float(
                    np.max(
                        np.abs(
                            np.array(scene.objects[name].matrix_world)
                            - np.array(matrix(body[index]) @ local)
                        )
                    )
                )
                for name, (index, local) in resolved.items()
            ),
        }
    )
    print("FRAME_DONE", k, flush=True)
    if k == 0:
        bpy.ops.wm.save_as_mainfile(filepath=str(out / "preview.blend"))
(out / "manifest.json").write_text(
    json.dumps(
        {
            "source_states_sha256": hashlib.sha256(
                (run / "states.npz").read_bytes()
            ).hexdigest(),
            "custom_bindings": bindings,
            "composite_policy": "native camera resolution; top aligned, black padding; no resize",
            "camera_sizes": {cid: [c["width"], c["height"]] for cid, c in cfg.items()},
            "physics_geometry_changed": False,
            "frames": checks,
            "source_fps": source_fps,
            "render_fps": render_fps,
            "fps_source": "states.npz time column, divided by --stride",
            "observer_cameras_fixed": True,
            "image_alignment_deferred": True,
        },
        indent=2,
    )
)
