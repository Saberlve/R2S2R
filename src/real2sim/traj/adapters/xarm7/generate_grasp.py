"""Generate a sensor-centered approach-close-lift task through the shared planner."""

import argparse
import json
import numpy as np
from scipy.spatial.transform import Rotation
from real2sim.traj.grasp_geometry import safe_side_grasp
from physics_model import ROOT
from plan_trajectory import solve

p = argparse.ArgumentParser()
p.add_argument("--offset", type=float, nargs=3, default=[0, 0, 0])
p.add_argument("--output", default="generated_grasp.npz")
p.add_argument("--sensor-bottom-offset", type=float, default=0.01752)
p.add_argument("--table-clearance", type=float, default=0.002)
p.add_argument("--pad-down", type=float, default=0.01417)
p.add_argument("--pad-up", type=float, default=0.01417)
p.add_argument("--minimum-contact-overlap", type=float, default=0.006)
a = p.parse_args()
out = ROOT / "results" / a.output
if out.exists():
    raise FileExistsError(out)
s = json.loads((ROOT / "scene_config.json").read_text())
B = np.array(s["T_sim_base"])
normal = np.array(s["table_matrix"])[:3, 2]
obj = np.array(s["bar"]["position_m"]) + np.array(a.offset)
obj_rotation = Rotation.from_quat(s["bar"]["quaternion_xyzw"])
grasp = safe_side_grasp(
    object_center_m=obj,
    object_size_m=s["bar"]["size_m"],
    object_quaternion_xyzw=s["bar"]["quaternion_xyzw"],
    table_matrix=s["table_matrix"],
    table_size_m=s["table_size_m"],
    sensor_bottom_offset_m=a.sensor_bottom_offset,
    clearance_m=a.table_clearance,
    pad_down_m=a.pad_down,
    pad_up_m=a.pad_up,
    minimum_overlap_m=a.minimum_contact_overlap,
)
x = obj_rotation.as_matrix()[:, 0]
z = -normal
x = x - z * np.dot(x, z)
x /= np.linalg.norm(x)
y = np.cross(z, x)
R = np.column_stack([x, y, z])
obj = grasp.position_m
waypoints = []
for position, g, duration in [
    (obj + normal * 0.12, 0, 3),
    (obj, 0, 2),
    (obj, 1, 1.5),
    (obj + normal * 0.12, 1, 2.5),
    (obj + normal * 0.12, 1, 2),
]:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position
    T = np.linalg.solve(B, T)
    waypoints.append(
        dict(
            position_m=T[:3, 3].tolist(),
            quaternion_xyzw=Rotation.from_matrix(T[:3, :3]).as_quat().tolist(),
            gripper=g,
            duration_s=duration,
        )
    )
task = out.with_suffix(".task.json")
if task.exists():
    raise FileExistsError(task)
task.write_text(
    json.dumps(
        dict(
            schema_version="1.0",
            fps=30,
            planning_frame={"kind": "sensor_center"},
            waypoints=waypoints,
        ),
        indent=2,
    )
)
episode, report = solve(task, out.parent)
np.savez_compressed(out, **episode)
report["grasp_geometry"] = {
    "policy": "center grasp raised along table normal for sensor clearance",
    "sensor_bottom_offset_m": a.sensor_bottom_offset,
    "table_clearance_m": a.table_clearance,
    "pad_down_m": a.pad_down,
    "pad_up_m": a.pad_up,
    "minimum_contact_overlap_m": a.minimum_contact_overlap,
    "object_min_height_m": grasp.object_min_height_m,
    "object_max_height_m": grasp.object_max_height_m,
    "center_height_m": grasp.center_height_m,
    "grasp_height_m": grasp.grasp_height_m,
    "upward_shift_m": grasp.upward_shift_m,
    "contact_overlap_m": grasp.contact_overlap_m,
}
out.with_suffix(".planning.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report), flush=True)
