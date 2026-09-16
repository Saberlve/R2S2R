"""Planning-only tool frame. Recorded EEF always keeps the controller TCP definition.

T_tcp_sensor maps sensor-center coordinates into the unchanged controller TCP.
All distances are meters; gripper is the existing 0=open, 1=closed command.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from real2sim.contracts import rigid, sha


class PlanningFrame:
    def __init__(self, config=None, task_dir=None):
        self.grid = None
        self.metadata = {"waypoint_frame": "controller_tcp", "recorded_eef": "controller_tcp",
                         "controller_tcp_changed": False}
        if config is None:
            return
        path = Path(config["transform_file"])
        if not path.is_absolute():
            path = Path(task_dir) / path
        path = path.resolve()
        data = json.loads(path.read_text(encoding="utf-8"))
        if (data.get("schema_version") != "1.0" or data.get("length_unit") != "m"
                or data.get("gripper_convention") != "0=open,1=closed"
                or data.get("transform_convention") != "T_tcp_sensor"):
            raise ValueError("Planning frame requires meters, T_tcp_sensor and normalized gripper")
        self.controller = rigid(np.array(data["T_flange_tcp"], float), "T_flange_tcp")
        expected = np.eye(4); expected[2, 3] = .172
        if not np.allclose(self.controller, expected, atol=1e-9, rtol=0):
            raise ValueError("xArm7 planner requires the unchanged TCP172 controller reference")
        self.grid = np.array(data["gripper_closed_fraction"], float)
        if (self.grid.ndim != 1 or len(self.grid) < 2 or not np.isfinite(self.grid).all()
                or not np.all(np.diff(self.grid) > 0) or self.grid[0] != 0 or self.grid[-1] != 1):
            raise ValueError("Planning frame gripper grid must strictly increase from 0 to 1")
        self.transforms = np.array([rigid(np.array(t, float), "T_tcp_sensor")
                                    for t in data["T_tcp_sensor"]])
        if len(self.transforms) != len(self.grid):
            raise ValueError("Planning frame sample counts differ")
        self.rotations = Slerp(self.grid, Rotation.from_matrix(self.transforms[:, :3, :3]))
        self.metadata = {**self.metadata, "waypoint_frame": "sensor_center",
                         "transform_file": str(path), "transform_sha256": sha(path),
                         "quality": data.get("quality", "unspecified"),
                         "gripper_source": "interpolated planned closed fraction; not measured feedback"}

    def tcp_to_sensor(self, gripper):
        g = float(gripper)
        if not np.isfinite(g) or not 0 <= g <= 1:
            raise ValueError("Gripper must be in [0,1]")
        T = np.eye(4)
        if self.grid is not None:
            T[:3, 3] = [np.interp(g, self.grid, self.transforms[:, i, 3]) for i in range(3)]
            T[:3, :3] = self.rotations(g).as_matrix()
        return T

    def planning_pose(self, T_world_tcp, gripper):
        return rigid(T_world_tcp, "TCP pose") @ self.tcp_to_sensor(gripper)

    def tcp_target(self, T_world_planning, gripper):
        return rigid(T_world_planning, "planning pose") @ np.linalg.inv(self.tcp_to_sensor(gripper))


def recorded_tcp_poses(T_sim_base, actual_controller_tcp):
    """Serialize original EEF FK only. No planning-frame transform is accepted here."""
    base = rigid(T_sim_base, "T_sim_base")
    poses = np.array([np.linalg.solve(base, rigid(t, "actual controller TCP"))
                      for t in actual_controller_tcp])
    return {"tcp_m": poses[:, :3, 3],
            "tcp_quat_xyzw": Rotation.from_matrix(poses[:, :3, :3]).as_quat()}
