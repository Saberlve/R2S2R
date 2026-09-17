"""Calibrated xArm7 adapter; reuses existing Newton world and contact engines."""

import pathlib, json, sys
import numpy as np
from scipy.spatial.transform import Rotation
import os

ROOT = pathlib.Path(os.environ["R2S_CASE_ROOT"]).resolve()
REPO = pathlib.Path(os.environ["R2S_NEWTON_PROJECT"]).resolve()
sys.path.insert(0, str(REPO))
from newton_gen.robot.xarm7 import XArm7, XArm7Info
from newton_gen.core.config import SimConfig
from newton_gen.scenes.spec import SceneSpec, ObjectSpec, Pose
from newton_gen.sim.world import build_world
import newton, warp as wp


def urdf_signature(file):
    import hashlib, xml.etree.ElementTree as ET

    file = pathlib.Path(file)
    h = hashlib.sha256(file.read_bytes())
    for filename in sorted(
        {m.get("filename") for m in ET.parse(file).findall(".//mesh")}
    ):
        p = pathlib.Path(filename)
        p = p if p.is_absolute() else file.parent / p
        h.update(filename.encode())
        h.update(p.read_bytes())
    return h.hexdigest()


def pose(T):
    q = Rotation.from_matrix(np.asarray(T)[:3, :3]).as_quat()
    return Pose(position=tuple(np.asarray(T)[:3, 3]), quat_wxyz=tuple(q[[3, 0, 1, 2]]))


class CalibratedArm(XArm7):
    def urdf_path(self):
        return str(ROOT / "inputs/xarm7_calibrated.urdf")

    def _load_urdf(self, builder):
        from real2sim.traj.planning_frame import SensorTCP

        SensorTCP(
            ROOT
        )  # Reject missing sensor mounts or detached/missing custom colliders.
        runtime = self.urdf_path()
        urdf_signature(runtime)
        builder.add_urdf(
            str(runtime),
            xform=self._base_xform(),
            enable_self_collisions=False,
            parse_visuals_as_colliders=False,
        )

    def _write_joint_setup(self, builder):
        super()._write_joint_setup(builder)
        # Current Newton URDF importer creates mimic equality constraints; only the
        # leader is actuated. Six independent drives over-actuated the old adapter.
        if len(builder.constraint_mimic_joint0) != 5:
            raise RuntimeError("Expected five gripper mimic constraints")
        builder.joint_target_ke[7] = getattr(self, "gripper_kp_override", 200.0)
        builder.joint_target_kd[7] = getattr(self, "gripper_kd_override", 5.0)
        builder.joint_effort_limit[7] = getattr(self, "gripper_effort_override", 1.0)
        for i in range(8, 13):
            builder.joint_target_ke[i] = 0.0
            builder.joint_target_kd[i] = 0.0
            builder.joint_effort_limit[i] = 1.0


def build(
    engine="mujoco_fast",
    with_bar=True,
    episode=0,
    offset=(0, 0, 0),
    friction=None,
    size_scale=1.0,
    arm_kp=None,
    tactile_config=None,
):
    s = json.loads((ROOT / "scene_config.json").read_text())
    T = np.array(s["T_sim_base"])
    p = pose(T)
    if size_scale <= 0:
        raise ValueError("size_scale must be positive")
    oldh = s["bar"]["size_m"][2]
    s["bar"]["size_m"] = (np.array(s["bar"]["size_m"]) * size_scale).tolist()
    s["bar"]["position_m"] = (
        np.array(s["bar"]["position_m"])
        + np.array(s["table_matrix"])[:3, 2] * (s["bar"]["size_m"][2] - oldh) / 2
    ).tolist()
    data = np.load(ROOT / "inputs/episodes" / f"{episode:03d}.npz")
    # Object pose is explicit per case; never infer it from the trajectory being evaluated.
    s["bar"]["position_m"] = (
        np.array(s["bar"]["position_m"]) + np.asarray(offset)
    ).tolist()
    if friction is not None:
        s["bar"]["friction"] = friction
    controller = s.setdefault("controller", {})
    if arm_kp is not None:
        controller["arm_kp"] = arm_kp
    info = XArm7Info(
        home_q=tuple(data["q"][0]),
        arm_kp=controller.get("arm_kp", 2500.0),
        arm_kd=controller.get("arm_kd", 80.0),
    )
    # world.tcp_body_index is a compatibility field used by generic diagnostics;
    # this adapter's authoritative TCP is SensorTCP.actual_pose(), so point that
    # field at the flange instead of reviving the removed fixed link_tcp body.
    info.tcp_link = "link7"
    robot = CalibratedArm(
        base_position=p.position, base_quat_wxyz=p.quat_wxyz, info=info
    )
    robot.gripper_kp_override = controller.get("gripper_kp", 200.0)
    robot.gripper_kd_override = controller.get("gripper_kd", 5.0)
    robot.gripper_effort_override = controller.get("gripper_effort_limit_Nm", 1.0)
    robot._desired_engine = engine
    if tactile_config is not None:
        from newton_gen.tactile.tacsim_api import MountFrame, NewtonLinkMountTarget

        mounts = tactile_config["mounts"]
        from real2sim.traj.planning_frame import SensorTCP

        tcp = SensorTCP(ROOT)
        for side in ("left", "right"):
            if mounts[side]["body"] != side + "_finger" or not np.allclose(
                mounts[side]["T_body_sensor"],
                tcp.mounts[side + "_finger"],
                atol=1e-7,
                rtol=0,
            ):
                raise ValueError(
                    "Tactile mount disagrees with current robot sensor center: " + side
                )

        def tactile_mount_targets():
            return tuple(
                NewtonLinkMountTarget(
                    MountFrame(
                        side, np.asarray(mounts[side]["T_body_sensor"], dtype=float)
                    ),
                    mounts[side]["body"],
                    replace_link_colliders=False,
                )
                for side in ("left", "right")
            )

        robot.tactile_mount_targets = tactile_mount_targets
        robot.set_tactile_sensor("photon")
    sim_substeps = (
        int(tactile_config.get("physics", {}).get("sim_substeps", 20))
        if tactile_config
        else 20
    )
    cfg = SimConfig(
        engine=engine, fps=30, sim_substeps=sim_substeps, contact_surface_observer=False
    )
    if tactile_config is not None:
        # The tactile clock IS the trajectory clock: one replayed frame is one control period,
        # so cfg.fps has to be exactly control_hz.  Rounding here instead of refusing would make
        # the replay integrate a different duration than the timestamps it records.
        control_hz = tactile_config["control_hz"]
        if not float(control_hz).is_integer():
            raise ValueError("control_hz must be a whole number of frames per second")
        cfg.fps = int(control_hz)
        cfg.force_log = True
        cfg.tactile_force_source = "native"
    table = ObjectSpec(
        id="table",
        size=tuple(s["table_size_m"]),
        pose=pose(s["table_matrix"]),
        dynamic=False,
    )
    b = s["bar"]
    q = np.array(b["quaternion_xyzw"])
    bx, by, bz = b["size_m"]
    bm = b["mass_kg"]
    b["inertia_diagonal_kg_m2"] = [
        bm * (by * by + bz * bz) / 12,
        bm * (bx * bx + bz * bz) / 12,
        bm * (bx * bx + by * by) / 12,
    ]
    bar = ObjectSpec(
        id="bar",
        size=tuple(b["size_m"]),
        pose=Pose(position=tuple(b["position_m"]), quat_wxyz=tuple(q[[3, 0, 1, 2]])),
        mass=b["mass_kg"],
    )
    scene = SceneSpec(
        name="EEFAlignmentV1",
        table=table,
        objects=[bar] if with_bar else [],
        ground=False,
    )
    if engine == "mujoco":
        import newton_gen.sim.world as world_module

        original = world_module.sdf_resolution_for_extent
        world_module.sdf_resolution_for_extent = lambda extent, cfg: 8 * (
            (original(extent, cfg) + 7) // 8
        )
    try:
        world = build_world(scene, robot, cfg, load_visuals=False)
    finally:
        if engine == "mujoco":
            world_module.sdf_resolution_for_extent = original
    return world, s


if __name__ == "__main__":
    wp.init()
    w, s = build(with_bar=False)
    m = w.model
    state = m.state()
    newton.eval_fk(m, m.joint_q, m.joint_qd, state)
    print("BODIES", m.body_label)
    print("JOINTS", m.joint_label)
    print("Q", m.joint_q.numpy())
    print("TCP", w.tcp_body_index, state.body_q.numpy()[w.tcp_body_index])
    np.savez(
        ROOT / "reports/model_probe.npz",
        body_q=state.body_q.numpy(),
        q=m.joint_q.numpy(),
    )
    (ROOT / "reports/model_labels.json").write_text(
        json.dumps(
            {
                "body": m.body_label,
                "joint": m.joint_label,
                "tcp_index": w.tcp_body_index,
                "ee_index": w.ee_link_index,
            },
            indent=2,
        )
    )
