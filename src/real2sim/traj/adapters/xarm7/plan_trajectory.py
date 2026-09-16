"""Plan a joint trajectory from a waypoint task spec; no hardware, no collision checking.

Task JSON (base frame, xyzw quaternion): schema_version 1.0, waypoints[]
(position_m, quaternion_xyzw, gripper 0=open..1=closed, duration_s), optional
q_start_rad, fps, vel_limits_rad_s/acc_limits_rad_s2. On any infeasibility the
report is written and the exit code is 2 — a trajectory is never faked.
"""
import argparse,json,pathlib,sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4]))

from real2sim.traj.planner import (
    Infeasible, JointTrajectory, Waypoint, interpolate_waypoints, validate_limits, validate_task,
)
from real2sim.contracts import rigid, save, sha
from real2sim.traj.planning_frame import PlanningFrame, recorded_tcp_poses


def base_waypoints_to_sim(task, T_sim_base):
    from scipy.spatial.transform import Rotation
    out = []
    for w in task["waypoints"]:
        T = T_sim_base @ _pose(w.position_m, w.quaternion_xyzw)
        out.append(Waypoint(tuple(T[:3, 3]), tuple(Rotation.from_matrix(T[:3, :3]).as_quat()),
                            w.gripper, w.duration_s))
    return out


def _pose(xyz, xyzw):
    from scipy.spatial.transform import Rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(xyzw).as_matrix()
    T[:3, 3] = xyz
    return rigid(T, "waypoint pose")


def solve(task_spec, out_dir):
    """Returns (episode dict, report dict), or raises Infeasible. Newton imports stay lazy."""
    import warp as wp
    import newton
    import newton.ik as ik
    from scipy.spatial.transform import Rotation
    from physics_model import ROOT, CalibratedArm, XArm7Info, pose
    from trajectory_adapter import transform

    task = validate_task(json.loads(pathlib.Path(task_spec).read_text(encoding="utf-8")))
    planning_frame = PlanningFrame(task["planning_frame"], pathlib.Path(task_spec).resolve().parent)
    settings = json.loads((ROOT / "scene_config.json").read_text())
    B = rigid(np.array(settings["T_sim_base"], float), "T_sim_base")
    q0 = task["q_start_rad"]
    if q0 is None:
        q0 = np.load(ROOT / "inputs/episodes/000.npz")["q"][0].tolist()

    wp.init()
    newton.use_coord_layout_targets = True
    base = pose(B)
    robot = CalibratedArm(base_position=base.position, base_quat_wxyz=base.quat_wxyz,
                          info=XArm7Info(home_q=tuple(q0)))
    builder = newton.ModelBuilder()
    robot._load_urdf(builder)
    robot._write_joint_setup(builder)
    model = builder.finalize()
    flange = next(i for i, n in enumerate(model.body_label) if n.endswith("/link7"))

    state = model.state()
    newton.eval_fk(model, state.joint_q, state.joint_qd, state)  # joint_q is home_q, which equals q_start
    f = state.body_q.numpy()[flange]
    start = transform(f[:3], f[3:])
    start[:3, 3] += start[:3, :3] @ np.array([0, 0, 0.172])
    start = planning_frame.planning_pose(start, task["waypoints"][0].gripper)
    start_wp = Waypoint(tuple(start[:3, 3]), tuple(Rotation.from_matrix(start[:3, :3]).as_quat()),
                        task["waypoints"][0].gripper, 1e-6)
    sim_waypoints = [start_wp] + base_waypoints_to_sim(task, B)
    times, positions, quats, grippers = interpolate_waypoints(sim_waypoints, task["fps"])

    po = ik.IKObjectivePosition(link_index=flange, link_offset=wp.vec3(0, 0, .172),
                                target_positions=wp.array([wp.vec3()], dtype=wp.vec3))
    ro = ik.IKObjectiveRotation(link_index=flange, link_offset_rotation=wp.quat_identity(),
                                target_rotations=wp.array([wp.vec4(0, 0, 0, 1)], dtype=wp.vec4))
    li = ik.IKObjectiveJointLimit(joint_limit_lower=model.joint_limit_lower,
                                  joint_limit_upper=model.joint_limit_upper)
    solver = ik.IKSolver(model=model, n_problems=1, objectives=[po, ro, li], lambda_initial=.01,
                         jacobian_mode=ik.IKJacobianType.ANALYTIC)
    q = wp.array(model.joint_q.numpy().reshape(1, -1), dtype=wp.float32)
    qs, errs, actual_tcp = [], [], []
    for planning_pos, planning_quat, gripper in zip(positions, quats, grippers):
        # Interpolate in sensor space first; convert each frame at its own opening.
        target = planning_frame.tcp_target(_pose(planning_pos, planning_quat), gripper)
        pos, quat = target[:3, 3], Rotation.from_matrix(target[:3, :3]).as_quat()
        po.target_positions.assign(np.array([pos], np.float32))
        ro.target_rotations.assign(np.array([quat], np.float32))
        solver.step(q, q, iterations=40)
        v = q.numpy()[0]
        qs.append(v[:7].copy())
        state.joint_q.assign(v)
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)
        f = state.body_q.numpy()[flange]
        actual = transform(f[:3], f[3:])
        actual[:3, 3] += actual[:3, :3] @ np.array([0, 0, 0.172])
        actual_tcp.append(actual.copy())  # Original controller EEF, not sensor pose.
        errs.append([np.linalg.norm(pos - actual[:3, 3]) * 1000,
                     Rotation.from_matrix(Rotation.from_quat(quat).as_matrix().T @ actual[:3, :3]).magnitude() * 180 / np.pi])
    errs = np.array(errs)
    fk_pos_mm, fk_rot_deg = float(errs[:, 0].max()), float(errs[:, 1].max())
    if fk_pos_mm > 1.0 or fk_rot_deg > 0.1:
        raise Infeasible("ik_accuracy", "FK gate failed: max %.3f mm / %.3f deg" % (fk_pos_mm, fk_rot_deg))

    qs = np.array(qs)
    traj = JointTrajectory(times=times, positions=qs,
                           joint_names=tuple("joint_%d" % (i + 1) for i in range(7)),
                           fk_error_m=fk_pos_mm / 1000.0)
    violations = []
    if task["vel_limits_rad_s"] is not None:
        violations = validate_limits(traj, task["vel_limits_rad_s"], task["acc_limits_rad_s2"])
        if violations:
            raise Infeasible("limits", json.dumps(violations[:5]))

    episode = {
        "time": times,
        "q": qs,
        "action_q": np.concatenate([qs[1:], qs[-1:]], axis=0),
        "gripper": grippers,
        "action_gripper": np.concatenate([grippers[1:], grippers[-1:]]),
        **recorded_tcp_poses(B, actual_tcp),
    }
    report = {
        "planner": "real2sim.traj.planner + newton.ik (analytic Jacobian)",
        "collision_checking": "not_implemented",
        "frames": int(len(times)),
        "duration_s": float(times[-1]),
        "fps": task["fps"],
        "time_step_contract": "uniform 1/fps grid; segment durations quantized to whole frames",
        "max_fk_error_mm": fk_pos_mm,
        "max_fk_error_deg": fk_rot_deg,
        "limit_violations": violations,
        "generated_trajectory_not_real_contact_validation": True,
        "planning_frame": planning_frame.metadata,
        "eef_recording": "FK of saved joint q at unchanged controller TCP172; no sensor transform",
    }
    return episode, report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--task", required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args()
    out = pathlib.Path(a.out).resolve()
    if out.exists():
        raise FileExistsError("A run directory must be new: " + str(out))
    out.mkdir(parents=True)
    try:
        episode, report = solve(a.task, out)
    except Infeasible as refusal:
        save(out / "plan_report.json", {"status": "infeasible", "reason": refusal.reason,
                                        "detail": refusal.detail,
                                        "task_sha256": sha(a.task)})
        print("INFEASIBLE", refusal.reason, flush=True)
        sys.exit(2)
    np.savez_compressed(out / "episode.npz", **episode)
    save(out / "manifest.json", {
        "schema_version": "1.0", "joint_unit": "rad", "time_unit": "s",
        "gripper_convention": "0=open,1=closed", "tcp_frame": "robot_base",
        "quaternion": "xyzw", "source": "plan_trajectory",
        "task_sha256": sha(a.task),
        "eef_definition": "controller_tcp",
        "T_flange_tcp": [[1,0,0,0], [0,1,0,0], [0,0,1,.172], [0,0,0,1]],
        "planning_frame": report["planning_frame"],
    })
    save(out / "plan_report.json", {"status": "planned", **report, "task_sha256": sha(a.task)})
    print(json.dumps({"status": "planned", "frames": report["frames"],
                      "max_fk_error_mm": report["max_fk_error_mm"]}))


if __name__ == "__main__":
    main()
