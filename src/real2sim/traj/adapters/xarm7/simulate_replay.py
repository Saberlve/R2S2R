"""Newton dynamics replay. Targets never overwrite simulated state after initialization."""

import argparse, json, numpy as np, warp as wp, newton, pathlib
from scipy.spatial.transform import Rotation
from physics_model import ROOT, build
from real2sim.traj.planning_frame import SensorTCP
from trajectory_adapter import transform, export_trajectory
from newton_gen.sim.engine.registry import get_engine_class

p = argparse.ArgumentParser()
p.add_argument("--mode", choices=["joint", "eef"], default="eef")
p.add_argument("--bar", action="store_true")
p.add_argument("--frames", type=int, default=330)
p.add_argument("--engine", default="mujoco_fast")
p.add_argument("--tag", default="")
p.add_argument("--episode", type=int, default=0)
p.add_argument("--trajectory")
p.add_argument("--tactile-config")
p.add_argument("--offset", type=float, nargs=3, default=[0, 0, 0])
p.add_argument("--friction", type=float)
p.add_argument("--size-scale", type=float, default=1.0)
p.add_argument("--arm-kp", type=float)
a = p.parse_args()
tactile = None
if a.tactile_config:
    from real2sim.tactile.run import (
        validate_tactile_config,
        resample_trajectory,
        sha256,
    )

    tactile = validate_tactile_config(
        json.loads(pathlib.Path(a.tactile_config).read_text()), engine=a.engine
    )
wp.init()
w, settings = build(
    a.engine,
    a.bar,
    a.episode,
    a.offset,
    a.friction,
    a.size_scale,
    a.arm_kp,
    tactile_config=tactile,
)
m = w.model
cfg = w.cfg
data = np.load(
    a.trajectory if a.trajectory else ROOT / "inputs/episodes" / f"{a.episode:03d}.npz"
)
tcp = SensorTCP(ROOT)
tcp.require_episode(data)
if tactile:
    arrays = {name: data[name] for name in data.files}
    arrays, time_report = resample_trajectory(arrays, tactile["control_hz"])
    data = arrays
qtargets = data["q"]
gripper_commands = data["gripper"]
grip = tcp.drive
mu = m.shape_material_mu.numpy()
if a.bar:
    mu[w.object_shape_indices("bar")] = settings["bar"]["friction"]
m.shape_material_mu.assign(mu)
engine = get_engine_class(a.engine)()
if a.engine == "mujoco":
    original_solver = newton.solvers.SolverMuJoCo

    class CompatibleSolver(original_solver):
        def __init__(self, *args, **kw):
            # The pinned Newton runtime predates Data-MechanicSim's optional static
            # friction anchors.  Hydroelastic contact itself remains active; discard
            # only these two unsupported tuning arguments and report that limitation.
            strong_friction_shapes = kw.pop("strong_friction_shapes", None)
            kw.pop("strong_friction_stiffness_scale", None)
            if strong_friction_shapes:
                print(
                    "Hydroelastic strong-friction anchors unavailable in pinned Newton; using MuJoCo friction",
                    flush=True,
                )
            super().__init__(*args, **kw)

    newton.solvers.SolverMuJoCo = CompatibleSolver
    try:
        engine.build(w, cfg)
    finally:
        newton.solvers.SolverMuJoCo = original_solver
else:
    engine.build(w, cfg)
s0 = m.state()
s1 = m.state()
control = m.control()
initial = m.joint_q.numpy()
initial[:7] = qtargets[0]
initial[7:13] = grip(data["gripper"][0])
s0.joint_q.assign(initial)
newton.eval_fk(m, s0.joint_q, s0.joint_qd, s0)
target = control.joint_target_q.numpy()
target[:13] = initial[:13]
control.joint_target_q.assign(target)
# Let drive settle at first pose; bar remains fully dynamic.
for _ in range(30):
    s0, s1 = engine.simulate_substeps(s0, s1, control, cfg.sim_dt)
graph = None
if tactile:
    print("CUDA graph disabled for ordered contact/Photon updates", flush=True)
else:
    try:
        with wp.ScopedCapture(device=m.device) as cap:
            end0, end1 = engine.simulate_substeps(s0, s1, control, cfg.sim_dt)
        assert end0 is s0
        graph = cap.graph
    except Exception as ex:
        print("Graph unavailable", str(ex), flush=True)
flange = next(i for i, n in enumerate(m.body_label) if n.endswith("/link7"))
B = np.array(settings["T_sim_base"])
out = ROOT / "results" / f"{a.mode}_{a.engine}_{'bar' if a.bar else 'free'}"
out = out.with_name(out.name + ("_" + a.tag if a.tag else ""))
out.mkdir(exist_ok=False)
(out / "scene_config.json").write_text(json.dumps(settings, indent=2))
if tactile:
    (out / "tactile_config.json").write_text(json.dumps(tactile, indent=2))
tactile_writer = None
tactile_runtimes = {}
mechanics_backend = None
if tactile:
    from real2sim.tactile.run import TactileRunWriter

    tactile_writer = TactileRunWriter(out / "tactile", tactile)
    import torch
    from newton_gen.tactile.tacsim_api import NewtonTactileSensor

    for side in ("left", "right"):
        tactile_runtimes[side] = NewtonTactileSensor(
            "photon", outputs=("depth", "rgb", "marker"), device="cuda"
        )
    from types import SimpleNamespace
    from newton_gen.tactile.models.hydroshear import build_hydroshear_backend

    facade = SimpleNamespace(
        world=w,
        model=m,
        solver=engine.solver,
        contacts=engine.contacts,
        collision_pipeline=engine.collision_pipeline,
        cfg=cfg,
        contact_surface_observer=None,
    )
    mechanics_backend = build_hydroshear_backend(facade, indenter_object_ids=["bar"])
rows = []
actual = []
targets = []
bodies = []
obj = []
errs = []
tlist = []
glist = []
qout = []
contact_log = []
n = min(a.frames, len(qtargets))
for i in range(n):
    target[:7] = qtargets[i]
    target[7:13] = grip(gripper_commands[i])
    control.joint_target_q.assign(target)
    if graph is not None:
        wp.capture_launch(graph)
    else:
        s0, s1 = engine.simulate_substeps(s0, s1, control, cfg.sim_dt)
    q = s0.joint_q.numpy()
    v = s0.body_q.numpy()
    T = tcp.actual_pose(v, m.body_label)
    goal = B @ transform(data["tcp_m"][i], data["tcp_quat_xyzw"][i])
    err = [
        np.linalg.norm(T[:3, 3] - goal[:3, 3]) * 1000,
        Rotation.from_matrix(T[:3, :3].T @ goal[:3, :3]).magnitude() * 180 / np.pi,
    ]
    actual.append(T)
    targets.append(goal)
    bodies.append(v)
    qout.append(q)
    errs.append(err)
    tlist.append(float(data["time"][i]))
    glist.append(float(gripper_commands[i]))
    if tactile_writer is not None:
        from tacsim import TactileLatentTensor

        sensor_rows = {}
        mounted = w.robot.tactile_sensor.instances
        mechanics_backend.update(s0)
        mechanics_results = mechanics_backend._implementation.latest_results

        def host(value):
            if hasattr(value, "detach"):
                value = value.detach().cpu().numpy()
            value = np.asarray(value)
            return value[0] if value.ndim and value.shape[0] == 1 else value

        for side in ("left", "right"):
            result = mechanics_results[side]
            surface = host(result.surface_vertices_local).astype(np.float32)
            indent = host(result.indentation_m).astype(np.float32)
            force = host(result.force_local).astype(np.float32)
            total = host(result.total_force_local).astype(np.float32)
            contact = host(
                getattr(result, "surface_contact_mask", result.contact_mask)
            ).astype(bool)
            displacement = host(result.surface_displacement).astype(np.float32)
            relative_motion = (
                displacement[contact, :2].mean(0)
                if contact.any()
                else np.zeros(2, np.float32)
            )
            if not all(
                np.isfinite(x).all()
                for x in (surface, indent, force, total, displacement, relative_motion)
            ):
                raise RuntimeError(
                    f"{side} tactile mechanics became non-finite at frame {i}"
                )
            instance = mounted[side]
            T_world_body = transform(
                v[instance.body_index, :3], v[instance.body_index, 3:]
            )
            metadata = {
                "T_world_sensor": T_world_body @ np.asarray(instance.body_from_gel),
                "contact_object_ids": ["bar"] if contact.any() and a.bar else [],
                "relative_motion_m": [
                    float(relative_motion[0]),
                    float(relative_motion[1]),
                    0.0,
                ],
                "forces": {"contact_force_N": total.tolist()},
                "force_availability": {
                    "contact_force_N": True,
                    "mount_reaction_N": False,
                    "photon_vendor_diagnostic": False,
                },
            }
            device = "cuda"
            latent = TactileLatentTensor(
                surface_local=torch.as_tensor(surface, device=device),
                indentation_m=torch.as_tensor(indent, device=device),
                force_local=torch.as_tensor(force, device=device),
                total_force_local=torch.as_tensor(total, device=device),
                max_indent_m=torch.as_tensor(
                    float(indent.max(initial=0)), device=device
                ),
                in_contact=torch.as_tensor(contact.any(), device=device),
                shear_m=torch.as_tensor(relative_motion, device=device),
            )
            frame = tactile_runtimes[side].render_tensor(latent).numpy()
            sensor_rows[side] = {
                **metadata,
                "depth_m": frame.depth_m,
                "rgb": frame.rgb,
                "marker_flow_px": frame.marker_flow,
            }
        tactile_writer.append(
            episode=a.episode,
            frame=i,
            time_s=float(data["time"][i]),
            sensors=sensor_rows,
        )
    if a.bar:
        obj.append(v[w.object_body_index("bar")])
        if engine.contacts is not None:
            engine.solver.update_contacts(engine.contacts, s0)
            c = engine.contacts
            ncontact = int(c.rigid_contact_count.numpy()[0])
            c0 = c.rigid_contact_shape0.numpy()[:ncontact]
            c1 = c.rigid_contact_shape1.numpy()[:ncontact]
            barshapes = w.object_shape_indices("bar")
            mask = np.isin(c0, barshapes) | np.isin(c1, barshapes)
            force = c.force.numpy()[:ncontact]
            distances = engine.solver.mjw_data.contact.dist.numpy().reshape(-1)[
                :ncontact
            ]
            contact_log.append(
                {
                    "frame": i,
                    "bar_contact_count": int(mask.sum()),
                    "sum_contact_force_magnitudes_N": (
                        float(np.linalg.norm(force[mask], axis=-1).sum())
                        if mask.any()
                        else 0.0
                    ),
                    "max_bar_penetration_m": (
                        float(max(0, -distances[mask].min())) if mask.any() else 0.0
                    ),
                }
            )
    if i % 60 == 0:
        print("FRAME", i, "error", err, flush=True)
assert np.isfinite(qout).all()
np.savez_compressed(
    out / "states.npz",
    time=tlist,
    body_labels=np.asarray(list(m.body_label)),
    length_unit="m",
    quaternion_order="xyzw",
    tcp_definition=tcp.definition,
    joint_q=qout,
    body_q=bodies,
    tcp_actual_sim=actual,
    tcp_target_sim=targets,
    gripper=glist,
    bar=obj,
)
report = {
    "tcp": tcp.metadata,
    "mode": a.mode,
    "engine": a.engine,
    "contact_backend": tactile["contact_backend"] if tactile else None,
    "cuda_graph": graph is not None,
    "episode": a.episode,
    "frames": n,
    "p95_position_mm": float(np.percentile(errs, 95, axis=0)[0]),
    "p95_rotation_deg": float(np.percentile(errs, 95, axis=0)[1]),
    "max_error": np.max(errs, axis=0).tolist(),
    "bar_enabled": a.bar,
    "no_object_attachment": True,
    "estimated_object": True,
}
if tactile:
    report.update(
        time_sync=time_report,
        calibration_status="uncalibrated",
        photon_gel_size_m=tactile["photon_gel_size_m"],
        coupling="coupled_hydroelastic",
    )
    report["tactile_status"] = "complete"
if a.bar:
    obj = np.array(obj)
    normal = np.array(settings["table_matrix"])[:3, 2]
    height = (obj[:, :3] - obj[0, :3]) @ normal
    report["max_bar_lift_m"] = float(height.max())
    report["held_5cm_for_2s"] = any(
        np.all(height[i : i + 60] >= 0.05) for i in range(max(0, n - 59))
    )
if contact_log:
    (out / "contacts.json").write_text(json.dumps(contact_log, indent=2))
    report["max_bar_penetration_m"] = max(
        x["max_bar_penetration_m"] for x in contact_log
    )
    report["max_bar_contact_force_sum_N"] = max(
        x["sum_contact_force_magnitudes_N"] for x in contact_log
    )
    report["contact_force_note"] = (
        "sum of contact force magnitudes, not net force or SDK force percent"
    )
if n > 1:
    export_trajectory(
        out / "trajectory_dry_run.jsonl",
        tlist,
        qtargets[:n],
        np.array(qout)[:, :7],
        np.array(targets),
        np.array(actual),
        glist,
        B,
        (m.joint_limit_lower.numpy()[:7], m.joint_limit_upper.numpy()[:7]),
    )
else:
    report["trajectory_export"] = "not_applicable_single_frame_smoke"
if tactile_writer is not None:
    tactile_writer.finalize(
        {
            "trajectory_sha256": sha256(a.trajectory) if a.trajectory else None,
            "scene_config_sha256": sha256(ROOT / "scene_config.json"),
            "runtime": "Data-MechanicSim + tacsim Photon vendor backend",
        }
    )
    from real2sim.tactile.run import write_rgb_videos

    write_rgb_videos(out / "tactile", tactile["control_hz"])
    for runtime in tactile_runtimes.values():
        runtime.close()
(out / "report.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report), flush=True)
