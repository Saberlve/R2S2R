# Sensor-center planning with unchanged recorded EEF

The xArm7 controller TCP remains `[0, 0, 0.172]` m. Existing capture, import,
replay, and export fields retain their controller TCP definition.

Opt into sensor-center waypoint planning in a task JSON:

```json
"planning_frame": {
  "kind": "sensor_center",
  "transform_file": "/absolute/case/path/sensor_planning_frame.json"
}
```

Without this field, waypoints still target the original controller TCP.
With it, waypoint positions/orientations describe the midpoint between the two
sensor contact surfaces, with orientation defined by the supplied calibration.
The transform file is case data outside Git. Relative paths resolve against the task JSON.

File contract: `schema_version="1.0"`, `length_unit="m"`,
`gripper_convention="0=open,1=closed"`, `transform_convention="T_tcp_sensor"`,
`T_flange_tcp` (4x4 original controller TCP), `gripper_closed_fraction` (strictly
increasing samples including 0 and 1), and corresponding `T_tcp_sensor` matrices.
Each matrix maps sensor coordinates into original TCP coordinates. Position is
interpolated linearly and rotation with Slerp. Extrapolation is rejected.
The table must use the same closed-fraction-to-drive mapping as the replay adapter,
not an assumption that normalized gripper is proportional to joint angle.

Planning interpolates sensor poses and opening first, then uses each frame's opening:

`T_base_tcp_target = T_base_sensor_target @ inverse(T_tcp_sensor(gripper))`

The unchanged TCP172 IK objective solves that target. The trajectory's `q` is
the resulting arm configuration, and `tcp_m/tcp_quat_xyzw` are **original TCP FK
of that saved q**, not sensor poses and not copied IK targets. Capture observations
are not transformed. Action/observation channels are not redefined. Sensor goals
are planning inputs; transform provenance is kept in the plan report and manifest.

Gripper-dependent compensation here uses planned opening, not measured servo
feedback. Motion planning remains IK-only: no new collision, contact, inertia,
or real-execution validation is implied. Actual hardware is never commanded.
