# Sensor-center TCP

The current custom xArm7 gripper uses a sensor-center TCP for all new planning,
replay and offline export. The origin is the midpoint of the two sensor contact
centers declared by `custom_contact_L/R_fix` in the current URDF; axes follow
`link7`. The offset varies with opening. It is derived from the articulated
URDF and the case's gripper command mapping, not a second transform table.

Task waypoints always specify this TCP. `"planning_frame":{"kind":"sensor_center"}`
is optional documentation of that choice. The old `transform_file` option is
rejected, as is replay of NPZ data without scalar `tcp_definition="sensor_center"`.
Regenerate trajectories rather than relabeling old positions.

Planning interpolates sensor poses and opening, then solves the flange target:

`T_base_flange_target = T_base_sensor_target @ inverse(T_flange_sensor(gripper))`

Saved `tcp_m/tcp_quat_xyzw` describe sensor-center FK of the saved arm joints and
planned gripper opening, in robot-base coordinates. During dynamics replay the
actual TCP is computed from both actual finger body poses, including tracking
error, not from commanded opening. Hardware controller and hand-eye calibration
records keep their original definitions; this does not configure a real robot.

The six custom parts belong to the two finger bodies for collision and follow
the same bodies in the repository's Cycles motion renderer. Photon adds gel
contact geometry without removing the rigid sensor parts. Model geometry,
materials and mounting remain uncalibrated for real contact; IK-only planning
does not establish collision-free or hardware-executable motion.
