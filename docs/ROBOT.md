# Newton / xArm7 适配器

通用场景契约与机器人专用控制分离。本版移植了 EEFAlignmentV1 中实际运行的7轴模型、Newton IK、MuJoCo/native与hydroelastic接触、G2联动和dry-run导出。修复了两项不可泛化行为：移除代码中的实验室绝对路径；移除“从正在评估的轨迹反推物体初始位姿”的自动逻辑。新场景必须在配置中显式给出估计或实测物体位姿。

## 当前适用范围

xArm7、7个arm自由度、G2平行夹爪、5个mimic约束、TCP `[0,0,0.172]m`。CLI遇到不同TCP直接拒绝。其他TCP/夹爪/机器人要实现与验证新的adapter，不能仅改视觉网格。

只驱动夹爪leader，从动关节由mimic约束驱动。normalized→开口→角度使用几何反解，不直接套用线性角度。当前力矩/响应仍是估算，不能解释为真实SDK力百分比。

## case目录输入

Case数据在Git外，可在任意位置；`--case`注入，底层通过R2S_CASE_ROOT访问。`--newton-project`指向已有Data-MechanicSim工程，其`.venv/bin/python`与editable Newton保持原样。

```text
case/
  scene_config.json
  inputs/
    xarm7_calibrated.urdf
    episodes/000.npz ...
    gripper_mapping.json
    robot_meshes/...
  results/
  reports/
```

scene_config必需：`T_sim_base`（4×4）、`tcp_offset_m=[0,0,.172]`、`table_matrix`、`table_size_m`；`bar`含size_m、mass_kg、position_m、quaternion_xyzw、friction、estimated；可选controller含arm_kp/arm_kd/gripper_kp/gripper_kd/gripper_effort_limit_Nm。尺寸与摩擦须记录来源/不确定范围。

URDF必须是该7轴+G2链，使用控制器标定joint origins；所有mesh路径可解析。官方物理链与视觉网格通过固定T_body_object绑定，禁止为了视觉匹配扭曲物理骨架。不要复用没有对应来源hash的碰撞缓存；新case初次构建生成缓存。

```bash
export PYTHONPATH=/path/to/real2sim-pipeline/src
PY=/path/to/Data-MechanicSim/.venv/bin/python
$PY -m real2sim.cli traj xarm7 --case /path/to/case --newton-project /path/to/Data-MechanicSim validate_kinematics -- --episode 0
$PY -m real2sim.cli traj xarm7 --case /path/to/case --newton-project /path/to/Data-MechanicSim simulate_replay -- --mode joint --episode 0 --tag joint_check
$PY -m real2sim.cli traj xarm7 --case /path/to/case --newton-project /path/to/Data-MechanicSim simulate_replay -- --mode eef --episode 0 --bar --tag eef_contact
```

先跑所有episode运动学，再做无接触跟踪，再做接触。标定/关节范围必须覆盖本次轨迹。生成示例`generate_grasp`使用episode000的初始姿态和配置中的单个box，不是通用任务规划器；`import_eef`接收自主TCP轨迹。当前simulate_replay支持单桌+单box物理任务；多物体、其他碰撞几何需要扩展SceneSpec adapter后测试。

## 当前未验证能力

自碰撞沿用旧加载器设置，未完整开启；真实夹爪力/开合响应未辨识；真实长条回放不能通过可信接触门槛。`trajectory_adapter --execute`明确拒绝真机执行。框架没有SDK运动入口，也没有调用会enable/reset的采集类。

成功的生成抓取、纯运动学渲染、真实轨迹复现是三种不同证据。必须在报告中分别列出，不能互相代替。
