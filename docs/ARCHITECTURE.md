# 架构：四域结构（v0.5.0）

仓库按四个领域组织 `src/real2sim/`，对应流水线职责划分。所有 JSON 契约、T_A_B 约定、quality 标签与验收阈值不变（见 CONTRACTS.md）。

## 领域划分

```
src/real2sim/
├── contracts.py            # 共享契约内核（T_A_B、rigid、pixel_ops、profile_id、validate_scene）
├── cli.py                  # r2s 分组子命令 + 旧扁平动词别名层
├── cases.py / runner.py    # 工作流引擎 / 计划 DAG 执行器（命令白名单来自 cli）
├── artifacts.py / mcp.py   # 留档与冻结 / 可选 Blender MCP 传输
├── schemas/                # scene.json JSON-Schema
├── workers/                # 独立 bpy worker（Cycles 渲染、扫描资产打包）
│
├── scene/                  # 领域 1：场景视觉重建
│   ├── scans.py            #   3D scanner 检查/注册/纹理烘焙（scan-inspect/register/bake）
│   ├── appearance.py       #   光照/材质有界拟合（fit-appearance）
│   ├── metrics.py          #   MAE/SSIM/LPIPS 评分（score）
│   ├── video.py            #   多机位 mosaic 编码（video）
│   ├── spatial.py          #   【接口预留】测距信息优化空间关系
│   └── modeling/           #   【接口预留】视频 + Blender + Agent 初始建模
│       └── interface.py
│
├── align/                  # 领域 2：真实-仿真对齐
│   ├── cameras.py          #   固定相机 PnP 外参标定（calibrate）
│   ├── camera_fit.py       #   内参未知：焦距+位姿联合拟合（fit-camera）
│   ├── wrist.py            #   腕部相机标定核心（T_flange_camera = T_flange_tcp @ T_tcp_camera）
│   └── base.py             #   基座坐标系对齐：scene_config 校验 + 控制器快照归一化
│
├── traj/                   # 领域 3：轨迹产生
│   ├── interface.py        #   TrajectoryGenerator / EngineAdapter / 引擎注册表
│   ├── planner.py          #   自有 waypoint 规划核心（纯 numpy；设计参考 newton_gen，无运行时依赖）
│   ├── trajectory.py       #   硬件无关 FK + episode 转换（convert）
│   ├── bridge.py           #   Newton 状态 → 渲染 state JSONL
│   └── adapters/xarm7/     #   Newton 引擎实现（含 plan_trajectory 规划作业）
│
└── tactile/                # 领域 4：触觉接入
    ├── interface.py        #   TactileSensorModel Protocol + 观测契约
    ├── photon.py           #   Photon 运行时检查与渲染配置校验（纯逻辑）
    └── photon_worker.py    #   Photon 离线渲染 worker（需 CUDA + OpenGL 环境）

external/Data-TacSim/       # git 子仓库：tacsim 库（Photon 后端），钉在 master 12793a2
```

## CLI 对照表

新命令为分组形式 `r2s <领域> <动词>`。旧扁平写法仍可用（stderr 打印弃用警告，未来版本移除）。

| 旧命令 | 新命令 |
|---|---|
| `r2s validate / build / audit / render` | `r2s scene validate / build / audit / render` |
| `r2s scan-inspect / scan-register / scan-bake` | `r2s scene scan-inspect / scan-register / scan-bake` |
| `r2s fit-appearance / score / video` | `r2s scene fit-appearance / score / video` |
| —（新） | `r2s scene draft-model`（视频初始建模占位，退出码 2） |
| —（新） | `r2s scene spatial-fit`（测距优化占位；约束校验已实现，求解器预留） |
| `r2s calibrate` | `r2s align calibrate`（固定相机 PnP） |
| `r2s fit-camera` | `r2s align fit-camera`（内参未知联合拟合） |
| —（新） | `r2s align base-check`（校验 scene_config 基座对齐字段） |
| `r2s convert` | `r2s traj convert` |
| `r2s xarm7 <job>` | `r2s traj xarm7 <job>`（TCP172 守卫不变；job 含 `plan_trajectory`，见 ROBOT.md） |
| —（新） | `r2s tactile describe`（打印触觉契约） |
| —（新） | `r2s tactile doctor [--python]`（Photon 运行时逐项检查，关键项缺失退出 2） |
| —（新） | `r2s tactile photon-render <config> --out --python`（离线触觉渲染，合成刺激） |
| 顶层不变 | `case-init / case-run / case-status / case-review / run / inventory / check-inventory / freeze` |

`cases.py` 与 `runner.py` 的命令白名单统一来自 `cli.allowlisted()`；旧写法在白名单检查前经 `cli.normalize_command()` 归一化，NAS 上既有 workflow.json 与 pipeline 计划无需修改。

## 预留接口（本轮只定义，不实现）

- **scene/modeling**：视频 + Blender + Agent 初始建模。产物只能是 quality=`estimated` 的草稿 scene.json；禁止覆盖既有场景与冻结基线。
- **scene/spatial**：测距约束 `DistanceConstraint(entity_a, entity_b, distance_m, sigma_m, evidence)`；`validate_constraints()` 已实现，`optimize_spatial()` 求解器预留。实现时必须遵守 PIPELINE.md 变量锁定顺序。

## 触觉：Photon 接入（2026-09-15）

- 后端：git 子仓库 `external/Data-TacSim`（tacsim 库，**无 LICENSE，内部代码**）+ 厂商专有包 xense-sim4.5（部署在子仓库 `third_party/` 忽略区，不进 git；部署记录见 PROVENANCE.md）。
- 运行要求：Python 3.10（厂商 .so 仅 cp310）、CUDA（`NewtonTactileSensor` 强制）、cffi + pyudev（已装入 Data-MechanicSim/.venv）、OpenGL 上下文——headless 主机用 `xvfb-run -a`。
- 适配进程内把 `external/Data-TacSim` 插到 `sys.path[0]`，保证使用本仓库钉住的 tacsim 副本（主 venv 另有 tacsim 解析到其他工作树）。
- 本轮只接通**离线 render_tensor 通路**（合成高斯压痕 → depth/rgb/marker_flow）；Newton 场景内接入（add_to_builder/drive/read_frame）留待下一步。仿真触觉输出不得声称真实接触验证。
- `photon-render` 与 xarm7 物理作业同级，走显式调用，不进 cases/runner 白名单。

## 轨迹：自有规划器（2026-09-15）

- `traj/planner.py`：waypoint 校验、quintic+Slerp 时间参数化、`JointTrajectory`/`Infeasible`、`validate_limits`——纯 numpy/scipy，设计参考 newton_gen `motion/planning/interface.py`，**不 import newton_gen.motion.\***，后续可自行修改。
- `traj/adapters/xarm7/plan_trajectory.py`：task JSON → 逐帧 TCP 目标 → Newton IK → FK 门禁（≤1mm/0.1°）→ episode NPZ + 报告；拒绝时退出码 2 且不产轨迹。本轮不做碰撞检查。

## 迁移说明（v0.4.0 → v0.5.0）

- 模块物理迁移：旧扁平模块移入四个子包；`real2sim.bridge` → `real2sim.traj.bridge`，`real2sim.scans` → `real2sim.scene.scans`，以此类推。
- `tools/adopt_wrist_tcp_calibration.py` 与 `tools/import_controller_snapshot.py` 的数学核心分别抽至 `align/wrist.py` 与 `align/base.py`，工具脚本保留原入口行为。
- xarm7 适配器脚本零逻辑改动，仅路径为 `traj/adapters/xarm7/`。
