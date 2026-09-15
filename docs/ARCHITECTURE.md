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
│   ├── interface.py        #   【接口预留】TrajectoryGenerator / EngineAdapter / 引擎注册表
│   ├── trajectory.py       #   硬件无关 FK + episode 转换（convert）
│   ├── bridge.py           #   Newton 状态 → 渲染 state JSONL
│   └── adapters/xarm7/     #   Newton 引擎实现（xArm7 + G2 + TCP172，逻辑未改）
│
└── tactile/                # 领域 4：触觉接入（全部预留）
    └── interface.py        #   【接口预留】TactileSensorModel + 契约草案（tacsim 等）
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
| `r2s xarm7 <job>` | `r2s traj xarm7 <job>`（TCP172 守卫不变） |
| —（新） | `r2s tactile describe`（打印触觉契约草案） |
| 顶层不变 | `case-init / case-run / case-status / case-review / run / inventory / check-inventory / freeze` |

`cases.py` 与 `runner.py` 的命令白名单统一来自 `cli.allowlisted()`；旧写法在白名单检查前经 `cli.normalize_command()` 归一化，NAS 上既有 workflow.json 与 pipeline 计划无需修改。

## 预留接口（本轮只定义，不实现）

- **scene/modeling**：视频 + Blender + Agent 初始建模。产物只能是 quality=`estimated` 的草稿 scene.json；禁止覆盖既有场景与冻结基线。
- **scene/spatial**：测距约束 `DistanceConstraint(entity_a, entity_b, distance_m, sigma_m, evidence)`；`validate_constraints()` 已实现，`optimize_spatial()` 求解器预留。实现时必须遵守 PIPELINE.md 变量锁定顺序。
- **traj/interface**：`TrajectoryGenerator` / `EngineAdapter` Protocol 与引擎注册表；xarm7 适配器为当前唯一实现。
- **tactile/interface**：`TactileSensorModel` Protocol 与观测契约草案（taxel 阵列、T_world_sensor、fixed|body 挂载）；tacsim 等后端为计划项。触觉仿真输出不得声称真实接触验证。

## 迁移说明（v0.4.0 → v0.5.0）

- 模块物理迁移：旧扁平模块移入四个子包；`real2sim.bridge` → `real2sim.traj.bridge`，`real2sim.scans` → `real2sim.scene.scans`，以此类推。
- `tools/adopt_wrist_tcp_calibration.py` 与 `tools/import_controller_snapshot.py` 的数学核心分别抽至 `align/wrist.py` 与 `align/base.py`，工具脚本保留原入口行为。
- xarm7 适配器脚本零逻辑改动，仅路径为 `traj/adapters/xarm7/`。
- 历史 validation/ 证据中的旧路径与旧命令记录不再回填修改。
