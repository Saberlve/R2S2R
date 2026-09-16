# 服务器使用手册

## 目录与事实来源

| 内容 | zju 路径（均位于 `/home/wangshuxun/VLA/data_sim/`） |
|---|---|
| 可复用代码、README、配置样例 | `R2S2R/` |
| 案例、原始资料和运行结果 | `real2sim/` |
| 当前模板 | `real2sim/WristCameraAlignmentV1/runtime/` |
| 当前场景入口 | `real2sim/current_scene.json` |
| 最新腕部相机估计及证据 | `real2sim/WristCameraAlignmentV1/calibration.json` |
| Newton 工程与机器人资产 | `R2S2R/external/Data-MechanicSim/`（子模块），`R2S_NEWTON_PROJECT` 可覆盖 |
| 已测量长条抓取案例 | `real2sim/MeasuredBarGraspV1/` |
| 真机回放录像/日志 | `real2sim/MeasuredBarGraspV1/RealReplayComparisonV1/raw/` |
| 以前的标定、扫描处理资料 | `real2sim/SourceData/` |
| 原始手机资料和用户资产 | `real2sim/source_inputs/supplied-originals/` |
| 本地完整迁移快照 | `real2sim/local_imports/20260914/snapshot/` |

这些案例数据不放进 Git，克隆仓库不会凭空获得扫描、视频和渲染结果（机器人网格与 Newton 工程随 `--recursive` 子模块带来，属于例外）。新研究人员在同一服务器可复用同一组 `R2S_*` 环境变量指向现有数据；跨机器搬迁须同时复制数据与依赖源码，并修改这些变量。

## 运行时路径

七个路径可用 `R2S_*` 环境变量覆盖，没有配置文件。能推导的都有仓库内默认值（见下表）；命令只在真正用得上某个路径时才要求它，且在创建输出目录之前报错，不会留下半成品 run。zju 当前的值见 `examples/site.zju.env`。

| 变量 | 含义 | 默认值 |
|---|---|---|
| `R2S_MAIN_PYTHON` | 主环境解释器（numpy、scipy、newton、warp、mujoco、cv2） | 子模块 `.venv/` |
| `R2S_CYCLES_PYTHON` | 带 bpy 的渲染环境解释器 | 子模块 `render_cycles/.venv/` |
| `R2S_FFMPEG` | ffmpeg 可执行文件 | PATH 上的 `ffmpeg` |
| `R2S_NEWTON_PROJECT` | Newton 工程与机器人资产根目录 | `external/Data-MechanicSim` |
| `R2S_MEASURED_CASE` | 已测量长条抓取案例 | 无 |
| `R2S_REFERENCE_TEMPLATE` | 当前参考场景 runtime 目录；`--template` 未指定时的默认值 | 无 |
| `R2S_RUNS_ROOT` | 新输出目录的默认根 | 仓库内 `runs/` |

`./r2s-server site` 打印当前解析到的七个值。载入方式（建议放进 shell 配置，不必每次手动 source）：

```bash
set -a; . examples/site.zju.env; set +a
```

## 研究人员第一次运行

```bash
cd /home/wangshuxun/VLA/data_sim/R2S2R
./r2s-server --help
./r2s-server site
./r2s-server doctor
./r2s-server smoke --samples 4
./r2s-server inspect
```

`doctor` 检查数值/仿真环境、bpy 环境、ffmpeg 与案例路径，缺失的路径项会打印该去补什么；触觉后端 tacsim 只作为非关键项报告解析结果，不影响场景渲染类命令。`smoke` 使用 CPU 跑最小规则场景；不需要 GPU 或真实相机。`inspect` 打开服务器模板并输出 `scene_inventory.json`，列出 Object、Collection、父子关系、米制变换、灯光名称、材质和纹理是否打包。

触觉另有独立自检：`r2s tactile doctor --python <解释器>` 逐项检查 tacsim 的解析与运行条件，关键项缺失退出 2，详见 [DEPENDENCIES.md](DEPENDENCIES.md)。

分配自己的工作目录时，可 `git clone --recursive /home/wangshuxun/VLA/data_sim/R2S2R YOUR_REPO`（`--recursive` 不能省，否则 `external/Data-MechanicSim` 是空目录），把 `examples/site.zju.env` 复制成自己的 `site.local.env` 并把 `R2S_RUNS_ROOT` 改成自己的输出目录。不要多人直接覆盖同一个实验输出。

## 用文件修改现有场景

没有本地或远程 GUI 编辑步骤。先读取 `inspect` 的清单，再编写 patch，例如：

```json
{
  "schema_version": 1,
  "lights": {
    "BG2_WindowFill.001": {"power_W": 15.0}
  },
  "materials": {
    "BG2_BalconyStone": {"albedo_gain": 0.98}
  }
}
```

名称来自当前实验室模板，不是任意场景的固定名称。单位：位置/尺寸为米，功率字段为渲染器瓦特参数，颜色为线性 RGB。带纹理的材质使用 `albedo_gain` 保留纹理；工具拒绝用常量 base color 默默遮盖纹理连接。

```bash
# --out 必须是尚不存在的目录；省略时自动生成唯一目录
./r2s-server edit --patch /absolute/path/appearance.json --out /absolute/path/experiment01
./r2s-server render --template /absolute/path/experiment01/template --gpu 1 --samples 32
```

patch 支持：

- `world_strength`：非负有限数值。
- `lights[name]`：`power_W`、`color_linear`、`position_m`、`target_m`、AREA 灯的 `size_m`。
- `materials[name]`：`base_color_linear`（仅未连接贴图时）、`albedo_gain`、`roughness`（仅未连接时）。
- `object_transforms[name]`：4×4 刚体 `T_world_object`，米制，不接受 scale/shear。

编辑器输出新模板、原/新清单、patch 和编辑报告。它是**视觉场景编辑器**：移动背景或外观不会自动改变 Newton 中的碰撞体、惯量或机器人坐标。修改桌子/物体的物理位置时，还须同步物理 `scene_config.json` 或新场景 schema 并重新验证。机器人关节、附件、相机不能通过此工具的通用变换入口改动，需使用已定义的运动学/相机配置接口，防止视觉与物理悄悄分离。

新建资产、背景几何或更换相机参数时，使用下文的通用 `scene.json` 流程及 [输入输出契约](CONTRACTS.md)；当前腕部变换定义见 `WristCameraAlignmentV1/calibration.json`。固定相机 `T_world_optical` 与腕部 `T_flange_camera` 不可混用。

## 渲染现有模板

```bash
./r2s-server render --gpu 1 --samples 32
# 无 GPU：省略 --gpu，使用 CPU
./r2s-server render --samples 4
# 使用已准备的 runtime 格式状态文件
./r2s-server render --states /absolute/path/states.jsonl --gpu 1 --samples 16
```

输出为 `OUTPUT/template/results/still/` 或 `motion/`，含三相机 PNG、拼接图与绑定验证。当前腕部与 B 均为640×480；Azure保留原来标定比例，三视角拼接时等比缩放并补边，不拉伸成新的相机内参。

这里 `--states` 接受实验室 runtime 格式：每行 `objects[ObjectName].matrix_world` 和时间信息，由已验证 link-to-mesh 绑定生成。**不能直接传 NPZ、关节列表或通用 CLI 的 `T_world_objects` 状态。** 通用 `real2sim.cli scene render` 使用独立 schema，转换方法见 [机器人接口](ROBOT.md)。

## Newton 离线仿真与视频

```bash
# 短程功能检查，不代表抓取完成
./r2s-server simulate --gpu 1 --frames 90 --stride 30 --samples 4
# 完整现有长条任务（481帧，30Hz，约16秒）
./r2s-server simulate --gpu 1 --frames 481 --stride 15 --samples 16
# 只生成物理状态
./r2s-server simulate --gpu 1 --frames 481 --no-render
```

输入：已测量的126×33×33 mm 长条、当前抓取案例配置、控制器标定 URDF、G2 映射、固定基座/桌面变换和生成轨迹 `grasp_clearance.npz`。工具把工作输入复制到新 run，使用 `mujoco_fast`；`--engine mujoco` 可选择既有另一后端，但本轮新入口只对默认后端做了功能测试。

输出：

```text
OUTPUT/
  receipt.json                # 参数、环境路径、退出结果
  simulation.log / render.log / encode.log
  case/scene_config.json      # 本次物理参数
  case/results/eef_mujoco_fast_bar_server/
    states.npz                # Newton实际状态，米+xyzw四元数
    report.json / contacts.json
    trajectory_dry_run.jsonl
    render/                   # 单视角与拼接帧
  ThreeViews.mp4
```

只使用服务器上的数据，没有相机采集、SDK连接、机器人 enable/reset/move。机械臂靠驱动和接触求解运动，长条不附着在夹爪上。本轮迁移只是验证软件入口；完整任务是否抓起以 `report.json` 的高度、持续时间和接触数据判断，不以命令退出0或视频好看判断。真实复现要另行现场执行。

物理任务参数位于输出案例 `scene_config.json`，不可仅修改 Blender 可视网格来声称摩擦、尺寸或碰撞参数已经改变。需要改变任务、物体摆放和轨迹时，应建立新案例并重跑生成/IK/碰撞及动力学检查，参见 `MeasuredBarGraspV1/run_offline.sh` 与原案例说明。

## 新场景完整 pipeline

按 README 的最小资料包准备：固定相机原图、环拍视频、3D Scanner完整导出、桌子尺寸与测距。以下仍是主流程：

```bash
export PYTHONPATH="$PWD/src"
PY=/path/to/main/python
CYCLES=/path/to/cycles/python
$PY -m real2sim.cli case-init /absolute/path/MyScene --cycles-python "$CYCLES"
# 填 intake.json；在服务器准备 raw/、assets/、config/scene.json 与 workflow.json
$PY -m real2sim.cli case-run /absolute/path/MyScene
$PY -m real2sim.cli case-status /absolute/path/MyScene
```

3D扫描背景与Azure内参不可靠时的焦距/外参联合拟合仍属于关键阶段，不能省略。分别看 [Scanner](SCANNER_BACKGROUND.md)、[自动化与人工审核](AUTOMATION.md)；未知内参走 `r2s align fit-camera`，历史 Azure 案例见 `examples/lab_reference/azure/`。必要人工介入是测距端点、语义实体划分、对应点和结果审核；不需要人在 Blender 里拖拽。

## 故障定位

- 先查看 `receipt.json` 和相应 `.log`；失败记录会保留。
- 输出目录存在：换新目录；不要覆盖 frozen baseline 或同名 run。
- `bpy` 导入失败：检查 `cycles_python`，不要装进 Newton 的3.10环境。
- Newton/API不兼容：核对 `environment/newton-source-snapshot.json`；不能靠重新安装任意 PyPI Newton 修复。
- 相机画面反转/裁切：核对 profile 与 pixel_ops，特别是腕部 raw 640×480；不要只改变分辨率字段。
- 找不到资产：克隆 Git 不含案例数据，检查 `R2S_*` 环境变量（`./r2s-server site`）、URDF绝对资产路径、案例 inputs。
- 需要大幅修改机器人/场景位置：同步视觉与物理配置，再验证坐标链，不能让相机补偿几何错误。
