# R2S2R

Real2Sim 流程仓库。从实拍资料（照片、环拍视频、3D 扫描）搭出尺度正确、相机约定一致、能接物理仿真的场景：场景 schema 与校验、GLB/规则几何构建、无界面 Cycles 渲染、固定内参 PnP 与焦距+位姿拟合、MAE/SSIM/LPIPS 评分、有界外观拟合、冻结快照、LeRobot v3 导入、Newton 状态桥，以及 xArm7/G2/TCP172 物理适配器。

资料准备、尺寸确认和结果审核仍需人参与，它不是"给段视频自动生成数字孪生"的工具。

代码仓库：<https://github.com/Saberlve/R2S2R>

## 快速开始

```bash
git clone --recursive https://github.com/Saberlve/R2S2R.git   # --recursive 不能省
cd R2S2R

# 两个解释器由 uv 按子模块里的 lockfile 建好；uv 会自行取所需的 Python 版本
(cd external/Data-MechanicSim && uv sync)                 # 主环境 → .venv/，Python 3.10
(cd external/Data-MechanicSim/render_cycles && uv sync)   # 渲染环境 → .venv/，Python 3.11 + bpy 4.5

./r2s-server site                           # 看当前解析到的路径
./r2s-server doctor                         # 检查环境，只读，不安装也不替换依赖
./r2s-server smoke --samples 4              # CPU 最小场景：校验 → 构建 → 渲染
```

`--recursive` 带出子模块 `external/Data-MechanicSim`，Newton 工程、机器人资产、触觉后端 tacsim 和上面两个解释器都在它里面；少写它会留下一个空目录，`doctor` 会报出来，补 `git submodule update --init --recursive` 即可。上游是内网 Git，见 [docs/DEPENDENCIES.md](docs/DEPENDENCIES.md)。

`uv sync` 之外不要再装 Newton：主环境里的 `newton` 是 editable 指向子模块 `third_party/newton` 的，**`pip install newton` 会用一个未经验证的 PyPI 版本顶掉它**。

上面几条**不需要任何配置**：`r2s-server` 默认就用那两个 `.venv`，场景数据也只在真正用得上时才要求。七个 `R2S_*` 环境变量是全部配置项，能推导的都有仓库内默认值：

| 变量 | 默认值 |
|---|---|
| `R2S_MAIN_PYTHON` | `external/Data-MechanicSim/.venv/bin/python` |
| `R2S_CYCLES_PYTHON` | `external/Data-MechanicSim/render_cycles/.venv/bin/python` |
| `R2S_NEWTON_PROJECT` | `external/Data-MechanicSim` |
| `R2S_RUNS_ROOT` | 仓库内 `runs/`（已被 `.gitignore` 忽略） |
| `R2S_FFMPEG` | PATH 上的 `ffmpeg` |
| `R2S_MEASURED_CASE` | 无 —— 案例数据，见 [docs/RESOURCES.md](docs/RESOURCES.md) |
| `R2S_REFERENCE_TEMPLATE` | 无 —— 场景模板，见 [docs/RESOURCES.md](docs/RESOURCES.md) |

只有最后两个要填。把 [`examples/site.example.env`](examples/site.example.env) 复制成仓库根目录的 `site.local.env`（文件名被 `.gitignore` 忽略）写好再 `set -a; . site.local.env; set +a`；`examples/site.zju.env` 是一份填好的实例可照抄。显式设置的值永远优先于默认值，`./r2s-server site` 显示每个路径解析成了什么，`null` 表示既没设也没默认。

数据不进 Git：渲染结果、录像、扫描、模型、数据集放在仓库外，需要有挂载和读权限。唯一例外是仓库级物体资产库 [`assets/`](assets/README.md) —— 只放跨任务复用的小物体，单文件 ≤ 2MB，格式受限，见 [docs/RESOURCES.md](docs/RESOURCES.md)。仓库自带的最小场景 `examples/minimal/` 不需要这些数据。

## 命令参考

`./r2s-server` 是主入口（自带上面的默认值，不需要设 `PYTHONPATH`）；通用 CLI 是 `r2s`，下面统一写成：

```bash
PY=external/Data-MechanicSim/.venv/bin/python
CYCLES=external/Data-MechanicSim/render_cycles/.venv/bin/python
export PYTHONPATH="$PWD/src"
```

### 服务器入口 `./r2s-server`

| 命令 | 作用 |
|---|---|
| `site` | 打印七个运行时路径解析结果 |
| `doctor` | 逐项检查两个解释器、ffmpeg 与路径；缺什么告诉你去哪补 |
| `smoke --samples 4` | CPU 跑最小场景，验证整条链路；不需要 GPU 或相机 |
| `inspect [--template DIR]` | 打开模板输出 `scene_inventory.json`：Object、Collection、父子关系、米制变换、灯光、材质、纹理是否打包 |
| `edit --patch P.json [--template DIR]` | 用 patch 改灯光/材质/物体变换，输出新模板与编辑报告，见下 |
| `render [--gpu 1] [--samples 32] [--states S.jsonl]` | 无界面 Cycles 渲染，默认三相机 |
| `simulate [--gpu 1] [--frames 481] [--stride 15] [--engine mujoco_fast] [--no-render]` | Newton 离线仿真，输出状态、接触数据与三视角视频 |

`render` / `simulate` / `inspect` / `edit` 需要 `R2S_REFERENCE_TEMPLATE`；`simulate` 还需要 `R2S_MEASURED_CASE`。缺了会在创建输出目录**之前**报错，不会留下半成品 run。每条命令输出唯一的 `OUTPUT` 目录和 `receipt.json`。

```bash
./r2s-server inspect
./r2s-server render --gpu 1 --samples 32
./r2s-server simulate --gpu 1 --frames 481 --stride 15 --samples 16
```

`render` 的 `--states` 只接受实验室 runtime 格式（每行 `objects[Name].matrix_world` 加时间戳），**不能直接传 NPZ 或通用 CLI 的状态**；转换见下面的 `tools/export_saved_states.py`。

`edit` 是**视觉**编辑器 —— 移动背景或改外观不会自动改变 Newton 里的碰撞体、惯量或机器人坐标；改桌子/物体的物理位置必须同步物理配置并重新验证。机器人关节、附件、相机不能走通用变换入口。

### 场景重建 `r2s scene`

| 命令 | 作用 |
|---|---|
| `validate SCENE` | 按 schema 校验场景 JSON |
| `build --scene S --out D --python "$CYCLES"` | 构建 `.blend` |
| `audit --scene S --out D --python "$CYCLES"` | mesh 审计 |
| `render --scene S --blend B --out D --python "$CYCLES" [--samples 32] [--states M.jsonl]` | 渲染 |
| `scan-inspect --obj O --texture T --out D` | 看扫描件：几何、UV、纹理、单位 |
| `scan-register CONFIG --out D` | 对齐到场景坐标系 |
| `scan-bake CONFIG --out D --python "$CYCLES"` | 裁切、清理、烘焙成闭合 GLB |
| `fit-appearance CONFIG --out D` | 有界外观拟合（不允许顺便移动相机或物体） |
| `score --real R --sim S --mask M --out D [--lpips-cache F]` | 区域 MAE/SSIM，可选 LPIPS |
| `video --manifest M.json --cameras CameraB Azure Wrist --out V.mp4` | 多视角拼接，需要 ffmpeg |

```bash
"$PY" -m real2sim.cli scene validate examples/minimal/scene.json
"$PY" -m real2sim.cli scene build  --scene examples/minimal/scene.json --out runs/example/scene.blend --python "$CYCLES"
"$PY" -m real2sim.cli scene render --scene examples/minimal/scene.json --blend runs/example/scene.blend \
  --out runs/example/render --python "$CYCLES" --samples 32
```

3D 扫描背景的完整流程见 [docs/SCANNER_BACKGROUND.md](docs/SCANNER_BACKGROUND.md)。渲染器正式输出 RGB 与有效像素 mask，**不输出深度**；需要深度或实例 ID 得加后端并通过专门测试。Blender worker 环境需有 bpy、numpy、Pillow，渲染 OpenCV Brown 畸变图还需 opencv-python-headless。

`scene draft-model` 和 `scene spatial-fit` 是**保留接口，尚未实现**，调用会直接抛 `NotImplementedError`：几何语义拆分、遮挡补全和独立实测点选取目前仍由人或建模 Agent 完成。

### 对齐 `r2s align`

| 命令 | 作用 |
|---|---|
| `calibrate --scene S --camera C --points P --out D` | 已知内参的固定相机 PnP 外参标定 |
| `fit-camera CONFIG --out D` | 内参未知：焦距+位姿七参数联合拟合，样例见 `examples/lab_reference/azure/` |
| `base-check CONFIG` | 校验 `scene_config.json` 的基座对齐字段 |

自动 PnP 的输出默认标为 `image_fitted`，**不会**因为训练点重投影误差小就当成独立标定成功。需要独立边缘或另一视角交叉检查。

### 轨迹 `r2s traj`

```bash
"$PY" -m real2sim.cli traj convert --input IN --calibration CAL --out D
"$PY" -m real2sim.cli traj xarm7 JOB --case CASE_DIR [-- args]
```

`JOB` ∈ `validate_kinematics`、`simulate_replay`、`generate_grasp`、`gripper_mapping`、`import_eef`、`trajectory_adapter`、`plan_trajectory`。`--newton-project` 不传就用 `R2S_NEWTON_PROJECT`，再不传就用仓库内子模块。目前只验证了 xArm7 + G2 + 172mm TCP，其他机器人/夹爪/TCP 需要新适配器；`plan_trajectory` 用自有 waypoint 规划器，**不做碰撞检查**。详见 [docs/ROBOT.md](docs/ROBOT.md)。

### 触觉 `r2s tactile`

```bash
"$PY" -m real2sim.cli tactile describe
"$PY" -m real2sim.cli tactile doctor --python "$PY"      # 关键项缺失退出 2
xvfb-run -a "$PY" -m real2sim.cli tactile photon-render \
  examples/tactile/photon_minimal.json --out /tmp/photon_demo --python "$PY"
```

`doctor` 逐项检查 tacsim 解析、厂商 bundle、Python 3.10、cffi/pyudev、CUDA，并报出**实际解析到的** tacsim 路径。触觉需要 CUDA 和 OpenGL 上下文（无显示器用 `xvfb-run -a`）。tacsim 由 `--python` 指定的解释器自行解析，本仓库不持有副本、也不把它写进依赖：**不要 `pip install tacsim`**，它没有 PyPI 包、没有 LICENSE，声明它会让外网主机上的 `pip install -e .` 直接失败。当前只接入离线合成刺激的渲染通路，Newton 场景内触觉尚未实现。

### 物体资产库 `r2s asset`

跨任务的物体放在仓库级资产库 `assets/<object_id>/`，一个物体只描述一次。**本轮只有库、校验与浏览，`scene.json` 与 task.json 还不能按 id 引用它**，字段集也等真实物体示例后定稿：

```bash
"$PY" -m real2sim.cli asset check          # 逐个校验，失败退出 2
"$PY" -m real2sim.cli asset list
"$PY" -m real2sim.cli asset show TestBox
```

规则见 [`assets/README.md`](assets/README.md)：目录名即 id，`asset.json` 记几何、物理与来源，缺的段落表示"尚未确定"，消费者必须拒绝而不是补默认值。

### 工作流与案例 `r2s` 顶层

| 命令 | 作用 |
|---|---|
| `case-init CASE` | 初始化新场景目录（必须尚不存在），建 `intake.json`/`runtime.json`/`workflow.json` 等 |
| `case-run CASE [--retry-failed]` | 按 `workflow.json` 跑阶段；复用未变化的阶段，每次重跑新建 attempt |
| `case-status CASE` | 打印各阶段状态、产物与本次审核 token |
| `case-review CASE --stage S --token T --decision approve\|reject --by NAME --note TEXT` | 记一次可追溯审核 |
| `run PLAN --out D` | 按 pipeline JSON 跑固定阶段序列，输出 `pipeline_receipts.json` |
| `freeze --scene S --blend B --out D` | 冻结视觉 baseline |
| `inventory ROOT --out D` / `check-inventory MANIFEST` | 生成/校验资产清单 |

`case-run` 完成返回 0，暂停或阶段失败返回 2，读 `TODO.md` 和 `runs/status.json` 区分原因。输出目录必须新建，不覆盖旧的 attempt、也不覆盖 frozen baseline。这是可追溯的审核约定，不是用户身份认证。

### 工具 `tools/`

| 脚本 | 作用 |
|---|---|
| `export_saved_states.py --states S.npz --bindings B.json --out M.jsonl` | Newton 状态 → 渲染用 JSONL（不做物理步进、不碰硬件） |
| `import_lerobot_v3.py` | 只读导入 LeRobot v3 → 规整 episode NPZ + 原视频索引；需先 `pip install -e '.[dataset]'`（pyarrow） |
| `import_controller_snapshot.py` | 规整已有的 xArm 控制器快照，不连 SDK |
| `match_camera_profile.py` | 零畸变同光路重采样，在两个显式 profile 之间换算 |
| `adopt_wrist_tcp_calibration.py` | 用实测 TCP→相机变换生成新的腕部 runtime |
| `capture_realsense.py` | 相机采集；默认 dry-run，**只有显式 `--capture` 才拍**；从不接触机器人 |
| `environment_snapshot.py --check F.json` | 记录/核对机器人适配器所用的外部源码树 |
| `import_local_snapshot.py` | 校验快照并补齐缺失文件，服务器冲突一律保留 |
| `check_git_payload.py` | 提交前检查：拒绝资源二进制与大于 5MB 的 blob；`assets/` 内的物体例外（≤2MB、限格式、纹理限 `textures/`） |

从 Newton 状态出视频：

```bash
"$PY" tools/export_saved_states.py --states /path/to/states.npz --bindings /path/to/bindings.json --out runs/motion/states.jsonl
"$PY" -m real2sim.cli scene render --scene /path/to/scene.json --blend /path/to/scene.blend \
  --states runs/motion/states.jsonl --out runs/motion/render --python "$CYCLES"
"$PY" -m real2sim.cli scene video --manifest runs/motion/render/render_manifest.json \
  --cameras CameraB Azure Wrist --out runs/motion/three_views.mp4
```

必须先按实际 link-to-mesh 关系准备 binding。用 `--stride` 抽取状态会改变渲染 FPS，不改变物理时间。

上面这些命令走 `PYTHONPATH="$PWD/src"`，依赖由主环境提供。仓库自身还有几个可选依赖组（`calibration`=opencv、`metrics`=scikit-image、`dataset`=pyarrow、`test`=pytest），缺哪个就 `pip install -e '.[组名]'` 补哪个。

## 新场景怎么做

### 1. 准备资料

最小资料包：每台固定相机的原图、房间环拍视频、3D Scanner 完整导出、实验桌尺寸。为了少来回补拍，最好再给几条距离锚点和相机实际采集设置。机器人运动与接触仿真需要额外数据，仅凭这个资料包不能验收。

| 资料 | 怎么提供 | 用途与缺失影响 |
|---|---|---|
| 每台固定相机原始图片 | 原始 RGB 文件，不截图、不额外裁剪；尽量与目标场景状态及照明一致 | 视角、遮挡、光照和颜色拟合的目标；手机照片不能替代 |
| 房间环拍视频 | 缓慢走动，覆盖桌子、墙角、门窗与邻桌，多视角有重叠 | 建立空间关系、判断遮挡与语义分离 |
| 3D Scanner 完整导出 | 保留 ZIP 中的 OBJ/MTL 或 GLB、纹理、UV、单位/导出元数据；不要只给预览截图 | 扫描背景的几何与实拍纹理；仍需裁切、清理、注册和闭合检查 |
| 主桌长×宽×高 | 单位明确，建议米；高度说明是否到桌面上表面 | 尺度与水平桌面约束 |
| 几条距离锚点 | 桌边到墙、邻桌前沿、门宽高等；照片标明测量起止点 | 避免用错误背景或相机位姿补偿空间误差；没有时记录不确定性 |
| 相机采集设置 | 型号、实际分辨率、裁剪/缩放/翻转；能获取则附 K、畸变和 SDK 元数据 | 无可靠内参时走焦距+位姿拟合，输出标为 `image_fitted` |
| 排除清单 | 明确不建模的椅子、线缆、设备等 | 统一建模范围和图像评分 mask |

额外照片优先补拍门窗转角、扫描缺损和材质细节；测距截图只用于几何判断，不直接烘焙成纹理。不要求一开始就人工提供相机外参，但对应点需要独立边缘或另一视角交叉检查。

要做关节/EEF 回放与接触抓取，另外需要：机器人模型及标定、TCP/夹爪配置、带时间戳的关节/EEF 数据和相机视频关联、物体尺寸质量和初始位姿（可从图像估计，但必须标注估计范围）。

### 2. 建案例并迭代

```bash
export PYTHONPATH="$PWD/src"
PY=external/Data-MechanicSim/.venv/bin/python
CASE=/path/to/MyNewScene          # 放在 Git 外

"$PY" -m real2sim.cli case-init "$CASE"     # 只初始化一次；目录必须尚不存在
# 把原始资料放入 raw/，填 intake.json；之后始终运行这条命令
"$PY" -m real2sim.cli case-run "$CASE"
```

新场景目录：

```text
MyNewScene/
  intake.json          # 你提供的文件路径、尺寸、相机设置、排除项
  runtime.json         # 已有运行环境、GPU 选择（默认 CPU）
  workflow.json        # Agent 配置阶段、依赖、参数及输入输出
  AGENT_HANDOFF.md     # Agent 工作约定
  raw/                 # 原始资料，保持不变
  assets/              # 本场景分离后的资产、纹理（与仓库级资产库 assets/ 不是一回事，见 assets/README.md）
  config/              # scene.json、拟合配置、对应点、evidence.md
  runs/                # 每阶段独立 attempt、日志、哈希、状态及审核记录
  TODO.md              # 当前下一步
```

默认流程：资料检查 → Agent 准备场景 → schema 检查 → 构建 → mesh 审计 → Cycles 渲染 → 人工审核 → 冻结视觉 baseline。Scanner 和 Azure 拟合有现成命令与配置样例，Agent 按场景加进 `workflow.json`，不需要改算法代码。该流程**不会**自动执行机器人抓取。

`runtime.json` 默认 CPU；确认 GPU 空闲后再设 `cuda_visible_devices`（例如 `"1"`）。它不安装环境、不启动相机、不驱动真机。运行中不要编辑配置。

### 3. 审核

先看状态列出的产物，再复制本次的 token：

```bash
"$PY" -m real2sim.cli case-status "$CASE"
"$PY" -m real2sim.cli case-review "$CASE" --stage review --token COPY_CURRENT_TOKEN \
  --decision approve --by YOUR_NAME --note "已检查相机、背景、尺度与材质"
"$PY" -m real2sim.cli case-run "$CASE"
```

不满意用 `--decision reject` 并写明问题，改完配置继续。旧 token 不会批准变化后的结果。冻结只表示视觉快照，**不能**据此宣称真实抓取通过。

### 4. 交给 Agent

可以直接复制这段：

> 使用 R2S2R 仓库处理 CASE。先读取 README、AGENT_HANDOFF.md、intake.json 和 TODO.md。根据原始扫描和视频准备独立资产、尺寸约束及相机拟合配置；把扫描处理、相机拟合和外观优化接入 workflow.json，声明所有输入。仅修改本场景配置和资产，不为常规新场景修改共享实现。运行 case-run，缺关键资料时集中列出最少补充项；产生可审阅结果后停在 review，不代替用户批准。保留失败、标定不确定性和物理验证边界。

阶段门槛与配置模板见 [docs/AUTOMATION.md](docs/AUTOMATION.md) 与 [docs/PIPELINE.md](docs/PIPELINE.md)；输入输出契约见 [docs/CONTRACTS.md](docs/CONTRACTS.md)。
