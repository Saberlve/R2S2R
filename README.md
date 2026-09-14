# R2S2R

Real2Sim 流程仓库。用途是从实拍资料（照片、环拍视频、3D 扫描）搭出尺度正确、相机约定一致、能接物理仿真的场景，并把可复用的流程代码和单个实验室的调试历史分开管理。它不是"给段视频自动生成数字孪生"的工具——资料准备、尺寸确认、相机拟合审核这些环节仍然要人参与。

仓库里有什么：场景 JSON schema 和校验、GLB/规则几何构建与审计、服务器无界面 Cycles 渲染入口、固定内参 PnP、区域 MAE/SSIM/可选 LPIPS、有界外观拟合、冻结快照、LeRobot v3 导入、Newton 状态桥，以及 xArm7/G2/TCP172 物理适配器。本场景用得最多的两条流程单独写了文档：[3D Scanner 扫描背景](docs/SCANNER_BACKGROUND.md) 和 [Azure 焦距+位姿联合拟合](docs/AZURE_CAMERA_ALIGNMENT.md)。

代码仓库：<https://github.com/Saberlve/R2S2R>

渲染结果、录像、扫描、模型、数据集不进 Git，统一放在 NAS：

```text
/run/determined/NAS1/public/wangshuxun/r2s2r_resources
```

NAS 不会随 Git 下载，需要有对应挂载和读权限。资源布局和校验方式见 [docs/RESOURCES.md](docs/RESOURCES.md)。

## 在 zju 上跑

当前工作副本以 zju 为准。场景 JSON 和 Python 直接在服务器上改，用 `./r2s-server` 跑 Newton 和无界面 Cycles，不需要本地 Blender 或 MCP 插件（Cycles 后端本身还是 bpy，只是不用开桌面）。

```bash
git clone https://github.com/Saberlve/R2S2R.git
cd R2S2R
cp examples/site.zju.json site.local.json
./r2s-server doctor                  # 检查已有环境，不安装或替换依赖
./r2s-server smoke --samples 4       # CPU 最小场景：构建、审计、渲染
./r2s-server render --gpu 1 --samples 16  # 实验室当前模板，三相机
# 完整离线抓取任务及三视角视频，GPU 选空闲的
./r2s-server simulate --gpu 1 --frames 481 --stride 15 --samples 16
```

每条命令输出唯一的 `OUTPUT` 目录和 `receipt.json`，默认写到 `../real2sim/server_runs/`。环境路径由 Git 外的 `site.local.json` 指定；换机器就复制 `examples/site.example.json` 填自己的路径。

- [服务器使用手册](docs/SERVER_GUIDE.md)：场景编辑、渲染、仿真、输入输出、日志与故障定位
- [依赖与部署](docs/DEPENDENCIES.md)：两个 Python 环境、精确版本、源码依赖及新机器安装步骤
- [本地迁移清单](docs/LOCAL_MIGRATION.md)：文件位置、完整性验证、历史原件缺失和冲突处理
- [运行验证](docs/SERVER_VALIDATION.md)：本轮实际执行的命令与结果

当前参考场景是 `WristCameraAlignmentV1/runtime/`：合并了背景扫描和机器人调色，腕部外参用最新录像拟合。腕部图像是原生 640×480、raw 方向；另一路录像重投影 RMSE 约 10.7 px，所以标为估计值。原始视觉 baseline 留在 `baseline/`。

新测量长条生成轨迹已有仿真结果，真机回放成功抓起，原始录像和日志可供复核。但要说明：这不等于早期 38 段录制轨迹都完成了接触动力学复现，也不等于本轮短程 smoke test 再次验证了完整抓取。

## 新场景要准备什么

最小资料包：每台固定相机的原图、房间环拍视频、3D Scanner 完整导出、实验桌尺寸。为了少来回补拍，最好再给几条距离锚点和相机实际采集设置。机器人运动与接触仿真需要额外数据，仅凭这个资料包不能验收。

| 资料 | 怎么提供 | 用途与缺失影响 |
|---|---|---|
| 每台固定相机原始图片 | 原始 RGB 文件，不截图、不额外裁剪；尽量与目标场景状态及照明一致 | 视角、遮挡、光照和颜色拟合的目标；手机照片不能替代 |
| 房间环拍视频 | 缓慢走动，覆盖桌子、墙角、门窗与邻桌，多视角有重叠 | 建立空间关系、判断遮挡与语义分离 |
| 3D Scanner 完整导出 | 保留 ZIP 中的 OBJ/MTL 或 GLB、纹理、UV、单位/导出元数据；不要只给预览截图 | 扫描背景的几何与实拍纹理；仍需裁切、清理、注册和闭合检查 |
| 主桌长×宽×高 | 单位明确，建议米；高度说明是否到桌面上表面 | 尺度与水平桌面约束 |
| 几条距离锚点 | 桌边到墙、邻桌前沿、门宽高等；照片标明测量起止点 | 避免用错误背景或相机位姿补偿空间误差；没有时记录不确定性 |
| 相机采集设置 | 型号、实际分辨率、裁剪/缩放/翻转；能获取则附 K、畸变和 SDK 元数据 | 无可靠内参时走焦距+位姿拟合，输出标为 `image_fitted`，不冒充物理标定 |
| 排除清单 | 明确不建模的椅子、线缆、设备等 | 统一建模范围和图像评分 mask |

额外照片优先补拍门窗转角、扫描缺损和材质细节；测距截图只用于几何判断，不直接烘焙成纹理。不要求一开始就人工提供相机外参，Agent 可以准备对应点和搜索配置，但需要独立边缘或另一视角交叉检查。

如果还要做关节/EEF 回放与接触抓取，另外需要：机器人模型及标定、TCP/夹爪配置、带时间戳的关节/EEF 数据和相机视频关联、物体尺寸质量和初始位姿。初始位姿可以先从图像估计，但必须标注估计范围。详见 [docs/ROBOT.md](docs/ROBOT.md)。

## 新场景的初始化和运行

```bash
cd /home/wangshuxun/VLA/data_sim/R2S2R
export PYTHONPATH="$PWD/src"
PY=/home/wangshuxun/VLA/data_sim/Data-MechanicSim/.venv/bin/python
CYCLES=/home/wangshuxun/VLA/data_sim/Data-MechanicSim/render_cycles/.venv/bin/python
CASE=/home/wangshuxun/VLA/data_sim/real2sim/MyNewScene

# 只初始化一次；目录必须尚不存在
$PY -m real2sim.cli case-init "$CASE" --cycles-python "$CYCLES"
# 把原始资料放入 raw/，填 intake.json；之后始终运行这条命令
$PY -m real2sim.cli case-run "$CASE"
```

`runtime.json` 默认 CPU；确认 GPU 空闲后再设置 `cuda_visible_devices`（例如 `"1"`）。它不会安装环境、启动相机或驱动真机。`case-run` 完成返回 0，暂停或阶段失败返回 2；读 `TODO.md` 和 `runs/status.json` 区分原因。

新场景目录长这样：

```text
MyNewScene/
  intake.json          # 你提供的文件路径、尺寸、相机设置、排除项
  runtime.json         # 已有运行环境、GPU 选择
  workflow.json        # Agent 配置阶段、依赖、参数及输入输出
  AGENT_HANDOFF.md      # Agent 工作约定
  raw/                 # 原始资料，保持不变
  assets/              # 分离后的资产、纹理
  config/              # scene.json、拟合配置、对应点、evidence.md
  runs/                # 每阶段独立 attempt、日志、哈希、状态及审核记录
  TODO.md              # 当前下一步；不需要翻整个聊天历史
```

给 Agent 的交接提示可以直接复制：

> 使用 real2sim-pipeline 仓库处理 CASE。先读取 README、AGENT_HANDOFF.md、intake.json 和 TODO.md。根据原始扫描和视频准备独立资产、尺寸约束及相机拟合配置；把扫描处理、相机拟合和外观优化接入 workflow.json，声明所有输入。仅修改本场景配置和资产，不为常规新场景修改共享实现。运行 case-run，缺关键资料时集中列出最少补充项；产生可审阅结果后停在 review，不代替用户批准。保留失败、标定不确定性和物理验证边界。

默认流程是资料检查 → Agent 准备场景 → schema 检查 → 构建 → mesh 审计 → Cycles 渲染 → 人工审核 → 冻结视觉 baseline。Scanner 和 Azure 拟合有现成命令与配置样例，Agent 按场景加入阶段，不需要改算法代码。该默认流程不会自动执行机器人抓取。

运行器会复用未变化的阶段；输入、代码、运行配置或依赖产物变化会使相关阶段重跑。每次重跑新建 attempt，不覆盖旧结果。失败不会无休止重试；修正配置后继续，或检查日志后用 `--retry-failed`。不要在运行中编辑配置。

审核时先看状态列出的产物，再复制本次的 token：

```bash
$PY -m real2sim.cli case-status "$CASE"
$PY -m real2sim.cli case-review "$CASE" --stage review --token COPY_CURRENT_TOKEN \
  --decision approve --by YOUR_NAME --note "已检查相机、背景、尺度与材质"
$PY -m real2sim.cli case-run "$CASE"
```

不满意用 `--decision reject` 并写明问题，Agent 修改场景配置后继续；旧 token 不会批准变化后的结果。这是可追溯的审核约定，不是用户身份认证。冻结只表示视觉快照，不能据此宣称真实抓取通过。

配置阶段、模板和自动化边界详见 [docs/AUTOMATION.md](docs/AUTOMATION.md)。

## 从哪里开始

1. 读 [docs/PIPELINE.md](docs/PIPELINE.md)，了解流程与阶段门槛。
2. 按 [docs/CONTRACTS.md](docs/CONTRACTS.md) 准备新场景资料。
3. 跑通下面的最小例子，再替换 `examples/minimal/scene.json` 中的资产、米制尺寸、相机和灯光。
4. 外观优化见 [docs/APPEARANCE.md](docs/APPEARANCE.md)，实际运行证据见 [docs/VALIDATION.md](docs/VALIDATION.md)。
5. 机器人部分另读 [docs/ROBOT.md](docs/ROBOT.md)。

## 环境

Python 3.10+，核心依赖见 `pyproject.toml`。推荐在独立环境安装 `pip install -e '.[calibration,metrics,test]'`。已有 Newton 的工作站不要重装 Newton，也不要运行 `pip install newton`。zju 验证时直接复用现有环境，通过 `PYTHONPATH=src` 运行；Cycles 用另一个已有 bpy 的环境。

```bash
cd /path/to/real2sim-pipeline
export PYTHONPATH="$PWD/src"
PY=/path/to/main/python
CYCLES=/path/to/python-with-bpy
$PY -m pytest -q
$PY -m real2sim.cli validate examples/minimal/scene.json
$PY -m real2sim.cli build --scene examples/minimal/scene.json --out runs/example/scene.blend --python "$CYCLES"
$PY -m real2sim.cli render --scene examples/minimal/scene.json --blend runs/example/scene.blend --out runs/example/render --python "$CYCLES" --samples 32
```

也可以把 `examples/minimal/pipeline.json` 中的 `cycles_python` 改成自己的环境路径，然后：

```bash
$PY -m real2sim.cli run examples/minimal/pipeline.json --out runs/example_pipeline
```

输出目录必须新建。每阶段命令、输入文件哈希、退出码和输出位置保存在 `pipeline_receipts.json`；依赖阶段失败即停止。

当前默认流程仅在服务器运行。历史 MCP 接口保留用于兼容旧记录，上手不需要安装插件或启动桌面软件。

## 目录

- `src/real2sim/`：场景、相机、轨迹、指标与 CLI
- `src/real2sim/schemas/`：机器可读输入约束
- `src/real2sim/workers/`：独立 bpy worker
- `src/real2sim/adapters/xarm7/`：来自已运行项目的物理适配器；通过环境注入案例/工程路径
- `tools/`：只读数据导入、控制器快照转换、需明确启用才拍摄的相机采集工具
- `examples/`：最小场景及任务计划，不包含真实图像或大模型
- `tests/`：坐标、相机 profile、数据语义、防覆盖与桥接检查
- `docs/`：实际流程、契约、经验边界及验收说明

原始照片、扫描、数据集、模型、运行缓存放在 Git 外；本仓库只管理代码、schema、配置样例和小型验证记录。参考场景的绝对路径只写在部署说明或 `site.local.json`，不嵌入通用模块。

## 当前边界

几何语义拆分、遮挡补全、独立实测点选取和资产质量审核仍需人工或建模 Agent；外观拟合不允许顺便移动相机或物体。自动 PnP 的输出默认 `image_fitted`，不会因为训练点重投影误差小就标为独立标定成功。

通用渲染器目前正式输出 RGB 与有效像素 mask，不输出深度，因此不存在把欧氏射线距离当成光轴 Z 深度的默认行为。需要深度或实例 ID 时要增加对应后端并通过专门测试再启用。

物理适配器目前只验证了 xArm7 + G2 + 172mm TCP；其他机器人/夹爪/TCP 需要新适配器。早期数据集逐段接触复现仍有局限；最新测量长条任务及真机日志另见服务器使用手册。历史分析见 [docs/LESSONS.md](docs/LESSONS.md)。生成抓取成功不能替代真实片段的动力学验收。

## 从 Newton 状态输出视频

```bash
$PY tools/export_saved_states.py --states /path/to/states.npz --bindings /path/to/bindings.json --out runs/motion/states.jsonl
$PY -m real2sim.cli render --scene /path/to/scene.json --blend /path/to/scene.blend --states runs/motion/states.jsonl --out runs/motion/render --python "$CYCLES"
$PY -m real2sim.cli video --manifest runs/motion/render/render_manifest.json --cameras CameraB Azure Wrist --out runs/motion/three_views.mp4
```

必须先按实际 link-to-mesh 关系准备 binding。用 `--stride` 抽取状态会改变渲染 FPS，不改变物理时间；encoder 依据时间戳编码。视频工具需要环境中现有 ffmpeg。

Blender worker 运行环境须有 bpy、numpy、Pillow；若直接渲染 OpenCV Brown 畸变图，还需该环境安装 opencv-python-headless。zju 现有 Cycles 环境未包含 cv2，本次验证使用无畸变 profile；正式使用畸变分支前应在独立环境补齐依赖并验证，不能将畸变参数直接清零冒充对齐。
