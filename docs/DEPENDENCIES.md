# 依赖与部署

## zju 已验证环境

不需要在本机安装 GUI、MCP 或显卡环境。zju 的 `site.local.json` 已配置，直接运行 `./r2s-server doctor`。本次没有安装或替换任何现有依赖。

| 栈 | Python | 已观察到的关键依赖 |
|---|---|---|
| 数值、标定、Newton、接触 | 3.10.12 | Newton 1.5.0、Warp 1.16.0、MuJoCo 3.11.0、NumPy1.26.4、SciPy1.15.3 |
| Cycles 无界面渲染 | 3.11.14 | bpy4.5.12 LTS、NumPy1.26.4、Pillow12.3.0 |
| 视频 | 独立可执行文件 | 现有 ffmpeg，路径写入 site.local.json |

完整已观察版本：`environment/main.json`、`environment/cycles.json` 及 `*-observed.txt`。这些文件是环境证据，**不保证直接 pip install 就能复原包含本地修改的机器人/引擎代码**。

`environment/newton-source-snapshot.json` 记录 Data-MechanicSim 与其 Newton 源码的 HEAD、dirty路径及927个源码哈希。已有 checkout 可能有本地改动；单纯 checkout HEAD 不等于当前验证环境。

```bash
/path/to/main/python tools/environment_snapshot.py --check environment/newton-source-snapshot.json
```

此命令仅检查源码，不检查驱动、全部二进制或设备确定性。GPU 浮点和渲染不保证逐位一致。

## 在新机器安装通用视觉 pipeline

以下是分离环境的部署步骤；本次实际验证在 zju 已有环境完成，新机器仍须跑 doctor / smoke。

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -c environment/core-tested.txt -e '.[calibration,metrics,test]'
python3.11 -m venv .venv-cycles
.venv-cycles/bin/python -m pip install --upgrade pip
.venv-cycles/bin/python -m pip install 'bpy==4.5.12' 'numpy==1.26.4' 'Pillow==12.3.0'
cp examples/site.example.json site.local.json
# 填写所有路径；安装系统ffmpeg或指定已有可执行文件
./r2s-server smoke --samples 4
```

核心最小依赖来自 `pyproject.toml`；标定需要 OpenCV，SSIM需要scikit-image，LeRobot Parquet需要pyarrow。LPIPS为可选评估，需要torch/lpips及相应权重，未配置时不得把缺失分数写成0。HEIC原件已归档；解码需额外HEIF支持或先导出无损PNG，不是普通场景渲染依赖。

`bpy` 后台仍负责读取已验证的 `.blend` 场景和执行 Cycles。这里的“服务器流程”指无需桌面 Blender 操作，并非换成另一个渲染器；如果完全移除 bpy，须另外迁移材质、灯光、相机与纹理并重新验证画面对齐。

## 机器人仿真依赖

完整 xArm7 任务还需要：

1. 与快照对应的 `Data-MechanicSim/newton_gen` 适配代码及其Newton工作树。
2. URDF引用的机器人/夹爪网格与已经验证的碰撞缓存。
3. `MeasuredBarGraspV1` 的标定、配置、轨迹与输入数据。
4. 兼容当前 Warp/Newton/MuJoCo GPU路径的 NVIDIA 驱动与CUDA运行环境；使用空闲GPU。

这些依赖通过 `site.local.json` 明确传入，没有内嵌到通用pip包。不要在已有工作站运行 `pip install newton` 来替换本地版本。跨服务器复制时需重写 URDF 的绝对mesh路径、复制资产并重新生成/验证缓存哈希，而不是把缓存失配检查删掉。

仅使用通用视觉构建/渲染的研究人员可以先跑 smoke，不必先准备机器人依赖。`doctor` 的完整检查面向本实验室配置，因此缺少Newton案例时会指出尚未安装的模块/路径。

## 配置和环境变量

`site.local.json` 不进Git：`main_python`、`cycles_python`、`ffmpeg`、`newton_project`、`measured_case`、`reference_template`、`runs_root`均填写绝对路径。

- `R2S_SITE_CONFIG`：替代默认site文件。
- `R2S_LAUNCH_PYTHON`：运行标准库launcher的Python，默认python3。
- `--gpu N`：显式选择CUDA可见设备；render/smoke不指定时CPU，simulate要求显式指定。
- 主CLI可直接使用 `PYTHONPATH=src`，无需修改现有Python环境。

迁移归档工具 `tools/import_local_snapshot.py` 使用安全tar过滤及流式哈希，运行它需要Python3.12+；常规pipeline最低版本仍为Python3.10。
