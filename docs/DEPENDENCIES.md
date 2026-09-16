# 依赖与部署

本仓库自带流程代码、schema 和配置样例，并通过子模块携带 Data-MechanicSim。本文说明要跑起来还需要哪些环境与外部资源，以及怎么自检。

## 需要哪些仓库

| 仓库 | 作用 | 获取方式 |
|---|---|---|
| **R2S2R**（本仓库） | 流程代码、schema、配置样例 | `git clone --recursive https://github.com/Saberlve/R2S2R.git`。**必须带 `--recursive`**：它带出子模块 Data-MechanicSim |
| **Data-MechanicSim** | Newton 工程、机器人资产、`newton_gen` 适配代码，以及触觉仿真库 **tacsim** | 本仓库的子模块，位于 `external/Data-MechanicSim`。上游为内网 Git：`https://git.weiyantech.cn/Worlddynamics/Data-MechanicSim.git`（需内网访问或 VPN 与账号，见下） |
| **tacsim**（Data-TacSim） | Photon 触觉仿真后端 | 已由 Data-MechanicSim 作为 `third_party/tacsim` 提供；**本仓库不再单独持有** |

`tacsim` 是**无 LICENSE 的内部代码**，不得对外再分发；其厂商运行时包 xense-sim4.5 同样专有。来源与许可记录见 [PROVENANCE.md](PROVENANCE.md)。

### Data-MechanicSim 子模块

它是本仓库的子模块，位于 `external/Data-MechanicSim`，`git clone --recursive` 会一并带出。它自己还有 `third_party/newton` 与 `third_party/tacsim` 两层子模块，所以递归是必需的 —— 少一层就只剩空目录。

```bash
git submodule update --init --recursive   # 克隆时漏了 --recursive 就补这条
```

子模块初始化好之后 **`R2S_NEWTON_PROJECT` 不必再设**：工具会回退到 `external/Data-MechanicSim`，`./r2s-server site` 显示当前解析到的值。只有在想把工程放到别处时才设它，设了就以你的值为准。

访问不了内网 Git 时子模块取不下来（`https://git.weiyantech.cn/...` 需内网或 VPN）—— 换网络，或向维护者索取一份可访问的镜像/打包，再把 `R2S_NEWTON_PROJECT` 指向那份 checkout。**不要再手工复制一份 DMS**：tacsim 会跟着出现第二份，而它由 `--python` 指定的解释器解析，两份会让"跑的到底是哪份"变得不可知。

克隆完自查：

```bash
git -C external/Data-MechanicSim/third_party/tacsim log --oneline -1   # 应 >= d5ce700
```

### tacsim 的版本下限

**必须 ≥ `d5ce700`（"Move xense sdk into repo"）。** 该提交把裁剪过的厂商运行时（约 20 MB）提交进 `third_party/xense_photon/`，使 tacsim 从纯 clone 即可用。更早的版本要求手工部署一套 1.7 GB 的厂商 dist，那份已随本仓库的去重一并删除。Data-MechanicSim 的 gitlink 已钉在 `d5ce700`，所以 `--recursive` 克隆拿到的就是对的；若你的 checkout 更旧，在那里 `git fetch origin && git merge --ff-only origin/master`。

## 触觉（Photon）的环境要求

Photon 跑在 `--python` 指定的既有解释器里，**不在本仓库的依赖列表里**。该解释器需要：

- **Python 3.10**（厂商 `.so` 只编译了 cp310）
- **CUDA**（`NewtonTactileSensor` 强制要求）
- **OpenGL 上下文** —— 无显示器主机用 `xvfb-run -a`
- 已安装的 **tacsim**（含其 `third_party/xense_photon/` 内的厂商 bundle）
- 下列公开 wheel，版本与厂商 dist 一致：

```text
cffi, pyudev
cryptography==45.0.5
PySide6==6.8.1.1      （连同 shiboken6）
qtpy==2.4.2
Cython==3.0.11
assimp-py==1.0.8
```

裁剪版 bundle **只带厂商编译产物，不带这些公开 wheel** —— 它们是"删掉的那 1.7 GB"的替代，必须由环境提供。缺任何一个都会在 `require_fem_sensor()` 处失败；注意 tacsim 自己的报错文案写的是"requires cffi + pyudev"，在 cffi/pyudev 都在时**具有误导性**。

Data-MechanicSim 已在它的 `pyproject.toml` 里用 `tactile` 依赖组（并挂进 `default-groups`）固定这些版本。tacsim 自己的 `benchmark` 组**不参与传递依赖**（其 `pyproject.toml` 明确写了 "Consumers of tacsim only get `[project.dependencies]`"），所以必须在消费方重复声明，否则 `uv sync` 会把它们清掉。

`uv sync` 在这个仓库里是安全的：`_editable_impl_tactile_benchmark.pth` 应指向 `external/Data-MechanicSim/third_party/tacsim`，也就是 `[tool.uv.sources]` 声明的那份，而它现在位于 `d5ce700`（含 bundle）。同步后用 `r2s tactile doctor` 确认解析路径即可。

> ⚠️ **不要把 `.venv` 提交进 Git。** 它已在 `.gitignore` 里，但 ignore 对**已跟踪**的文件无效。历史上有过一次教训：一个 worktree 把 `.venv` 作为**绝对路径符号链接**提交了，checkout 到另一个 worktree 后变成指向自身的自环，把真实的虚拟环境目录顶掉。若发现 `.venv` 出现在 `git ls-files` 里，先 `git rm --cached .venv` 再用 `uv sync` 重建。

### 自检

```bash
PY=<能被 tacsim 解析的解释器>
PYTHONPATH=src $PY -m real2sim.cli tactile doctor --python "$PY"
```

逐项打印 `tacsim_importable` / `tacsim_repo_root_found` / `xense_bundle_deployed` / `python_is_3.10` / `cffi_importable` / `pyudev_importable` / `cuda_available`，并报出**实际解析到的** tacsim 路径（`tacsim_root`）与 bundle 位置。关键项缺失时退出码 2。想要可跑的完整示例：

```bash
PYTHONPATH=src xvfb-run -a $PY -m real2sim.cli tactile photon-render \
  examples/tactile/photon_minimal.json --out /tmp/photon_demo --python "$PY"
```

输出目录必须尚不存在。

## 两个 Python 环境

两个环境都在 Data-MechanicSim 子模块内部，各由一份 `uv.lock` 固定，用 `uv sync` 建出即可：

| 栈 | 位置 | Python | 用途 |
|---|---|---|---|
| 数值、标定、Newton、接触、触觉 | `external/Data-MechanicSim/.venv/` | 3.10 | `real2sim.cli`、`r2s-server` 的多数命令、`photon-render` |
| Cycles 无界面渲染 | `external/Data-MechanicSim/render_cycles/.venv/` | 3.11 + `bpy` | `scene build/render`、`tools/server_scene.py` |

```bash
(cd external/Data-MechanicSim && uv sync)                 # 主环境
(cd external/Data-MechanicSim/render_cycles && uv sync)   # 渲染环境
```

`uv` 按各自的 `.python-version` 自行取得所需 Python 版本。主环境里的 Newton 不是从 PyPI 装的：`[tool.uv.sources]` 把 `newton` 指向 `third_party/newton`（editable），tacsim 同理指向 `third_party/tacsim`。渲染环境的 `bpy` 是官方 wheel，Cycles 内核与 CUDA 随包自带，不需要系统安装 Blender。

工具默认就用这两个路径，不必设 `R2S_MAIN_PYTHON` / `R2S_CYCLES_PYTHON`；只有把环境放在别处时才用这两个变量覆盖。zju 现有值见 `examples/site.zju.env`。

`bpy` 后台仍负责读取已验证的 `.blend` 并执行 Cycles —— "服务器流程"指无需桌面 Blender 操作，不是换渲染器。

## 环境证据

`environment/` 下是**已观察到的**版本记录，用于对照而不是保证复原：

- `main.json` / `main-observed.txt`、`cycles.json` / `cycles-observed.txt`：两个环境的版本快照
- `core-tested.txt`：验证过的核心依赖约束
- `newton-source-snapshot.json`：Data-MechanicSim 与其 Newton 源码的 HEAD、dirty 路径及源码哈希

```bash
$PY tools/environment_snapshot.py --check environment/newton-source-snapshot.json
```

只检查源码，不检查驱动、全部二进制或设备确定性；GPU 浮点和渲染不保证逐位一致。

## 外部资源（NAS）

渲染结果、录像、扫描、模型、数据集不进 Git，统一放在 NAS（例外的只有仓库级物体资产库 `assets/`，限制见 [RESOURCES.md](RESOURCES.md)）：

```text
/run/determined/NAS1/public/wangshuxun/r2s2r_resources
```

NAS 不会随 Git 下载，需要挂载和读权限。布局与校验方式见 [RESOURCES.md](RESOURCES.md)。

## 相关文档

- [SERVER_GUIDE.md](SERVER_GUIDE.md)：服务器使用手册、故障定位
- [PROVENANCE.md](PROVENANCE.md)：各来源与许可边界
- [RESOURCES.md](RESOURCES.md)：NAS 资源布局
- [ARCHITECTURE.md](ARCHITECTURE.md)：四域结构与触觉接入说明
