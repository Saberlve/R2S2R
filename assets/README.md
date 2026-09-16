# 物体资产库

放不同任务需要的物体资产。一个物体只描述一次，任务按 id 引用，不再各自内联一份几何与物理参数。

不要和 case 目录里的 `assets/` 混淆：那个是**某个场景**从原始资料分离出来的资产（由 `r2s case-init` 建立，见 [docs/RESOURCES.md](../docs/RESOURCES.md)）；这里是**跨场景复用**的物体库，路径一律写成 `assets/<object_id>/`。

## 布局

```text
assets/
├── README.md
└── <object_id>/
    ├── asset.json        # 物体规格，schema_version 1.0
    ├── body.glb          # 可选；纯 box 物体没有二进制
    ├── textures/         # 可选
    └── PROVENANCE.md     # 可选；测量、标定或扫描出处
```

`<object_id>` 与 scene 资产同一条 id 规则 `^[A-Za-z][A-Za-z0-9_.-]*$`，**目录名就是 id**，`asset.json` 里的 `id` 必须与目录名一致。多个任务要用同一个物体时，引这一份，不要复制目录。

## 规格

```json
{
  "schema_version": "1.0",
  "id": "TestBox",
  "role": "task",
  "geometry": {"kind": "box", "size_m": [0.12, 0.06, 0.04]},
  "physics": {"mass_kg": 0.35, "friction": 0.6},
  "visual": {"color_linear": [0.8, 0.2, 0.1], "roughness": 0.5},
  "collision": {"closed_required": true},
  "provenance": {"confidence": "model", "source": "examples/minimal/scene.json", "notes": ""}
}
```

| 字段 | 说明 |
|---|---|
| `id` | 必须等于目录名，`^[A-Za-z][A-Za-z0-9_.-]*$` |
| `role` | `table` / `robot` / `task` / `misc` / `background`，与 scene.json 同一套 |
| `geometry.kind` | `box` 必须给 `size_m`（米）；`mesh` 必须给 `path`（相对 `asset.json`，只接受 `.glb`/`.gltf`，且不能跑出本物体目录）。`mesh` 也可另给实测 `size_m` 作为 bbox 记录 |
| `physics` | `mass_kg` > 0、`friction` ≥ 0（库仑 μ，无量纲）；`inertia_diagonal_kg_m2` 可选，**缺省时以 xArm7 适配器的 box 公式为准**（[traj/adapters/xarm7/physics_model.py](../src/real2sim/traj/adapters/xarm7/physics_model.py)），不要在这里再写一份 |
| `visual` | 可选。`color_linear` 在 [0,1]，`roughness` 在 [0,1] |
| `collision.closed_required` | 可选。网格是否需要封闭（审计用） |
| `provenance` | `confidence` ∈ `measured` / `estimated` / `model`；`source` 非空，指向测量或来源文件；`notes` 记不确定性与使用边界 |

**缺省不等于默认值。** 没写的段落表示"尚未确定"，需要它的消费者必须拒绝，不能自己补一个数。估计值不要写成 `measured`。

## 二进制策略

本库是全仓库**唯一**允许提交 mesh 与纹理的地方，由 [tools/check_git_payload.py](../tools/check_git_payload.py) 与 [.gitignore](../.gitignore) 同时约束，两者必须保持一致：

- 仅 `assets/` 前缀下放行网格 `.glb` / `.gltf`（`.bin` 作为 glTF 边车）与纹理 `.png` / `.jpg`；
- 纹理**必须放在本物体的 `textures/` 子目录**，这样渲染结果不会以 `assets/<id>/render.png` 的形式混进来；
- 单文件不超过 **2 MB**；
- 全局 >5 MB blob 封禁不变，其他目录继续沿用原有资源二进制封禁清单。

`.gitignore` 的例外规则和 `tools/check_git_payload.py` 的判定必须逐条对应，改一处就要改另一处；`tests/test_assets.py` 会检查这条一致性。

扫描、渲染结果、录像、数据集仍然不进 Git。超限的物体放在 NAS 资源根，不要靠 LFS 绕过。

## 校验与浏览

改动之后跑一遍；`check` 失败退出码 2：

```bash
export PYTHONPATH="$PWD/src"
PY=external/Data-MechanicSim/.venv/bin/python

"$PY" -m real2sim.cli asset check          # 逐个校验并输出报告
"$PY" -m real2sim.cli asset list           # 列出 id / role / 几何类型
"$PY" -m real2sim.cli asset show TestBox   # 打印一个物体的规格

python3 tools/check_git_payload.py         # 提交前确认没有越界的二进制
```

## 现状：库已就位，引用尚未接通

本轮只建立库、校验与浏览。`scene.json` 与 `plan_trajectory` 的 task.json **还不能**按 id 引用这里的物体，字段集也等真实物体示例确认后再定稿。在那之前，把物体登记进来、把来源写清楚是有用的；但不要声称某个任务已经在用库里的物体。
