# 新场景自动化契约

## 分工与阶段边界

用户准备 README 中的原始资料和尺寸。Agent 做语义判断、扫描裁切与注册、对应点标注、初始配置及缺陷分析。确定性命令完成检查、拟合、构建、渲染、评分和导出。人类只在关键资料缺失和可审阅结果生成时介入。

当前一键入口 `case-run` 是可恢复的离线视觉编排器，不是自动从任意视频分割并恢复真实动力学的模型。默认两个停点：资料/场景准备不足，以及最终视觉 review。资料齐全后，Agent 一次配置各阶段，多次执行不需要反复编辑共享代码。

## 输入和输出

- `intake.json`：单位米；`table_size_m` 为长宽高；`cameras` 含唯一语义 ID、相对于 case 的原图路径、native_wh、pixel_ops（未知用 null）、内参文件可为空；`scanner_exports` 是完整导出包路径；`anchors` 建议每项写 `from/to/distance_m/evidence/uncertainty_m`。
- 原始文件存在性、输入尺寸合法性和 SHA256 会检查；不会仅凭文件扩展名推断内容正确。Agent 仍须解码视频/图片、检查 ZIP 完整性和测距起止点。
- `config/scene.json` 必须通过已有 scene schema。`config/evidence.md` 记录扫描来源及注册变换、相机拟合/独立验证、排除项、尺度与照明证据。文件存在门槛不是内容被自动验证的证明。
- `workflow.json`：有序 DAG；依赖必须出现在前面。每阶段 id 唯一，由字母、数字、下划线或连字符组成。
- `runs/status.json`：当前结果、状态、输入指纹、输出哈希和 attempt 路径。`TODO.md`：当前需要人或 Agent 处理的事项。
- 状态为 `needs_user/needs_agent/needs_review/failed/complete`。每个命令 attempt 下有 `command.json` 和 `stage.log`。

## 阶段类型

| kind | 输入 | 输出 |
|---|---|---|
| intake | intake.json 和其中原始文件 | 缺失项、警告、原始文件哈希 |
| files | inputs 中的文件/目录 | 存在检查及内容哈希；缺失暂停给 Agent |
| command | allowlist CLI argv、inputs、可选 templates | 独立 attempt 中的声明 outputs 与日志 |
| review | requires 依赖产物、instruction | 与依赖指纹绑定的 approve/reject 记录 |

允许命令（分组形式，见 docs/ARCHITECTURE.md）：`scene validate/build/audit/render/scan-inspect/scan-register/scan-bake/fit-appearance/score/video`、`align calibrate/fit-camera`、`traj convert`、顶层 `freeze`。旧扁平写法（如 `build`、`calibrate`）在白名单检查前自动归一化为分组形式，既有 workflow.json 无需修改。无 shell 命令和硬件调用。Newton/xarm7 回放和 LeRobot 导入继续使用已有独立接口；当前 case-run 不自动调用会写入 case 的物理适配器。物理流程说明见 ROBOT.md，不能将视觉 complete 解释为整个物理闭环通过。

每个 command 必须声明 `outputs`；`scene validate` 可以为空。`--out` 必须指向 `{out}` 下尚不存在的子路径，声明的输出也不得逃出 attempt。外部资产、mask、视频、JSON 引用的其他文件必须显式加入该阶段 `inputs`，或加入上游 files 阶段并建立依赖；**运行器不会递归猜测任意配置中的路径**。目录 inputs 会递归做哈希，拒绝符号链接。不要把 runs 或整个 case 作为输入，避免把生成物纳入自身指纹。

## 占位符与 JSON 模板

支持 `{case}`、`{out}`、`{runtime.cycles_python}`、`{stage.STAGE_ID}`。后者是已执行命令的 attempt 根目录，使用它的阶段必须直接或间接依赖该阶段。argv 是字符串数组，无 shell 展开。

复杂拟合配置可使用 templates：

```json
{
  "id": "camera_fit",
  "kind": "command",
  "requires": ["scene_ready"],
  "inputs": ["{case}/config/camera_points.json"],
  "templates": {"fit": "{case}/config/camera_fit.template.json"},
  "argv": ["fit-camera", "{template.fit}", "--out", "{out}/fit"],
  "outputs": ["fit"]
}
```

模板结构仍遵循相应命令的契约；仅把文件路径改成 `"{case}/config/scene.json"`、`"{case}/config/camera_points.json"` 等绝对路径占位符。运行器将展开后的 JSON 放入 attempt/configs，原模板保持不变。模板路径自动参与输入哈希；模板引用的其他文件仍须声明。模板不可互相引用。

## 把两条关键路径接入新场景

1. Scanner：保留原包 → Agent 安全解包、识别语义区域 → `scene scan-inspect` → 准备注册点/尺度约束 → `scene scan-register` → `scene scan-bake` → 独立 GLB/纹理及审计。详细字段与案例见 SCANNER_BACKGROUND.md 和 examples/lab_reference/scanner。最终 GLB 路径及变换写入 scene；注册质量和裁切接缝需要复核。
2. Azure/未知 K：先固定尺寸、桌子、机器人基座及可信 B 相机 → 准备多处 3D–2D 对应点、线和深度约束 → `align fit-camera` → 独立边缘/另一视角检查 → 接受后下游使用候选 scene。见 AZURE_CAMERA_ALIGNMENT.md 和 examples/lab_reference/azure；不能只用训练重投影误差验收。
3. 外观：冻结几何与相机 → `scene fit-appearance` 完整 Cycles 候选 → 检查 fit/holdout loss、有效 mask、前景曝光 → review → 采用明确的参数文件，再 render。拟合器不会自动把候选结果写回 baseline。
4. motion：已有 Newton 状态通过 export_saved_states.py 转标准 JSONL，再配置 render 的 `--states` 与 video 阶段。机器人碰撞、TCP、接触验收仍是独立要求。

上游输出路径可以通过 stage 占位符传给后续命令或模板；不同命令的具体文件名以该命令报告为准。需要语义判断时插入 files 阶段并写清 instruction，Agent 补配置后继续。不要给每个数值优化候选插人工审批。

## 恢复与缓存

指纹覆盖阶段配置、runtime.json、包内代码/schema、显式输入与依赖的指纹/输出哈希/attempt。缓存复用前重新验证声明输出哈希。修改源资产、配置或输出会重新执行依赖链并撤销旧审核的适用性；无关分支可复用。外部 Python/Newton 二进制和系统库升级不是通过路径就能发现的，环境升级后应更新 runtime.json 的自定义 environment_revision 字段，并用 environment_snapshot.py 留存实际版本。

新尝试始终使用新目录，失败日志保留。未修改的失败阶段需要显式 --retry-failed。并发进程被 runs/.lock 阻止；异常断电留下锁时先确认原进程已退出，再由操作者移走锁文件，程序不会杀进程或自动猜测过期锁。

审核 token 绑定当次产物，reviewer 为填写的记录字段而非认证身份。Agent 不得冒名批准；自动化测试只能写 integration_test 并明确合成测试。run 期间不修改配置或审核文件。完整输入复制/环境容器化、远程队列调度及自动原始视频语义建模不在本版范围。
