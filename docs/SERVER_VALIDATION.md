# 服务器入口实际验证：2026-09-14

代码版本见本次服务器Git提交。结构化记录位于 `validation/server_20260914/`，完整日志和图像位于 `real2sim/server_runs/`。

| 检查 | 实际结果 |
|---|---|
| 本地上传完整性 | 3513/3513 文件大小与SHA-256一致；702补齐、2720已有相同、3冲突保留 |
| Python单元/契约测试 | 26 passed，包含模板自包含导出、防覆盖、缺失标定拒绝 |
| 源码依赖快照 | 927个源码文件全部匹配，无新增/丢失/变更 |
| doctor | 主环境、bpy环境、ffmpeg和案例路径通过 |
| inspect | 真实参考场景对象、灯光、材质清单成功输出 |
| smoke | CPU最小场景校验、构建、mesh审计、Cycles渲染通过 |
| edit | 真实背景材质贴图增益与灯光功率修改到新模板；相机/机器人变换保护生效 |
| render | 修改后模板三视角成功渲染 |
| simulate | GPU1，mujoco_fast，90帧Newton状态，stride30输出3个三视角帧并编码视频 |

仿真入口检查的视频为1920×480、3帧、1fps。**这只是3秒短程软件检查，尚未到闭合/抬升阶段，因此不能把本轮结果称为重新完成抓取验收。** 完整481帧命令见使用手册；已有完整任务与真机回放证据保存在MeasuredBarGraspV1，不因迁移而改变结论。

所有验证均通过服务器命令执行，未连接真实机器人或相机，未调用本地Blender/MCP。

对应运行目录：

- `setup_doctor_20260914`
- `setup_smoke_20260914`
- `setup_inventory_20260914`
- `setup_edit_20260914` / `setup_edit_final_20260914`
- `setup_edit_render_20260914`
- `setup_simulation_20260914`

`receipt.json`记录各阶段参数、运行路径及状态，详细错误或警告在worker/render/simulation日志中。新机器的安装步骤尚未在空白系统验证；应先跑自己的doctor/smoke，再做完整物理验收。
