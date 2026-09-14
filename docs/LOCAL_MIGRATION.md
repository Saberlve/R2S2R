# 本地项目迁移记录：2026-09-14

本机现存、已明确属于本项目的文件已上传到 zju；之后以服务器仓库和案例目录为工作副本。上传没有删除本地文件。

## 范围

- `D:/Code/LabSceneNewtonV1`：场景、代码、原始资料、校准、抓取、录像对比。
- `D:/Code/real2sim-pipeline`：本地可复用仓库副本。
- `C:/Users/WSX/Documents/Codex/2026-09-09/blender-3d-1-cad-2-mesh`：原任务工作目录及脚本。
- 本任务明确提供的微信 MOV/HEIC/JPG/PNG、最新扫描ZIP、腕部相机GLB与桌面 `metal_plate_result` 资产目录。

完整上传包：`/home/wangshuxun/VLA/data_sim/real2sim/local_imports/20260914/local-project.tar.gz`。

解包快照：同目录 `snapshot/`，保留四个分组：`LabSceneNewtonV1/`、`real2sim-pipeline-local/`、`desktop-workspace/`、`supplied-originals/`。按 `manifest.json` 可从每个服务器文件反查本地原路径。

## 校验

3513 个文件，1,737,878,387 bytes原始内容。压缩包SHA-256：

```
38393858f5315687c967c90e35b09a214df667e8dc63cf7e73acd3ff28c08bad
```

上传后逐文件核对大小和SHA-256，3513/3513通过。报告见同目录 `verification.json`。

补齐当前案例目录时：702个文件新增，2720个文件与服务器内容一致，3处不同版本均保留；88个本地仓库文件保存在迁移快照中，服务器Git仓库继续作为代码主版本。

版本差异是 `current_scene.json`、`MeasuredBarGraspV1/run_offline.sh` 和项目根README。导入没有用旧本地文件覆盖服务器版本；旧本地副本可以从快照读取。之后的服务器入口/文档更新属于正常开发，不改变上传快照。

## 原始资料归位

`real2sim/source_inputs/supplied-originals/` 保留存在的用户原始附件；`real2sim/SourceData/` 保留扫描处理与早期标定资料。`desktop-workspace/` 补入 `real2sim/source_inputs/desktop-workspace/`。

以下6个历史路径在本次迁移前已不存在，未虚构为已上传：Downloads中的 `green_background.zip`、`desk_and_door.zip`，以及4张早期Temp剪贴板截图。它们的具体路径在manifest中。已有扫描派生资产、参考图和处理记录已上传；这不等于缺失的ZIP字节级原件已经恢复。其余本次实际发现的文件均已上传。

## 复查命令

迁移工具需要Python3.12+：

```bash
python3 tools/import_local_snapshot.py   --bundle /home/wangshuxun/VLA/data_sim/real2sim/local_imports/20260914   --cases-root /home/wangshuxun/VLA/data_sim/real2sim
```

它先验证压缩包和每个文件，再只补齐缺失文件；已有不同内容不会覆盖。重新运行的“新增/相同”计数会变化，因此本次3513文件的初始统计保存在仓库验证摘要中。大型数据保持Git外。
