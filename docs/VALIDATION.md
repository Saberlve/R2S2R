# zju 部署与验证记录 · 2026-09-12

## 仓库与运行环境

- Git仓库：`/home/wangshuxun/VLA/data_sim/R2S2R`
- 本地源文件副本：`D:/Code/real2sim-pipeline`
- 主Python：`/home/wangshuxun/VLA/data_sim/Data-MechanicSim/.venv/bin/python`（3.10）
- Cycles Python：`/home/wangshuxun/VLA/data_sim/Data-MechanicSim/render_cycles/.venv/bin/python`（3.11、bpy4.5）
- 原始LeRobot导入使用服务器已有`python3`，其环境有pyarrow；主Newton环境没有pyarrow。
- 现有Data-MechanicSim HEAD：`40cef6f352b271646dbd4991356d8dd6c7bd6022`
- editable Newton HEAD：`6ca77978526193d8db46feb96c3836b03003fa99`

Data-MechanicSim存在原有未提交修改，所以仅checkout上述HEAD不够。`validation/2026-09-12/environment.json`记录实际工作树Python源码hash、dirty路径和包版本；`tools/environment_snapshot.py --check`检查源码漂移。没有改动或重装现有Newton依赖。运行时可选择空闲GPU，不能把本次GPU编号当成永远空闲。

## 本仓库实际重新执行

|检查|结果|证据（仓库相对路径）|
|---|---|---|
|坐标、profile、PnP、FK语义、桥接、mask、防覆盖、视频时间检查|12个测试通过|`validation/2026-09-12/pytest.txt`|
|最小配置DAG：validate → build → render|通过；独立闭合box、Apply Scale、160×120 RGB|`runs/smoke4/pipeline_receipts.json`|
|Blender相机投影对照K|3个测试点最大约0.00000384像素|`runs/projection_check.json`|
|LeRobot v3全量导入与关节→TCP|38段、16133帧；原始关节/action/gripper/time逐值不变；source data Parquet哈希未变化|`runs/conversion_check.json`|
|转换与旧已验证结果交叉检查|最大位置差3.17e-8 m，旋转差1.65e-7 rad|同上；不是独立控制器或实机精度测试|
|新适配器Newton FK|episode0的489帧：最大0.000765 mm、0.0000469°|`runs/reference_xarm7/reports/newton_fk.json`|
|新适配器EEF动态跟踪|120帧无物体：P95 1.18451 mm / 0.175233°；max 1.31026 mm / 0.212465°|`runs/reference_xarm7/results/eef_mujoco_fast_free_adapter_check/report.json`|
|完整Cycles有界外观优化|8次候选，fit loss 0.08326→0.002772；holdout 0.07920→0.002624|`runs/appearance_check/fit/history.json`、`acceptance.json`|
|保存Newton实际body状态→JSONL→固定双相机+body相机→视频|三帧、每视角160×120、拼接480×120、7.5FPS；显式stride4|`runs/state_video_check2/three_views.mp4`及同名.video.json|

外观测试是两台合成相机、已知光照参考，测试优化实现能否回归已知渲染；不表示真实图像已达到该loss。视频测试将一个简单标记box绑定到真实Newton回放的link7，腕部相机采用测试安装变换；它验证接口和同步，不是完整机器人视觉资产或真实手眼标定的验收。

`runs/`是Git外的本机验证证据；小型数值报告复制到`validation/2026-09-12/`随Git保存。失败尝试保留在runs中供诊断，不纳入通过结果。原baseline、原始资料与原EEF工程未修改。

## 尚未在本轮重新验证

- Blender MCP网络入口、真实相机采集（默认dry-run）、LPIPS权重分支。
- 直接带Brown畸变渲染：现有Cycles环境缺cv2；用独立环境补齐后需专门验证。无畸变合成profile通过，不代表可以清零真实D。
- GLB复杂资产的自动语义正确性、真实新场景外观、资产自碰撞。
- 真实长条可靠接触复现。本轮没有重新训练摩擦/夹爪参数；旧项目的成功与失败详见LESSONS.md。
- 本次源树整理后的hydroelastic与生成抓取全量重跑；移植的历史实现不冒充本次回归证据。

## 快速复跑

```bash
cd /home/wangshuxun/VLA/data_sim/R2S2R
export PYTHONPATH="$PWD/src"
PY=/home/wangshuxun/VLA/data_sim/Data-MechanicSim/.venv/bin/python
$PY tools/environment_snapshot.py --check validation/2026-09-12/environment.json
$PY -m pytest -q
# site.local.json已填写本服务器的Cycles路径；每次使用新的输出目录。
CUDA_VISIBLE_DEVICES=1 $PY -m real2sim.cli run site.local.json --out runs/my_first_scene
```

运行前自行检查GPU是否空闲；不改动他人任务。完整新场景按NEW_SCENE_CHECKLIST.md准备输入，不能把合成fixture相机参数当成真实标定。


后续补充：扫描背景与Azure七参数拟合已纳入，并以原资料复跑；当前总计15项测试。详见[SCANNER_AZURE_VALIDATION.md](SCANNER_AZURE_VALIDATION.md)，勿将下方/上方早期12项记录当作最新覆盖范围。
