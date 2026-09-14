# 扫描背景与Azure补充验证 · 2026-09-12

本次补充专门恢复第一版pipeline遗漏的两个关键环节。原baseline和真实设备均未改动；以下是新仓库中重新执行的结果。

## 原始扫描资产

实际读取用户原扫描的textured_output.obj和8K JPG：door源248119三角面，cloth源288285三角面。source SHA、配置SHA、覆盖mask、GLB及重导入检查见`validation/scanner_azure/`，大文件在`runs/scanner_reference/`，例子依赖的四个原始OBJ/JPG在Git外`data/lab_scanner/`。

|结果|门板|绿布|
|---|---|---|
|选中源三角面|17288|175042|
|直接观测纹理覆盖|98.4393%|91.4933%|
|有限距离填补|1.5607%|7.5784%|
|剩余未知纹理|0%|0.9283%|
|规则化几何|0.89×2.08m，厚35mm|151×181前表面网格，起伏±25mm，厚1.5mm|
|闭合网格顶点|8|54662|
|非流形边|0|0|
|GLB mesh / scene数|1 / 1|1 / 1|

门板重新运行了历史把手区域mask修补和低频曝光校正；mask作为案例文件保存。绿布的裁剪朝向从原扫描重新估计，局部几何最近观测距离最大约0.212m，显式上限0.25m；这部分是补全，不是测量。剩余白边/接缝等仍应由新场景的人工语义与图像审核处理，不能把此预览当成已替换房间的最终结果。

通用build重新导入两件GLB、每实体内1e-7m接缝焊接后审计通过，随后完成640×480、16sample Cycles预览。修正了GLB导出夹带默认Cube和UV/法向接缝造成的假拓扑开边问题。

`NAS/repository_assets/validation/scanner_azure/scanner_assets_preview.png`

该图仅为资产检查视角，资产摆放不是实验室注册结果；没有覆盖冻结背景。原始光照仍烘在部分纹理内，也未验证物理碰撞或真实布料动力学。

## Azure历史数值复现

使用历史scene_before中的相机/桌角、旧fit_whole_0.0中的机器人点，40起点联合优化焦距与外参，固定场景几何。

|指标|修改前|新入口结果|
|---|---:|---:|
|fit机器人点平均像素误差|12.80975|5.95077|
|held-out点平均像素误差|7.23871|6.75282|
|桌边1 RMSE|7.71109|2.15435|
|桌边2 RMSE|34.15121|0.228178|
|焦距px|364.52484|731.508043|
|完整几何正深度|不满足|满足|

新焦距与历史“机器人不动”候选差约8.37e-7px，数值流程复现通过。held-out改善有限，不能声称物理标定精度达到这些像素指标；新候选未重新覆盖baseline或进行全房间外观优化。

历史baseline保留的是734.304655px的另一阶段参数，另存`examples/lab_reference/azure/accepted_baseline_camera.json`。旧projection_K_verified字段残留364.525px，已作为来源冲突记录；没有修改原文件。最终采用与复跑候选分开保存。

## 自动检查与复跑

`validation/scanner_azure/pytest.txt`：15项测试通过。新增检查覆盖米制扫描刚体配准/留出点、原UV转移和有限填洞、七参数焦距/位姿恢复、冻结其他相机及旧标注hash拒绝。扫描register用已知变换的合成点测试；未虚构实测scan/world配对点。

```bash
cd /home/wangshuxun/VLA/data_sim/R2S2R
export PYTHONPATH="$PWD/src"
PY=/home/wangshuxun/VLA/data_sim/Data-MechanicSim/.venv/bin/python
CYCLES=/home/wangshuxun/VLA/data_sim/Data-MechanicSim/render_cycles/.venv/bin/python
$PY -m real2sim.cli fit-camera examples/lab_reference/azure/fit.json --out runs/azure_replay_new
$PY -m real2sim.cli scan-bake examples/lab_reference/scanner/door.json --out runs/door_new --python "$CYCLES"
$PY -m real2sim.cli scan-bake examples/lab_reference/scanner/cloth.json --out runs/cloth_new --python "$CYCLES"
```

这些命令只生成新候选；将T_world_asset设为新场景实际注册结果后，才能放入该场景。原始ZIP/照片/测量留在原资料库；Git中的示例和hash不代替原始资料备份。
