# 来源与复用边界

本仓库从一次实际场景工程中提炼，并重新实现为可配置接口；不是把旧版本目录整体复制成产品。

|旧产物/阶段|提炼到本仓库|
|---|---|
|LidarBackgroundTrialV1/analyze_scans.py、prepare_assets.py、clean_cloth.py、bake_scan_uv.py、finalize_mcp.py|scans.py及scan_asset.py：语义裁剪、原UV烘焙、规则门板、绿布局部起伏、曝光修补、单物体闭合GLB|
|AzureOnlyAlignV1/search_azure_final.py、RobotYawAzureJointFitV1/fit_azure.py|camera_fit.py：多起点七参数焦距+外参、桌边/机器人点、先验及深度约束；历史零旋转候选数值复现|
|NativeCameraRealSimCompareV1、CameraPoseSearchV1|native profile核对、pixel变换、可信内参下的PnP与分阶段注册|
|StableReferencePhotometricV1、UpperLeftLightingV1|固定ROI、限制外观参数、完整渲染复核；替换历史专用ROI硬编码|
|WristAssetsAlignedV1、baseline/render_api.py|body-local固定绑定、同状态多相机渲染；bridge重新加了刚体矩阵检查|
|EEFAlignmentV1/src|7个经过运行的xArm7适配脚本，移除案例根路径与自动估计任务物体位置；核心FK接口单独实现|
|EEFAlignmentV1/REPORT.md|明确记录通过和未通过的阶段，不把开发成功样例扩展为普适验收结论|

第三方Newton、Warp、Data-MechanicSim、Blender、SDK与资产保留各自许可；本仓库不重新打包它们，不默认分发真实照片/视频/用户扫描资产。未为整个历史工程擅自指定新的开源许可。

## 触觉与轨迹新增来源（2026-09-15）

- **Data-TacSim**（`https://git.weiyantech.cn/Worlddynamics/Data-TacSim.git`，git 子仓库 `external/Data-TacSim`，钉在 master `12793a2`）：Photon（Xense G1-WS）触觉后端。**该仓库无 LICENSE 文件**，按内部代码/全部权利保留处理，不得对外再分发。
- **xense-sim4.5 厂商仿真包**（Xense Robotics 专有）：运行时部署在 `external/Data-TacSim/third_party/` 下（子仓库 `.gitignore` 忽略区），**不进 git**。
- **`src/real2sim/traj/planner.py`**：自有实现，隔离契约设计（结构化 `Infeasible`、`validate_limits`、FK 校验分离）参考 Data-MechanicSim `newton_gen/motion/planning/interface.py`；不 import `newton_gen.motion.*`，无运行时依赖，便于后续自行修改轨迹生成逻辑。

新场景运行的receipt和环境版本应随成果保存。旧实验数据位于原real2sim目录，不由本仓库清理脚本修改。
