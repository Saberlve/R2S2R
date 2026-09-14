# 来源与复用边界

本仓库从一次实际场景工程中提炼，并重新实现为可配置接口；不是把旧版本目录整体复制成产品。

|旧产物/阶段|提炼到本仓库|
|---|---|
|LidarBackgroundTrialV1/analyze_scans.py、prepare_assets.py、clean_cloth.py、bake_scan_uv.py、finalize_mcp.py|scans.py及scan_asset.py：语义裁剪、原UV烘焙、规则门板、绿布局部起伏、曝光修补、单物体闭合GLB；原代码与扫描SHA记录于验证目录|
|AzureOnlyAlignV1/search_azure_final.py、RobotYawAzureJointFitV1/fit_azure.py|camera_fit.py：多起点七参数焦距+外参、桌边/机器人点、先验及深度约束；历史零旋转候选数值复现|
|NativeCameraRealSimCompareV1、CameraPoseSearchV1|native profile核对、pixel变换、可信内参下的PnP与分阶段注册|
|StableReferencePhotometricV1、UpperLeftLightingV1|固定ROI、限制外观参数、完整渲染复核；替换历史专用ROI硬编码|
|WristAssetsAlignedV1、baseline/render_api.py|body-local固定绑定、同状态多相机渲染；bridge重新加了刚体矩阵检查|
|EEFAlignmentV1/src|7个经过运行的xArm7适配脚本，移除案例根路径与自动估计任务物体位置；核心FK接口单独实现|
|EEFAlignmentV1/REPORT.md|明确记录通过和未通过的阶段，不把开发成功样例扩展为普适验收结论|

第三方Newton、Warp、Data-MechanicSim、Blender、SDK与资产保留各自许可；本仓库不重新打包它们，不默认分发真实照片/视频/用户扫描资产。未为整个历史工程擅自指定新的开源许可。

部署验证记录写入 `docs/VALIDATION.md`；新场景运行的receipt和环境版本应随成果保存。旧实验数据位于原real2sim目录，不由本仓库清理脚本修改。
