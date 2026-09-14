# Azure：内参不可靠时的焦距与位姿联合拟合

这是本项目让Azure视角明显贴近实拍的关键步骤。原流程只有固定K的PnP不足以描述它：当焦距本身错了，仅移动/旋转相机无法同时对齐近处桌边与远处机器人。

## 历史流程及采用原则

1. 先核查相机数据流的真实分辨率、裁剪、缩放、翻转。Azure原始对比使用1280×720；拼接时letterbox到640×480不代表相机变成4:3。
2. 尝试读取Azure厂家参数失败后，不继续假定旧焦距可靠。初始模型在1280×720下约364.525px，视角过宽。
3. 固定已经被B确认的主桌、机器人实际姿态和基座；B也固定。收集已知3D位置的机器人轮廓点以及桌边直线，标出其对应的实拍原生像素。
4. 先做六维外参搜索作为诊断；若桌边和机器人不能同时对齐，改为焦距+六维外参七参数搜索。固定主点、方形像素，零畸变只是当前缺资料条件下的明确假设。
5. 多起点、有界、soft-L1稳健最小二乘；使用机器人点误差、桌边点到图像直线的距离、宽松焦距/旋转先验及正深度约束。保留未参与拟合的点检查。
6. 回到完整Cycles渲染对照实拍，检查B和被冻结对象没有改变；再决定是否采纳候选。

历史上试过机器人整体/第一关节小角度候选，后来用户要求恢复原机器人姿态。**这些旋转搜索不是新场景的默认必做步骤**；通用fit-camera根本不允许修改机器人。最终baseline相机参数应以baseline/camera_config.json实际供render.py使用的K和T为准，不直接拿某个中间候选覆盖。

本实验baseline保留的Azure焦距约734.304655px；“机器人不动”历史数值候选约731.508042px。它们是不同阶段产物，仓库分别保存并注明，不能混成同一份标定。

## 输入输出

输入配置严格包含`schema_version=1.0, scene, camera_id, annotations, search`。scene与annotations路径相对配置。

annotations严格包含：

- scene_sha256：提取3D点时的scene文件哈希，变化后原标注失效。
- profile_id：原生图像尺寸/K/D/pixel操作的哈希；必须匹配初始相机。
- coordinates：固定为native_pixels。历史1920→1280的手工标注已换算后保存；新项目须使用明确pixel变换，不能暗中resize。
- points：每项world_m[3]、pixel_xy[2]、split=fit|holdout；至少4个fit点、1个holdout点，fit点非共线。建议覆盖多个高度和深度，数量越少越易退化。
- lines：每项world_points_m[N,3]、pixel_endpoints_xy[2,2]、split。直线只约束法向距离，不约束沿线滑动，因此还要点/角点。
- depth_guard_points_world_m：完整桌角/机器人几何等正深度检查点。这些几何点不提供留出2D误差给优化器。

search严格包含：focal_bounds_px、position_bounds_m（两行世界XYZ上下界）、rotation_bound_rad、starts、seed、max_nfev、focal_prior_px、focal_prior_log_sigma、rotation_prior_rad、point_sigma_px、line_sigma_px。

旋转是相对初始光学相机的世界轴rotvec增量；位置是世界米；优化log(f)保证焦距为正。示例用40起点、seed24、soft-L1 f_scale=2、最大450次函数评估/起点。例子的焦距480–850px、位置边界都只适合该历史1280×720案例，新相机重新设置。

输出新scene.json与camera_fit_report.json：before/after点误差、分开列出的fit/holdout均值、桌边RMSE、各起点cost、焦距、T_world_optical、profile变化、边界触碰、正深度、冻结对象一致性、假设与来源哈希。profile因焦距改变会更新；不能再把旧profile的标注当作已匹配新profile的数据重复使用。

```bash
$PY -m real2sim.cli fit-camera examples/lab_reference/azure/fit.json --out runs/azure_candidate
# 将候选相机配置接入真实场景模板后，沿用render进行全场景验证。
```

数值搜索本身不运行Blender也不修改blend。它只改变指定相机的K/T；其他相机、资产、灯光和世界参数逐项保持一致。适用已明确无畸变/已显式校正的图像，或本例明确标为假设零畸变的情况。有可靠厂家K/D时优先走固定K标定，不用这条弱约束分支替代。

## 什么算通过

- 桌边和机器人同时改善；留出点无明显退化；不触碰无合理解释的参数边界。
- 几何正深度成立；相机不穿进桌面/墙里；真实观察覆盖范围合理。
- 相同原生图像与ROI重渲染，不能将裁剪后对齐误认成外参正确。
- 检查完整渲染中的遮挡和两个视角，不能只看训练重投影误差。

`quality`始终image_fitted，automatic_adoption=false。即使Jacobian满秩，它包含先验，也不是物理参数置信度。只有独立标定板/测量与多姿态验证才能升级物理精度结论。

## 历史配置字段冲突

旧baseline Azure条目的K约734.305，但projection_K_verified还残留364.525，max_K_error_px也不能描述这两个字段的差异。检查旧render.py确认实际使用的是K。新pipeline不信任这种陈旧验证字段：重新用相机配置计算投影并渲染，报告源文件与参数版本。仓库没有修改原baseline，仅记录这个来源问题。

scene_sha256锁定的是场景描述，不自动覆盖外部blend/mesh字节。提取3D点时应同时保存模板/资产inventory；完整重渲染前检查这些hash。案例scene仅供历史数值重放，机器人点已经冻结在annotations中，不包含完整机器人视觉资产。
