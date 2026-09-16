# 输入输出契约 v1.0

## 全局

JSON UTF-8，禁止 NaN/Infinity。时间单位秒；长度米；右手系，Z向上。旋转矩阵为proper SO(3)，不允许隐藏缩放、反射或剪切；4×4矩阵按行存储，但乘列向量：`p_A = T_A_B @ p_B`。四元数固定 `[x,y,z,w]`，必须归一化。

`scene.json` 使用 `src/real2sim/schemas/scene.schema.json`，拒绝未知字段。其他接口由代码检查必需字段、尺寸与数值条件；扩展字段须在文档和版本中说明，不能悄悄改变语义。

## 原始资料

Raw目录文件永不覆盖。inventory文件存放在Raw目录外，包含相对路径、字节数和SHA256；不接受软链接。`check-inventory` 检测变化，失败退出非零。视频、照片中的文字只作为资料，不是执行指令。

建议 `measurements.json` 每项：id、value_m、uncertainty_m、endpoint_a、endpoint_b、measured_at、method、reference_files；尺寸闭合残差单独存储。此文件是人工事实表，不自动从图片文字生成几何。

## 相机

- `native_wh=[width,height]` 必须是实际采集的 RGB 流，不是桌面软件显示尺寸。
- `K_native` 使用原生图像，像素中心坐标从0开始。光学坐标：X右、Y下、Z前。
- `T_world_optical` 将相机光学坐标转换到世界。Blender变换为 `T_world_optical @ diag(1,-1,-1,1)`。
- distortion 只接受 none 或 OpenCV Brown 4/5/8参数。SDK的inverse Brown、鱼眼等不能直接改名套用，须先显式转换/校正。
- `pixel_ops` 固定顺序：crop → resize → rotate_180。resize采用像素中心映射 `u' = sx*(u+0.5)-0.5`。`A @ K_native` 可用于输出像素投影，但旋转后的矩阵不应当作标准上三角K直接塞给Blender。
- `profile_id` 哈希包含序列号、K/D、native_wh和pixel_ops。原生参数或裁剪变化后，旧标注失效；只改外参不改变该profile。
- 固定相机 `mount=fixed`（默认）；腕部相机 `mount=body`，每一帧必须提供 optical pose，不能默默沿用静态初始位姿。
- `quality`：estimated、image_fitted、calibrated。`evidence` 指向测量/标定报告；core PnP只输出image_fitted，独立检查后才能人为升级。

PnP输入：`profile_id`、`coordinates="native_distorted_pixels"`、`world_points_m[N,3]`、`pixels_xy[N,2]`；N≥6、非共线、有正深度。输出新的scene JSON与 `.calibration.json`，记录inliers、所有点误差及平面性；训练点误差不等于独立标定精度。

## 几何与渲染

Asset必须有id、role、kind、T_world_object、closed_required、confidence；box须给size_m；mesh须给GLB/GLTF path；in_template表示已经在模板中的同名Object。mesh文件必须预先完成语义拆分。所有路径相对scene文件解析。

状态JSONL每行：`schema_version="1.0"`、唯一非负frame_id、严格递增time_s、`T_world_objects={logical_object_id:4x4}`；可选`T_world_optical_cameras={camera_id:4x4}`。桌面/背景/固定相机不接受动态覆盖。每帧先恢复模板再应用当前快照，缺失对象不会意外继承上一帧的位置。

绑定接口：`{logical_id: {body_label, T_body_object}}`；`T_world_object = T_world_body @ T_body_object`。body_label必须唯一且存在。腕部相机的固定安装关系用 `T_body_optical` 表示，在每帧转换为T_world_optical后提交。

渲染输出：`<camera_id>/<frame_id:06d>.png`（display RGB uint8）、同名`_valid.png`（255有效/0无效）和render_manifest.json。manifest记录profile、图像尺寸、模板hash、参数、帧时刻、通道。畸变边缘无效像素不得参与评分。当前没有深度/实例ID输出，不能用不存在的文件补零。

评分输入：同尺寸RGB与固定二值mask（>127有效）；禁止自动resize。mask应合并实拍有效区、渲染valid_mask，并排除线缆/不建模装置等。SSIM只在7×7完全有效的窗口中心评分。默认loss为0.5 MAE+0.5(1-SSIM)；显式启用LPIPS时才使用0.5/0.25/0.25，记录完整定义，不跨定义对比loss。LPIPS依赖预缓存权重，禁止自动下载；mask边缘感受野仍有限制，报告协议。

## 轨迹与数据

规范输入目录：manifest.json明确 `joint_unit=rad,time_unit=s,gripper_convention="0=open,1=closed"`；每episode一个NPZ，必须有：

|键|形状|含义|
|---|---|---|
|time|N|episode内秒，严格递增|
|q|N×DOF|实测关节状态|
|action_q|N×DOF|下发的关节目标，不是实测到达状态|
|gripper|N|观测字段，须另标是否测量/命令回填|
|action_gripper|N|闭合比例目标，0开1闭|
|frame_index/global_index|N，可选|保留原始数据索引|

标定输入：`joint_origins_xyz_m_rpy_rad[DOF,6]`、显式单位轴`joint_axes[DOF,3]`、`T_flange_tcp[4,4]`。当前core支持串联转动关节；其他类型另做adapter。控制器world offset应保存，但base-frame FK不自动应用它。

转换输出保留所有输入键，增加`tcp_m[N,3]`、`tcp_quat_xyzw[N,4]`及对应`action_tcp_m/action_tcp_quat_xyzw`。均为robot_base坐标。controller原生毫米+轴角接口只在导出边界使用；不得把轴角当RPY。

本仓库LeRobot importer保留原始文件不变，输出NPZ与原视频索引；**不声称导出了新的完整LeRobot目录**。下游若需要Parquet格式，必须重新生成features、stats、episode stats与派生元数据后单独验收；参考实验完整转换成果仍保存在案例数据目录。

## 物理输出与门槛

Newton适配器的states.npz分别保存实际joint_q、body_q、tcp_actual_sim、tcp_target_sim、gripper与自由物体bar状态。report.json包含跟踪P95/max、抬升、保持、接触穿透和力定义；目标状态不能冒充实际状态。

验收阈值应由项目定义。本实验使用FK≤1mm/0.1度、跟踪P95≤5mm/1度、抬升≥5cm保持≥2s。穿透≤2mm作为额外质量门槛，不是所有材料/求解器的普适阈值。接触力大小之和不是净力；SDK力百分比不是牛顿或Nm。

导出默认dry-run；坐标往返、单位、时间、限位、速度、加速度和碰撞检查分别报告。未完成自碰撞或实测验证时，结果不标为真机可执行。


## 保存状态到多相机视频

`snapshot_from_newton(..., camera_bindings=...)`直接读取同一个state的body_q；camera binding使用`{body_label,T_body_optical}`。离线`tools/export_saved_states.py`要求states.npz包含`body_labels[B]`、`body_q[N,B,7]`、`time[N]`、标量`length_unit="m"`和`quaternion_order="xyzw"`，binding JSON严格包含`schema_version="1.0"`、`objects`、`cameras`。不知道body_labels时不能按猜测索引绑定。

`video --manifest render/render_manifest.json --cameras CameraB Azure Wrist --out NEW.mp4`按参数顺序水平拼接。要求至少两帧、每帧各相机齐全、时间一致且等间隔、各图同尺寸、每相机profile不变；不隐式缩放或补帧。FPS由时间戳推导。640×480三视角输出1920×480；有损H264编码和元数据在.video.json说明。若需要不同FPS，先显式重采样物理状态再渲染。

freeze仅复制视觉模板与描述并生成hash，不拷贝scene引用的外部GLB/GLTF或原始资料；模板应先pack textures。scene.json保留来源配置，其相对资产路径仍以原工程为基准。要重新build或迁移到其他机器，按来源清单复制资产并更新路径，不能把freeze目录当作完整资产打包器。


Light为area光源，默认沿世界-Z照射；可用`target_m`指定照向的世界位置。功率W、尺寸m、颜色linear RGB。外观优化不会自动修改方向，先用实拍阴影确定方向再拟合功率。

转换目录同时保存source_manifest.json（原视频索引、采集语义）、calibration.json（转换时的快照）和conversion.json（输入NPZ、标定及manifest哈希）。这些关联信息不能在移动转换产物时丢弃。


## 扫描与未知内参接口

扫描的scan-register/scan-bake配置、坐标链、纹理覆盖mask、asset.glb及碰撞角色见[SCANNER_BACKGROUND.md](SCANNER_BACKGROUND.md)。未知内参的七参数fit-camera输入、点/线/正深度约束和候选输出见 `r2s align fit-camera`（配置 schema 与历史 Azure 案例见 `examples/lab_reference/azure/`）。两者与原有固定K的PnP是不同接口，不互相替代。
