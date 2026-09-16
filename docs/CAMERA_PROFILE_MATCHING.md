# 离线图像对齐原生相机 profile

`tools/match_camera_profile.py` 将保存的图像映射到同一相机、同一光学坐标系的另一套内参。输入原图不改写，输出必须是新目录。源和目标 JSON 均明确包含 `width`、`height`、`K`、`D`、`serial`；当前仅支持全零畸变、标准无 skew 内参，目标视野必须包含在源图内。

```bash
python tools/match_camera_profile.py \
  --source-profile /case/source_profile.json \
  --target-profile /case/target_profile.json \
  --out /case/new_output /case/real.png /case/render.png
```

整数像素坐标代表像素中心。逐个目标像素计算 `p_source = K_source @ inv(K_target) @ p_target`，使用 OpenCV `remap`、`INTER_LINEAR`。边缘浮点误差用 `BORDER_REPLICATE` 处理；不会补黑边。输出内参就是目标 K。`pixel_mapping.json` 保存两套 profile、完整像素映射矩阵、输入路径和 SHA256。该文件为独立离线派生图像记录，不直接作为 scene.schema 的 pixel_ops 使用，也不复用原图标注/profile_id。

参考项目 `~/VLA/VLArmory/examples/realRobots/xArm7/lerobot_xarm7` 中，`config/gello/xarm7_gello_record_config.yaml` 为 RealSense 配置原生 640×480、30 FPS。其安装的 LeRobot `cameras/realsense/camera_realsense.py` 通过 `enable_stream` 请求对应彩色流；后处理检查原生尺寸、转换颜色及按配置旋转，没有将 1920×1080 做 letterbox。机器人适配器另有尺寸覆盖时的 resize 分支，不能把它误当作 RealSense 原生流的生成过程。

因此这里固化的是通过已知两档内参**匹配原生射线与视野**的离线方法，不声称复刻 RealSense 固件的 ISP、抗混叠、曝光或降采样核。不能仅按输出宽高直接拉伸原图。常见 1920×1080 到 640×480 的两档内参可对应左右裁剪再等比缩小，但具体映射必须由该设备实际 K 确定，不在工具中硬编码中心裁剪。

若直接渲染目标 profile，应使用目标分辨率和目标 K，并保持相机光学外参一致。若已经渲染了源 profile，使用本工具对实拍与渲染执行同一个映射。该处理不改变任何机器人或相机世界位姿。
