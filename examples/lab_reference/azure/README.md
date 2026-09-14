# 历史Azure数值重放

fit.json/scene.json/annotations.json足以在CPU上运行fit-camera，重放“机器人不动”的历史候选。scene只含拟合所需的桌面描述，机器人点已冻结在annotations，不能当作完整场景模板渲染。accepted_baseline_camera.json另存历史最终采用状态，不作为本次搜索初值或自动覆盖目标。

请勿改动scene.json的内容或格式后继续使用旧scene_sha256。新项目应重新采样3D/2D对应点并保存来源模板/图像hash。
