# 外观拟合的可执行接口

先锁定几何、相机和图像处理。一次只拟合少量参数。当前优化器只允许灯光功率和物体材质反照率倍率；灯光方向、粗糙度、响应曲线等仍需人工设置或另加经过验证的参数适配器。

`fit-appearance config.json --out NEW_DIRECTORY` 使用有界 Powell；每个候选重新打开同一个模板、进行完整 Cycles 渲染。不会将重复倍率累乘到上一次结果。mask包含的像素必须均在渲染valid mask内，否则直接失败。输出仅为候选，不自动覆盖baseline。

配置结构：

```json
{
  "scene": "scene.json",
  "blend": "scene.blend",
  "cycles_python": "/path/to/python-with-bpy",
  "samples": 32,
  "max_evaluations": 12,
  "holdout_tolerance": 0.002,
  "parameters": [
    {"key": "light/KeyLight/power_W", "bounds": [30, 180], "initial": 60},
    {"key": "material/TableTop/albedo_gain", "bounds": [0.8, 1.1], "initial": 1}
  ],
  "references": [
    {"camera_id": "CameraB", "profile_id": "REPLACE_WITH_VALIDATE_OUTPUT", "real": "reference/b.png", "mask": "masks/b.png", "split": "fit"},
    {"camera_id": "CameraCheck", "profile_id": "REPLACE_WITH_VALIDATE_OUTPUT", "real": "reference/check.png", "mask": "masks/check.png", "split": "holdout"}
  ]
}
```

路径相对config。真实输入分辨率须与对应profile输出一致。这里的JSON用于说明，camera/object id须替换为场景实际id，profile哈希必须来自`scene validate`。

输出：`parameters_NNNN.json`、`render_NNNN/`、`history.json`、`best.json`、`acceptance.json`。best只按fit loss选取；holdout不用于选择。验收还要人工检查光源方向、阴影、高光、颜色及前景曝光，不能只看综合分数。

当前fit仅用MAE/SSIM；单独`score --lpips-cache`可以额外报告LPIPS。不要把未安装/无缓存权重的LPIPS记为0。优化预算包括初始候选，预算耗尽不表示收敛。不同场景重新设定边界和评价mask。
