# 大文件资源与渲染产物

GitHub只管理可复用代码、配置和文档。没有录制视频、扫描网格或渲染图片/视频；仓库中的小型`door_erase_mask.png`是扫描清理所需的人工标注mask，不是渲染结果。

唯一例外是仓库级物体资产库 `assets/`：跨任务复用的**小物体**可以提交，单文件 ≤ 2MB，仅限 `.glb`/`.gltf`（`.bin` 边车）与 `textures/` 下的 `.png`/`.jpg`。判定在 [tools/check_git_payload.py](../tools/check_git_payload.py)，`.gitignore` 的例外规则与它逐条对应，两者必须一起改。扫描件、渲染结果、录像、数据集仍然一律放 NAS，不因为这条例外而放宽；库的用法见 [assets/README.md](../assets/README.md)。

统一资源根目录：

```
/run/determined/NAS1/public/wangshuxun/r2s2r_resources
```

| 子目录 | 内容 |
|---|---|
| `real2sim/` | 全部案例、当前场景、相机校准、原始资料、仿真状态及渲染结果 |
| `real2sim/local_imports/20260914/` | 本地完整上传包及逐文件校验 |
| `real2sim/MeasuredBarGraspV1/RealReplayComparisonV1/raw/` | 真机重放录像与日志 |
| `real2sim/WristCameraAlignmentV1/runtime/` | 当前经过腕部外参修正的场景模板 |
| `datasets/xarm7_gello_pick_bar/` | 原始38段关节采集数据及视频 |
| `repository_assets/validation/` | 从代码仓库移出的历史渲染图 |
| `repository_data/`、`repository_runs/` | 原仓库Git外的输入资料和运行产物 |
| `resource_manifest.json` | 来源、目标相对路径、大小、SHA-256 |

`examples/site.zju.env`用`R2S_*`环境变量直接指向上述目录，默认新运行结果也写入NAS，不再写入代码仓库。实际机器配置复制为被Git忽略的`site.local.env`再改。环境变量不会随Git传播，换机器必须自己设。

Newton工程和已安装Python环境仍是独立软件依赖，见DEPENDENCIES.md。数据和渲染结果不可用Git LFS绕过此资源分离约定；资产库同样不使用 LFS——它靠的是上面的尺寸与格式上限，不是靠大文件存储。

此前本地已经不存在的两份旧扫描ZIP和四张临时截图仍列在迁移记录中；NAS提供已有派生资产与记录，不冒充缺失原件。

兼容旧路径：`/home/wangshuxun/VLA/data_sim/real2sim` 已指向NAS的 `real2sim/`，因此旧URDF和标定文件中的绝对资源路径仍可读取。原服务器副本保留为 `real2sim_before_nas_20260914`，本次未删除。新实验输出按NAS site配置写入。

发布前运行 `python3 tools/check_git_payload.py` 检查全部可达Git历史，拒绝渲染/资源二进制和大于5MB的blob。初始NAS迁移验证14076个文件、8,881,557,800字节；NAS加载场景渲染和26项测试通过。
