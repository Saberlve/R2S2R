# 3D Scanner / iPhone LiDAR 背景重建

这是本项目背景真实性改善的主要来源，不能省略成“准备一个GLB”。目标是把扫描的真实纹理和局部表面细节转移到可编辑、尺度受约束的独立实体。

## 本次实际采用了什么

输入为`green_background.zip`和`desk_and_door.zip`。前者345张RGB、288285个纹理三角面；后者335张RGB、248119个纹理三角面；均有8192×8192纹理。扫描包包含OBJ/MTL/JPG、refined OBJ、逐帧手机相机信息、ARKit world map等。没有证据表明包内提供了可直接使用的逐帧独立深度图；ARKit二进制world_map不能冒充已解码RGB-D。

- **绿布**：保留照片纹理和局部褶皱；最终使用连续网格，去掉扫描整体弯曲趋势，将局部起伏限制到±25mm，增加1.5mm视觉厚度。它是静态视觉外壳，不是布料动力学模型。
- **白门**：独立规则门板维持0.89×2.08m；从扫描OBJ的原UV和8K图集中重新烘焙门板外观。门把手保持独立，纹理中已有的把手/阴影按明确mask修补，避免重复出现。图中已有横向分缝时，不再叠加一套重复线条。
- **墙面**：从无遮挡扫描墙面取得较干净的纹理片段，转移到规则墙体。
- **没有直接采用**：反光玻璃、缺失的阳台扫描表面、不完整的邻桌扫描几何。这些仍用实测约束和简单独立结构补全。

历史扫描试验先用Blender验证，后来进入Newton+Cycles的场景模板。不能把早期Blender试验报告误写成已经通过Newton接触测试。

## 阶段与严格输入输出

|阶段|输入|输出|检查|
|---|---|---|---|
|S0 留档/检查|原ZIP、OBJ/MTL、原始纹理、手机元数据；明确资料版本|原始文件hash、`scene scan-inspect`报告|不执行包内脚本；不把手机K当Azure K；OBJ单位未知时明确标注|
|S1 尺度与配准|至少3个非共线scan/world配对点及1个留出点、明确units_to_m|`scene scan-register`输出固定尺度的刚体T与fit/holdout残差|不允许通过非均匀缩放或移动主桌/机器人/B相机掩盖错误|
|S2 语义裁剪|扫描平面坐标、u/v/depth范围、法向阈值、可选颜色门限|选中的表面三角面、裁剪配置|颜色仅辅助，不能自动证明语义正确；独立排除线缆、主桌、装置残留|
|S3 纹理转移|原OBJ的独立vertex/UV索引、原图集、明确区域|basecolor.png、observed/filled mask、fill_distance_m.npy|用三角形重心插值读取原UV；不把断裂的UV岛自动当漂浮噪声；限制填补距离|
|S4 规则几何|panel实测宽高/厚度，或relief网格/平滑尺度/深度上限|独立surface.obj、闭合asset.glb、asset.json|规则门板不能沿用扫描裁剪宽度代替实测门宽；绿布不随意整体拉伸|
|S5 视觉审核|独立资产、冻结的scene/cameras、相同实拍ROI|Cycles渲染、背景对比、覆盖和误差报告|检查接缝、光照重复烘入、反光、遮挡、UV方向；通过后才进入baseline|

先用门尺寸、2.1m总间距、0.9m邻桌前沿间距、1.2m余量和0.39/0.78/0.95m侧墙段核对尺度与空间关系。**这些是本实验数据，不是新场景默认尺寸。** 本次门板源裁剪宽约0.78个假定米制扫描单位；输出门宽仍由独立0.89m测量决定，是纹理重投影，不是把0.78当作0.89的实测同一段。

## 可执行命令

```bash
$PY -m real2sim.cli scan-inspect --obj /data/scan/textured_output.obj --texture /data/scan/textured_output.jpg --out runs/scan_report.json
$PY -m real2sim.cli scan-register /data/registration_points.json --out runs/registration.json
$PY -m real2sim.cli scan-bake /data/door_bake.json --out runs/door_asset --python "$CYCLES"
```

`scene scan-bake`在主Python中做UV转移/规则化，使用独立bpy进程打包GLB；没有安装或替换Newton。每次使用新目录。将产物asset.json条目合入scene.assets，按scene文件位置重设其相对GLB路径，再运行已有build/render。

配准输入：`schema_version=1.0, scan_points[N,3], world_points_m[N,3], split[N], units_to_m>0`。变换严格为：

`p_world = T_world_scan_m @ [units_to_m * p_raw_scan, 1]`

尺度由独立尺寸确认后锁定，register不优化尺度；fit只用fit点，holdout单独报告。没有真实配对点时，可以手工配准，但只能标estimated。扫描平面PCA只帮助确定裁剪朝向，不能代替世界坐标配准。

## scan-bake配置契约

可运行的本实验配置见`examples/lab_reference/scanner/`，不是通用默认值。

- 顶层严格包含：schema_version、id、source_obj、source_texture、units_to_m、T_scan_m_plane、T_world_asset、crop、texture_wh、max_fill_distance_m、max_unknown_fraction、surface、appearance。
- 源文件路径相对配置。单个三角化OBJ、单材质图集、每面独立UV索引；多材质包先拆分，未三角化或缺UV会拒绝。
- `T_scan_m_plane`将局部平面(u,v,depth)转换到米制scan坐标；`T_world_asset`只指定生成资产的世界放置，不自动猜测房间位置。
- crop严格包含`u_range_m/v_range_m/depth_range_m`、`normal_abs_depth_min`和`green_gate`。green_gate=null或`{min_g,g_over_r,g_over_b}`，RGB范围0–1。
- surface包含`mode=panel|relief`、grid_wh、smooth_sigma_cells、trend_sigma_cells、relief_limit_m、depth_sign、thickness_m、target_size_m、max_grid_fill_distance_m。
- panel：target_size_m=[实测宽,实测高]，资产原点为前表面中心，厚度沿局部-Z。扫描纹理映射到该规则面。
- relief：target_size_m必须null；保留源平面u/v米制跨度，深度去趋势并裁限，原点沿用局部扫描平面。grid与sigma变化会改变平滑的物理尺度，需同时记录。
- max_fill_distance_m限制纹理最近邻填洞，max_grid_fill_distance_m单独限制几何网格补全距离。超出纹理距离的像素为中性灰并标为unknown；unknown比例超限即失败。补全从来不是测量。
- appearance严格包含erase_mask和exposure_normalization。mask为与输出纹理同尺寸的L图；归一化可为null或`{sigma_px,target_mean,gain_bounds}`。它只能近似削弱曝光不均，不能把含阴影的图集变成真实反照率。

完整输出：surface.obj/mtl、basecolor.png、observed_mask.png、filled_mask.png、fill_distance_m.npy、asset.glb、asset.json、build.json、scan_report.json、mesh_audit.json。GLB检查仅一个mesh/scene、闭合边和unit scale。闭合不代表真实可碰撞资产；碰撞仍使用独立简单静态代理。

## 本轮复跑边界

本轮使用原始OBJ/JPG重新跑通了门与绿布，包括UV烘焙、纹理修补分支、闭合GLB打包、通用scene导入和Cycles预览。绿布裁剪基准用原扫描重新估计，仍为示例资产空间；未将新候选覆盖已冻结房间。原有手工房间配准属于历史采用结果，新的通用register另用已知变换测试验证。详见SCANNER_AZURE_VALIDATION.md。

GLB可能为UV/法向接缝复制顶点，重新导入后出现拓扑边界。scanner产物显式设置weld_distance_m=1e-7，build仅在该独立实体内部合并近乎完全重合的顶点，保留逐角UV，再重新检查闭合；不会跨物体焊接，也不使用毫米级容差吞掉薄层。

配准矩阵的接入：relief可从T_world_scan_m @ T_scan_m_plane得到T_world_asset，再审核去趋势后平面位置；panel的原点在裁剪中心，因此还需右乘Translate((u_min+u_max)/2,(v_min+v_max)/2,0)。已知门板尺寸直接生成几何，不把尺寸修正塞进刚体矩阵。相对应的测量点必须真的是同一物理点，不能把扫描裁剪角当成整扇门角。
