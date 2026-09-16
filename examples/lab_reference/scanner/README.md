# 原扫描的可复跑案例

door.json与cloth.json是本实验专用裁剪、规则化和外观参数。原OBJ/JPG位于Git外data/lab_scanner/{desk_and_door,green_background}/。zju已经放置这四个文件。

使用scene worker打包所需Python由--python指定。T_world_asset=identity仅用于资产制作检查，**不是房间真实配准**。合入新场景前必须用实测锚点/人工配准更新它。门板erase_mask来自历史手工区域，只能用于该裁剪与分辨率。不可直接套用到另一扇门。
