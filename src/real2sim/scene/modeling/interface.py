"""视频 + Blender + Agent 初始场景建模接口（契约草案，未实现）。

定位：从房间环拍视频出发，由 Agent 驱动既有 Blender worker（scene build/audit）
搭建几何与比例正确的初始 scene.json 草稿，随后由扫描注册（scene.scans）与
测距优化（scene.spatial）细化。

硬性规则：
- 产物一律是**草稿**：所有相机/实体位姿 quality 只能标 "estimated"，
  禁止标 "image_fitted" 或 "calibrated"（见 docs/CONTRACTS.md 的 quality 语义）。
- 禁止覆盖既有 scene.json 或冻结基线；每次建模写全新输出目录。
- 视频原始素材只读，纳入 inventory/SHA256 留档。
"""
from __future__ import annotations

from typing import Protocol

VIDEO_INTAKE_CONTRACT = {
    "schema_version": "1.0-draft",
    "status": "reserved_not_implemented",
    "videos": [
        {
            "path": "原始环拍视频路径（只读）",
            "camera_profile_id": "可选；关联 CONTRACTS.md 相机 profile 哈希",
            "keyframes": "可选；显式抽帧时间戳列表（秒），禁止静默抽帧",
        }
    ],
    "world": {"length_unit": "m", "up_axis": "Z", "handedness": "right"},
    "notes": "输出为草稿 scene.json（quality=estimated），供 scan-register 与 spatial-fit 继续收敛。",
}


class InitialSceneModeler(Protocol):
    """初始场景建模器接口（预留，未实现）。"""

    def build(self, video_intake: dict, out_dir: str) -> dict:
        """消费视频素材契约，产出草稿 scene.json 与建模报告。

        实现体预期为 Agent + `r2s scene build/audit` worker 的组合；
        返回报告需列出每个实体的依据与不确定度。
        """
        ...


def describe() -> dict:
    return VIDEO_INTAKE_CONTRACT


def build_draft(video_intake: dict, out_dir: str) -> dict:
    raise NotImplementedError(
        "视频初始建模为预留接口，本轮仅提供契约草案 describe()；"
        "当前请按 docs/AUTOMATION.md 由 Agent 手工起草 scene.json"
    )
