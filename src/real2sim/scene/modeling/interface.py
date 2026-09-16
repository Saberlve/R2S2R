"""Video + Blender + Agent initial scene modeling interface (draft contract, not implemented).

Purpose: start from a room walkthrough video and have an Agent drive the existing Blender
worker (scene build/audit) to build a first scene.json draft whose geometry and scale are
correct, then refine it with scan registration (scene.scans) and distance optimization
(scene.spatial).

Hard rules:
- The output is always a **draft**: every camera and entity pose quality may only be
  "estimated", never "image_fitted" or "calibrated" (see the quality semantics in
  docs/CONTRACTS.md).
- Never overwrite an existing scene.json or a frozen baseline; every modeling run writes a
  fresh output directory.
- Video source material is read-only and is recorded in the inventory with its SHA256.
"""
from __future__ import annotations

from typing import Protocol

VIDEO_INTAKE_CONTRACT = {
    "schema_version": "1.0-draft",
    "status": "reserved_not_implemented",
    "videos": [
        {
            "path": "path to the original walkthrough video (read-only)",
            "camera_profile_id": "optional; links to a camera profile hash in CONTRACTS.md",
            "keyframes": "optional; explicit list of frame timestamps in seconds, never a silent subsample",
        }
    ],
    "world": {"length_unit": "m", "up_axis": "Z", "handedness": "right"},
    "notes": "The output is a draft scene.json (quality=estimated) for scan-register and spatial-fit to refine further.",
}


class InitialSceneModeler(Protocol):
    """Initial scene modeler interface (reserved, not implemented)."""

    def build(self, video_intake: dict, out_dir: str) -> dict:
        """Consume the video intake contract and produce a draft scene.json plus a modeling report.

        The implementation is expected to combine an Agent with the `r2s scene build/audit`
        worker. The returned report must list the evidence and the uncertainty behind every
        entity.
        """
        ...


def describe() -> dict:
    return VIDEO_INTAKE_CONTRACT


def build_draft(video_intake: dict, out_dir: str) -> dict:
    raise NotImplementedError(
        "Initial video modeling is a reserved interface; this round only provides the draft "
        "contract describe(). For now, have an Agent draft scene.json by hand following "
        "docs/AUTOMATION.md"
    )
