"""General trajectory generation interfaces.

Current implementation: real2sim.traj.adapters.xarm7 (Newton engine, xArm7 + G2 + sensor-center TCP).
These interfaces describe the surface that future planners and other engines plug into; this
round adds no new implementation behind them.
The trajectory data contract follows the episode NPZ schema in docs/CONTRACTS.md
(time/q/action_q/gripper/action_gripper, radians and seconds, gripper 0=open,1=closed).
"""
from __future__ import annotations

from typing import Protocol

ENGINE_REGISTRY = {
    "newton_xarm7": {
        "adapter": "real2sim.traj.adapters.xarm7",
        "status": "implemented",
        "planning": "real2sim.traj.planner + plan_trajectory.py (our own implementation, design informed by newton_gen.motion.planning, no runtime dependency on it)",
        "hardware_guard": "xArm7 G2 sensor-center TCP; no real-robot commands",
    },
    "tacsim_tactile": {
        "adapter": None,
        "status": "reserved_not_implemented",
        "notes": "tactile simulation backend hook; see real2sim.tactile.interface",
    },
}


class TrajectoryGenerator(Protocol):
    """Task-level trajectory generator interface (reserved, not implemented)."""

    def generate(self, task_spec: dict, scene_config: dict) -> dict:
        """Generate an episode (NPZ + manifest) from a task description.

        Requirement: the output must pass the existing kinematics and dynamics gates (the FK,
        tracking and penetration thresholds are in docs/CONTRACTS.md). A generated trajectory
        is not real-trajectory contact validation.
        """
        ...


class EngineAdapter(Protocol):
    """Simulation engine adapter surface: the capabilities the xarm7 adapter already implements."""

    def validate_kinematics(self, case: str) -> dict: ...
    def simulate_replay(self, case: str) -> dict: ...
    def export_trajectory(self, case: str) -> dict: ...
