"""Tactile integration interfaces.

This module holds the contract and the sensor model interface; the simulation itself lives
next door in photon.py and photon_worker.py.
Honesty rule: tactile output is a simulated observation, and it must never be used to claim
that real contact has been validated.
"""
from __future__ import annotations

from typing import Protocol

CONTRACT_DRAFT = {
    "schema_version": "1.0",
    "status": "contact_driven_photon_integration",
    "sensor": {
        "photon": {
            "model": "Xense G1-WS optical tactile sensor (marker-based, 20x11 marker grid)",
            "backend": "tacsim (Data-TacSim) resolved from the interpreter passed to --python + vendor package xense-sim4.5 (committed under that checkout's third_party/xense_photon)",
            "integrated_path": "physical replay -> hydroelastic gel state -> Photon RGB/depth/marker flow",
            "gel_size_m": [0.0173, 0.02914, 0.003],
            "outputs": {"depth": [100, 64], "rgb": [700, 400, 3], "marker_flow": "[...,2]"},
        },
        "taxel_grid": "tactile array dimensions [rows, cols]; each taxel reports normal and shear force (N)",
        "frame": "T_world_sensor is a rigid 4x4 (T_A_B convention, metres); checked with contracts.rigid",
        "mount": "fixed | body; body mounting follows the wrist mounting semantics of the camera contract (needs a per-frame pose)",
        "output": "per-frame taxel forces plus the total force and total torque; units N / N*m, time in seconds",
    },
    "contact_backends": {"hydroelastic": "mujoco"},
    "notes": [
        "Simulated tactile output is not real-contact validation; reports must keep the simulation source and its uncertainty.",
        "Integration must not change a locked camera or base calibration; friction and other contact parameters follow the locking order in PIPELINE.md.",
    ],
}


class TactileSensorModel(Protocol):
    """Tactile sensor model interface (reserved, not implemented)."""

    def attach(self, body: str, T_body_sensor: list) -> None:
        """Attach the sensor rigid body to a given body; T_body_sensor must pass contracts.rigid."""
        ...

    def sense(self, state: dict) -> dict:
        """Sample one frame of simulation state and return an observation matching CONTRACT_DRAFT.sensor.output."""
        ...


def describe() -> dict:
    return CONTRACT_DRAFT
