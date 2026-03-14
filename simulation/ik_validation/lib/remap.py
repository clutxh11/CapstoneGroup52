"""
Remap layer: PID outputs [Roll_T, Pitch_T, Yaw_T] -> [Roll_T', Pitch_T', Yaw_T'] before IK.

Swap Roll/Pitch only (no sign flip). Matches Simulink when roll/pitch directions differ.
Chosen: swap=True, roll_sign=+1, pitch_sign=+1.
"""

from __future__ import annotations

import numpy as np

REMAP_SWAP = True
REMAP_ROLL_SIGN = 1.0
REMAP_PITCH_SIGN = 1.0


def remap(tau: np.ndarray) -> np.ndarray:
    """Remap [Roll_T, Pitch_T, Yaw_T] -> [Roll_T', Pitch_T', Yaw_T']. Modifies only roll/pitch."""
    r, p = float(tau[0]), float(tau[1])
    if REMAP_SWAP:
        r, p = p, r
    r *= REMAP_ROLL_SIGN
    p *= REMAP_PITCH_SIGN
    return np.array([r, p, tau[2]], dtype=float)
