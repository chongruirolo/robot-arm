"""
Inverse kinematics for the Piper arm using ikpy + the official URDF.

Usage
-----
    from arm_ik import PiperIK
    ik = PiperIK()

    # Get joint angles (degrees) to reach a target XYZ
    joints_deg = ik.solve(x=0.25, y=-0.29, z=0.22)

    # Solve with a seed (current joint angles) for a more predictable solution
    joints_deg = ik.solve(0.25, -0.29, 0.22, seed_deg=current_joints)

    # Check where the arm currently is in XYZ given joint angles
    xyz = ik.fk(joints_deg)
"""

import math
import os
import numpy as np
import ikpy.chain

URDF_PATH    = os.path.join(os.path.dirname(__file__), "piper.urdf")
ACTIVE_LINKS = [False, True, True, True, True, True, True, False, False]
Z_OFFSET     = 0.1357  # URDF base frame is 135.7 mm lower than arm's reported frame


class PiperIK:
    def __init__(self):
        self._chain = ikpy.chain.Chain.from_urdf_file(
            URDF_PATH,
            active_links_mask=ACTIVE_LINKS,
        )

    def solve(self, x: float, y: float, z: float,
              seed_deg: list[float] | None = None) -> list[float]:
        """Return 6 joint angles in degrees for the target XYZ (metres).

        seed_deg: current joint angles in degrees — helps IK find a nearby
                  solution rather than a random one.
        """
        seed_rad = self._to_ikpy_rad(seed_deg) if seed_deg else None

        result = self._chain.inverse_kinematics(
            target_position=[x, y, z - Z_OFFSET],
            initial_position=seed_rad,
        )
        # result has 9 values (one per chain link); extract the 6 active ones
        return [math.degrees(result[i]) for i in range(1, 7)]

    def fk(self, joints_deg: list[float]) -> np.ndarray:
        """Forward kinematics — return XYZ (metres) for given joint angles."""
        rad = self._to_ikpy_rad(joints_deg)
        matrix = self._chain.forward_kinematics(rad)
        xyz = matrix[:3, 3].copy()
        xyz[2] += Z_OFFSET
        return xyz

    def _to_ikpy_rad(self, joints_deg: list[float]) -> list[float]:
        """Convert 6 joint degrees to the 9-element ikpy format (radians)."""
        rad = [math.radians(d) for d in joints_deg]
        return [0.0] + rad + [0.0, 0.0]
