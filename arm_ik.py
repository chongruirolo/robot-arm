"""
Inverse kinematics and Cartesian path planning for the Piper arm.

Single-point IK:   uses ikpy (fast, lightweight)
Cartesian paths:   uses roboticstoolbox ctraj — generates a straight line
                   in Cartesian space and solves IK at each waypoint,
                   seeding each step from the previous solution for smooth posture.

Usage
-----
    from arm_ik import PiperIK
    ik = PiperIK()

    # Single-point IK
    joints_deg = ik.solve(0.25, -0.29, 0.22, seed_deg=current_joints)

    # Straight-line Cartesian path between two joint configs
    waypoints = ik.cartesian_path(seed_start_deg, seed_end_deg, steps=50)
    for joints in waypoints:
        arm.JointCtrl(...)

    # Forward kinematics
    xyz = ik.fk(joints_deg)
"""

import math
import os
import numpy as np
import ikpy.chain
import roboticstoolbox as rtb
from spatialmath import SE3

URDF_PATH    = os.path.join(os.path.dirname(__file__), "piper.urdf")
ACTIVE_LINKS = [False, True, True, True, True, True, True, False, False]
Z_OFFSET     = 0.1357  # URDF base frame is 135.7 mm lower than arm's reported frame


class PiperIK:
    def __init__(self):
        # ikpy — used for single-point IK
        self._chain = ikpy.chain.Chain.from_urdf_file(
            URDF_PATH,
            active_links_mask=ACTIVE_LINKS,
        )
        # URDF limits are conservative — widen J5 to match physical capability
        # (arm reaches 75.86° routinely; URDF caps at 69.9°)
        for link in self._chain.links:
            if link.name == "joint5":
                link.bounds = (-math.radians(80), math.radians(80))
        # roboticstoolbox — used for Cartesian path planning
        self._robot = rtb.ERobot.URDF(URDF_PATH)
        self._gb    = next(l for l in self._robot.links if l.name == "gripper_base")

    def set_gripper_down_reference(self, joints_deg: list[float]):
        """No-op — kept for call-site compatibility. solve_down() now uses an
        analytical constraint and does not need a recorded reference."""
        del joints_deg

    _IK_XY_TOL = 0.015  # metres — FK verification tolerance in XY
    _IK_Z_TOL  = 0.010  # metres — FK verification tolerance in Z

    def solve_down(self, x: float, y: float, z: float,
                   seed_deg: list[float]) -> list[float]:
        """IK to (x, y, z) with gripper forced to point straight down.

        Constrains the tool Y-axis (the approach axis for this URDF) to point
        in the world -Z direction. Only one axis is constrained so the IK
        remains free to choose jaw rotation, giving much better convergence
        than orientation_mode='all'.

        Raises RuntimeError if FK verification of the solution misses the
        target by more than _IK_XY_TOL / _IK_Z_TOL — catches bad IK solutions
        before the arm moves.
        """
        joints = self.solve(x, y, z, seed_deg=seed_deg,
                            orientation=np.array([0.0, 0.0, -1.0]),
                            orientation_mode="Y")
        fk = self.fk(joints)
        ex, ey, ez = abs(fk[0] - x), abs(fk[1] - y), abs(fk[2] - z)
        if ex > self._IK_XY_TOL or ey > self._IK_XY_TOL or ez > self._IK_Z_TOL:
            raise RuntimeError(
                f"IK solution failed FK verification — target ({x:.4f}, {y:.4f}, {z:.4f}) "
                f"FK result ({fk[0]:.4f}, {fk[1]:.4f}, {fk[2]:.4f}) "
                f"error (x={ex*1000:.1f}mm y={ey*1000:.1f}mm z={ez*1000:.1f}mm)"
            )
        return joints

    # ------------------------------------------------------------------
    # Single-point IK (ikpy)
    # ------------------------------------------------------------------

    # URDF joint limits in radians (J1–J6) — used to clip seed before passing to ikpy
    _URDF_LIMITS = [
        (-2.6179, 2.6179),   # J1
        (0.0,     3.14),     # J2
        (-2.967,  0.0),      # J3
        (-1.745,  1.745),    # J4
        (-1.3963, 1.3963),   # J5 — widened to ±80° (physical limit > URDF's ±69.9°)
        (-2.0944, 2.0944),   # J6
    ]

    def solve(self, x: float, y: float, z: float,
              seed_deg: list[float],
              orientation: np.ndarray | None = None,
              orientation_mode: str | None = None) -> list[float]:
        """Return 6 joint angles in degrees for the target XYZ (metres)."""
        seed_rad = self._to_ikpy_rad(seed_deg)
        # Clip seed to URDF limits — recorded positions can marginally exceed them
        # (e.g. J5=69.93° vs URDF limit of 69.9°), which causes scipy to reject the guess
        for i, (lo, hi) in enumerate(self._URDF_LIMITS):
            seed_rad[i + 1] = np.clip(seed_rad[i + 1], lo, hi)
        mode = orientation_mode if orientation_mode else ("all" if orientation is not None else None)
        result   = self._chain.inverse_kinematics(
            target_position=[x, y, z - Z_OFFSET],
            target_orientation=orientation,
            orientation_mode=mode,
            initial_position=seed_rad,
        )
        return [math.degrees(result[i]) for i in range(1, 7)]

    def get_orientation(self, joints_deg: list[float]) -> np.ndarray:
        """Return the 3x3 rotation matrix of the end-effector for given joints."""
        rad = self._to_ikpy_rad(joints_deg)
        return self._chain.forward_kinematics(rad)[:3, :3]

    def fk(self, joints_deg: list[float]) -> np.ndarray:
        """Forward kinematics — return XYZ (metres) for given joint angles."""
        rad = self._to_ikpy_rad(joints_deg)
        matrix = self._chain.forward_kinematics(rad)
        xyz = matrix[:3, 3].copy()
        xyz[2] += Z_OFFSET
        return xyz

    # ------------------------------------------------------------------
    # Cartesian path planning (roboticstoolbox)
    # ------------------------------------------------------------------

    def cartesian_path(self, seed_start_deg: list[float], seed_end_deg: list[float],
                       steps: int = 50, fix_orientation: bool = True) -> list[list[float]]:
        """Return a list of joint angle arrays (degrees) forming a straight
        Cartesian line from the pose at seed_start_deg to the pose at seed_end_deg.

        fix_orientation=True (default): orientation is locked to the start pose for
        every waypoint — only position is interpolated. Guarantees gripper stays
        facing the same direction throughout the path.

        fix_orientation=False: orientation is SLERP-interpolated between start and
        end — gripper may drift if the two recorded orientations differ.

        Each step seeds the IK from the previous solution so posture stays smooth.
        Steps that fail IK silently reuse the previous solution.
        """
        q_start = [math.radians(d) for d in seed_start_deg]
        q_end   = [math.radians(d) for d in seed_end_deg]

        T_start = self._robot.fkine(q_start, end=self._gb)
        T_end   = self._robot.fkine(q_end,   end=self._gb)

        if fix_orientation:
            # Interpolate position only — freeze orientation at start pose
            R_fixed = T_start.R
            positions = np.linspace(T_start.t, T_end.t, steps)
            traj = [SE3.Rt(R_fixed, p) for p in positions]
        else:
            traj = rtb.ctraj(T_start, T_end, steps)

        q_cur = q_start
        result = []
        failures = []

        for i, T in enumerate(traj):
            sol = self._robot.ikine_LM(T, end=self._gb, q0=q_cur, joint_limits=False)
            if sol.success:
                q_cur = sol.q
            else:
                failures.append(i)
            result.append([math.degrees(r) for r in q_cur])

        if failures:
            print(f"  WARNING: IK failed at {len(failures)}/{len(traj)} waypoints: steps {failures}")
            print(f"  These steps reuse the previous solution — path may not be straight")
        return result

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _to_ikpy_rad(self, joints_deg: list[float]) -> list[float]:
        rad = [math.radians(d) for d in joints_deg]
        return [0.0] + rad + [0.0, 0.0]
