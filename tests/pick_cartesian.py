"""
Pick sequence using only Move L (EndPoseCtrl) from the Piper SDK.

No IK solver. Reads the current Cartesian pose and steps toward targets
in Move L segments no longer than STEP_M. Gripper orientation is captured
once at init (from whatever the arm is currently holding) and held fixed
throughout — so call this after the approach sequence, when the gripper
is already pointing straight down.

Constraints
-----------
- Arm must already be near the pick zone (approach sequence played first)
- Each Move L step is ≤ STEP_M (default 0.05 m) — tune down if rejections occur
- Drop zone transit is the caller's responsibility (play retreat sequence after)
"""

import time
import numpy as np


STEP_M = 0.05  # max Cartesian distance per single Move L command (metres)


class PickCartesian:
    def __init__(self, arm):
        """
        arm: a connected RobotController instance, already in the pick zone
             with the gripper pointing straight down.
        """
        self._arm = arm

        # Capture gripper-down orientation from current pose.
        # Called after approach sequence so the arm is already perpendicular.
        p = arm._arm.GetArmEndPoseMsgs().end_pose
        self._rx = p.RX_axis / 1_000
        self._ry = p.RY_axis / 1_000
        self._rz = p.RZ_axis / 1_000
        print(f"  PickCartesian: orientation locked — rx={self._rx:.1f} ry={self._ry:.1f} rz={self._rz:.1f} deg")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pick(self, x: float, y: float, z: float) -> bool:
        """Move L pick sequence from current position to (x, y, z).

        Sequence
        --------
        1. Move to hover above pick point  (x, y, z + approach_clearance)
        2. Descend to pick point           (x, y, z - sink)
        3. Two-stage gripper close
        4. Partial lift to verify grasp    (x, y, z + verify_lift)
        5. Full lift back to hover         (x, y, z + approach_clearance)

        Returns True if gripper is holding something after the lift, False otherwise.
        Caller is responsible for moving to the drop zone (play retreat/approach
        sequences or use another method).
        """
        cfg      = self._arm
        h        = cfg._approach_m
        sink     = cfg._descend_sink_m
        verify_h = cfg._verify_lift_m

        for attempt in range(2):
            self._arm.open_gripper()

            print(f"  [attempt {attempt + 1}] Moving to hover ({x:.4f}, {y:.4f}, {z + h:.4f})")
            self._move_stepped(x, y, z + h)

            print(f"  [attempt {attempt + 1}] Descending to ({x:.4f}, {y:.4f}, {z - sink:.4f})")
            self._move_stepped(x, y, z - sink)

            self._arm.close_gripper()
            time.sleep(0.3)

            print(f"  [attempt {attempt + 1}] Partial lift to verify grasp")
            self._move_stepped(x, y, z + verify_h)

            if self._arm._is_holding():
                print("  Grasp confirmed — wing in gripper")
                break

            print(f"  Grasp attempt {attempt + 1} failed — gripper open, retrying")
            self._move_stepped(x, y, z + h)

            if attempt == 1:
                print("  WARNING: Grasp failed after retry — aborting pick")
                self._arm.home()
                return False

        print(f"  Full lift to ({x:.4f}, {y:.4f}, {z + h:.4f})")
        self._move_stepped(x, y, z + h)
        return True

    def move_to_drop(self, dx: float, dy: float, dz: float):
        """Move L transit to drop zone, descend, and release.

        Call after a successful pick(). Moves in steps so no single
        Move L exceeds STEP_M.
        """
        h = self._arm._approach_m

        print(f"  Moving to hover above drop ({dx:.4f}, {dy:.4f}, {dz + h:.4f})")
        self._move_stepped(dx, dy, dz + h)

        print(f"  Descending to drop height ({dx:.4f}, {dy:.4f}, {dz:.4f})")
        self._move_stepped(dx, dy, dz)

        self._arm.open_gripper()
        time.sleep(0.2)
        print("  Released")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _current_xyz(self) -> tuple[float, float, float]:
        p = self._arm._arm.GetArmEndPoseMsgs().end_pose
        return p.X_axis / 1_000_000, p.Y_axis / 1_000_000, p.Z_axis / 1_000_000

    def _move_stepped(self, tx: float, ty: float, tz: float):
        """Walk from current position to (tx, ty, tz) in Move L steps ≤ STEP_M."""
        cx, cy, cz = self._current_xyz()
        total = np.linalg.norm([tx - cx, ty - cy, tz - cz])
        if total < 0.001:
            return

        n_steps = max(1, int(np.ceil(total / STEP_M)))
        xs = np.linspace(cx, tx, n_steps + 1)[1:]
        ys = np.linspace(cy, ty, n_steps + 1)[1:]
        zs = np.linspace(cz, tz, n_steps + 1)[1:]

        for i, (sx, sy, sz) in enumerate(zip(xs, ys, zs)):
            self._arm.move_cartesian(sx, sy, sz, self._rx, self._ry, self._rz)
