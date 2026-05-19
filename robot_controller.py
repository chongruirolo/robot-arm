"""
AgileX Piper 6-DOF arm controller.

SDK unit conventions (verified from piper_sdk 0.6.1 source):
  EndPoseCtrl  — X/Y/Z in 0.001 mm   → metres × 1_000_000
  JointCtrl    — angles in 0.001 deg  → degrees × 1_000
  GripperCtrl  — angle in 0.001 mm   → mm × 1_000
                 effort in 0.001 N/m  → N/m × 1_000

Speed is set once via MotionCtrl_2 before each motion command.

J1: [-150°, 150°]
J2: [  0°,  180°]
J3: [-170°,   0°]
J4: [-100°, 100°]
J5: [ -70°,  70°] 
J6: [-120°, 120°]

"""

import time
import numpy as np
import yaml
from piper_sdk import C_PiperInterface
from arm_ik import PiperIK


def _load_config(path: str = "config.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)["robot"]


class RobotController:
    def __init__(self, config_path: str = "config.yaml"):
        cfg = _load_config(config_path)

        # just pulls info out from config file
        self._can = cfg["can_interface"]
        self._speed_pct = int(cfg["speed_pct"])
        self._approach_m = float(cfg["approach_clearance_m"])
        self._gripper_open = int(cfg["gripper_open_mm"] * 1_000)
        self._gripper_contact = int(cfg["gripper_contact_mm"] * 1_000)
        self._gripper_effort = int(cfg["gripper_effort_nm"] * 1_000)
        self._descend_sink_m = float(cfg["descend_sink_m"])
        self._verify_lift_m = float(cfg["verify_lift_m"])
        self._drop_xyz = np.array(cfg["drop_zone_xyz_m"], dtype=float)
        self._home_joints_deg = cfg["home_joints_deg"]

        # judge_flag=False: skip CAN port validation on init — set to False if the check causes issues on your setup
        self._arm = C_PiperInterface(can_name=self._can, judge_flag=False)
        self._arm.ConnectPort()
        
        
        # Switch to slave mode so the arm responds to CAN commands
        self._arm.MasterSlaveConfig(0xFC, 0, 0, 0) 
        time.sleep(0.1)
        self._arm.MotionCtrl_1(0x00, 0x00, 0x02)  # exit drag-teach mode if active
        time.sleep(0.1)
        self._arm.EnableArm(7)
        deadline = time.time() + 5.0
        while not all(self._arm.GetArmEnableStatus()):
            if time.time() > deadline:
                raise RuntimeError("Arm motors did not enable within 5 s — check power and CAN connection")
            self._arm.EnableArm(7)
            time.sleep(0.1)
        self._arm.MotionCtrl_2(0x01, 0x01, self._speed_pct)  # EnableArm puts arm in STANDBY — force back to CAN control
        self._ik = PiperIK()

    # ------------------------------------------------------------------
    # Public API
    # basically another layer of abstraction on top of the sdk that came with the product
    # so these are functions i can invoke next time to move the robot instead of using SDK functions
    # ------------------------------------------------------------------

    def home(self):
        """Move joints to home one at a time: wrist first, base last."""
        self._move_sequential(self._home_joints_deg)

    def open_gripper(self):
        self._arm.GripperCtrl(self._gripper_open, int(1.5 * 1_000), 0x01, 0)
        time.sleep(0.5)

    def close_gripper(self):
        # Stage 1: close to soft contact so fingers touch the wing before squeezing
        self._arm.GripperCtrl(self._gripper_contact, int(1.0 * 1_000), 0x01, 0)
        time.sleep(0.4)
        # Stage 2: full squeeze with configured effort
        self._arm.GripperCtrl(0, self._gripper_effort, 0x01, 0)
        time.sleep(0.6)

    def move_joints(self, degrees: list[float]):
        """Move to joint angles (degrees) one at a time: wrist first, base last."""
        if len(degrees) != 6:
            raise ValueError("Need exactly 6 joint angles")
        self._move_sequential(degrees)

    def move_cartesian(self, x_m: float, y_m: float, z_m: float,
                       rx_deg: float | None = None, ry_deg: float | None = None, rz_deg: float | None = None,
                       timeout: float = 15.0):
        """Move end-effector in a straight Cartesian line (Move L).

        Sends the command ONCE and waits for the firmware to complete the move.
        rx/ry/rz default to the arm's current orientation so the gripper does
        not rotate during the move — pass explicit values to change orientation.

        Only suitable for short moves (<~15 cm). For large workspace traversals
        use move_straight_line() instead.
        """
        # Default orientation = current, so the gripper stays facing the same way
        if rx_deg is None or ry_deg is None or rz_deg is None:
            p = self._arm.GetArmEndPoseMsgs().end_pose
            rx_deg = p.RX_axis / 1_000
            ry_deg = p.RY_axis / 1_000
            rz_deg = p.RZ_axis / 1_000

        self._set_cartesian_mode()
        self._send_end_pose_and_wait(x_m, y_m, z_m, rx_deg, ry_deg, rz_deg, "Move L", timeout)

    _STEP_M = 0.04  # max distance per single Move L segment (metres)

    def move_p(self, x_m: float, y_m: float, z_m: float,
               rx_deg: float | None = None, ry_deg: float | None = None, rz_deg: float | None = None,
               timeout: float = 20.0):
        """Move end-effector in a straight Cartesian line using stepped Move L.

        Breaks the path into segments of at most _STEP_M so that no single
        Move L command covers enough distance to hit joint limits mid-stroke.
        Orientation is locked to the current pose at the start of the move
        (or to the explicitly supplied rx/ry/rz) and held fixed throughout.
        """
        p = self._arm.GetArmEndPoseMsgs().end_pose
        if rx_deg is None or ry_deg is None or rz_deg is None:
            rx_deg = p.RX_axis / 1_000
            ry_deg = p.RY_axis / 1_000
            rz_deg = p.RZ_axis / 1_000

        cx = p.X_axis / 1_000_000
        cy = p.Y_axis / 1_000_000
        cz = p.Z_axis / 1_000_000

        dist = np.linalg.norm([x_m - cx, y_m - cy, z_m - cz])
        if dist < 0.001:
            return

        n = max(1, int(np.ceil(dist / self._STEP_M)))
        xs = np.linspace(cx, x_m, n + 1)[1:]
        ys = np.linspace(cy, y_m, n + 1)[1:]
        zs = np.linspace(cz, z_m, n + 1)[1:]

        self._set_cartesian_mode()  # mode switch once, not per segment
        for sx, sy, sz in zip(xs, ys, zs):
            self._send_end_pose_and_wait(sx, sy, sz, rx_deg, ry_deg, rz_deg, "Move L", timeout)

    def pick_and_drop(self, robot_xyz: np.ndarray, safe_joints: list[float],
                      return_home: bool = True) -> bool:
        """Full pick sequence for one wing.

        robot_xyz:   pick point in robot base frame (metres)
        safe_joints: joint angles of the approach endpoint — used as the safe
                     transit position before/after each pick attempt.
        return_home: if False, arm stops after drop — caller plays retreat sequence.
        Returns True if wing was picked and dropped, False on grasp failure.

        Motion plan
        -----------
        1. IK solve_down → joint angles directly above pick at approach-endpoint height
        2. move_straight_line (J1 first) → hover above pick, gripper pointing down
        3. Move L straight down → pick point
        4. Close gripper, Move L partial lift → verify
        5. move_straight_line (J1 first) back to approach endpoint
        6. IK solve_down + move_straight_line → above drop zone
        7. Move L descend → release
        """
        x, y, z = float(robot_xyz[0]), float(robot_xyz[1]), float(robot_xyz[2])
        sink     = self._descend_sink_m
        verify_h = self._verify_lift_m
        dx, dy, dz = self._drop_xyz.tolist()

        # Use approach-endpoint height as hover height — known reachable, no workspace guessing
        hover_z = self._arm.GetArmEndPoseMsgs().end_pose.Z_axis / 1_000_000

        # Pre-compute hover joints: gripper down, directly above pick point
        hover_joints = self._ik.solve_down(x, y, hover_z, seed_deg=list(safe_joints))
        print(f"  Hover position: ({x:.4f}, {y:.4f}, {hover_z:.4f})  gripper-down IK solved")

        for attempt in range(2):
            self.open_gripper()

            print(f"  [attempt {attempt + 1}] Moving to hover above pick (J1 first)")
            self.move_straight_line(hover_joints)   # J1 rotates first, then arm positions gripper-down

            print(f"  [attempt {attempt + 1}] Descending to pick point ({x:.4f}, {y:.4f}, {z - sink:.4f})")
            self.move_cartesian(x, y, z - sink)     # Move L straight down — orientation locked from hover
            if not self._reached(x, y, z - sink):
                print(f"  Grasp attempt {attempt + 1} aborted — did not reach pick position")
                self.move_straight_line(safe_joints)
                if attempt == 1:
                    self.home()
                    return False
                continue

            self.close_gripper()
            time.sleep(0.3)

            print(f"  [attempt {attempt + 1}] Partial lift to verify grasp")
            self.move_cartesian(x, y, z + verify_h) # Move L short lift to check grip
            if self._is_holding():
                print("  Grasp confirmed.")
                break

            print(f"  Grasp attempt {attempt + 1} failed — returning to approach endpoint")
            self.move_straight_line(safe_joints)
            if attempt == 1:
                print("  WARNING: Grasp failed after retry — aborting")
                self.home()
                return False

        print("  Returning to approach endpoint (J1 first)")
        self.move_straight_line(safe_joints)

        # Solve gripper-down IK above drop zone, J1 first
        seed = self._get_current_joints_deg()
        drop_hover_joints = self._ik.solve_down(dx, dy, dz + self._approach_m, seed_deg=seed)
        print(f"  Moving to above drop zone (J1 first, gripper down)")
        self.move_straight_line(drop_hover_joints)

        print(f"  Descending to drop height ({dx:.4f}, {dy:.4f}, {dz:.4f})")
        self.move_cartesian(dx, dy, dz)             # Move L descend to release height
        if not self._reached(dx, dy, dz):
            print("  WARNING: Did not reach drop height — releasing at current position")
        self.open_gripper()
        time.sleep(0.2)
        if return_home:
            self.home()
        return True

    def move_to_xyz(self, x: float, y: float, z: float,
                    seed_deg: list[float] | None = None):
        """Move gripper tip to XYZ (metres) using IK + joint-space control."""
        seed   = seed_deg if seed_deg else self._get_current_joints_deg()
        target = self._ik.solve(x, y, z, seed_deg=seed)
        self._move_joints_simultaneous(target)

    def set_gripper_down_reference(self, joints_deg: list[float]):
        """Call once with any recorded position where gripper is pointing straight down.

        After this, hover_above() will always enforce gripper-down orientation
        regardless of where the arm moves — the IK computes the correct J4/J5/J6
        automatically for each target position.

        Example:
            arm.set_gripper_down_reference(HOVER_PICK)
        """
        self._ik.set_gripper_down_reference(joints_deg)

    def hover_above(self, x: float, y: float, z: float,
                    offset_m: float = 0.20,
                    seed_deg: list[float] | None = None):
        """Move gripper to hover above (x, y, z) at z + offset_m, gripper pointing down.

        Requires set_gripper_down_reference() to have been called first.
        Uses J1-first two-phase motion for a clean sweep.
        """
        seed   = seed_deg if seed_deg else self._get_current_joints_deg()
        target = self._ik.solve_down(x, y, z + offset_m, seed_deg=seed)
        self.move_straight_line(target)

    def hover_lateral(self, x: float, y: float,
                      seed_deg: list[float] | None = None):
        """Move to (x, y) while maintaining the arm's current height, gripper pointing down.

        Reads the actual arm z before moving so height is preserved exactly,
        regardless of IK z accuracy at different positions.
        Requires set_gripper_down_reference() to have been called first.
        """
        current_z = self._arm.GetArmEndPoseMsgs().end_pose.Z_axis / 1_000_000
        seed      = seed_deg if seed_deg else self._get_current_joints_deg()
        target    = self._ik.solve_down(x, y, current_z, seed_deg=seed)
        self.move_straight_line(target)

    def move_straight_line(self, seed_end_deg: list[float]):
        """Two-phase transit to seed_end_deg.

        Phase 1: rotate base (J1) only to the target J1 angle — arm swings
                 at constant height since J2-J6 don't change.
        Phase 2: move all remaining joints simultaneously to the target.
        """
        current = self._get_current_joints_deg()
        j1_cmd = list(current)
        j1_cmd[0] = seed_end_deg[0]

        print("  Phase 1: rotating base (J1) ...")
        self._move_joint_until_reached(j1_cmd, joint_idx=0)

        print("  Phase 2: moving to final position ...")
        self._move_joints_simultaneous(seed_end_deg)

    def get_joints_deg(self) -> list[float]:
        """Return current joint angles in degrees (J1–J6)."""
        return self._get_current_joints_deg()

    def get_gripper_mm(self) -> float:
        """Return current gripper opening in mm."""
        raw = self._arm.GetArmGripperMsgs()
        return raw.gripper_state.grippers_angle / 1_000

    def is_holding(self) -> bool:
        """Return True if the gripper is closed enough to be holding a wing."""
        return self._is_holding()

    def record_position(self) -> dict:
        """Snapshot current joints (degrees) and gripper (mm)."""
        return {
            "joints_deg": self._get_current_joints_deg(),
            "gripper_mm": self.get_gripper_mm(),
        }

    def play_sequence(self, positions: list[dict], dwell_s: float = 0.5, return_home: bool = True):
        """Move through a list of recorded positions sequentially.

        Each entry: {"joints_deg": [j1..j6 in degrees], "gripper_mm": float}
        All 6 joints move simultaneously to avoid awkward intermediate poses.
        dwell_s: pause after each position reaches target.
        return_home: set False when the sequence is an approach/retreat leg
                     and the caller controls what happens next.
        """
        for n, pos in enumerate(positions):
            print(f"  [{n + 1}/{len(positions)}] Moving to position {n + 1} ...")
            self._move_joints_simultaneous(pos["joints_deg"])
            gripper_raw = int(pos["gripper_mm"] * 1_000)
            self._arm.GripperCtrl(gripper_raw, self._gripper_effort, 0x01, 0)
            time.sleep(max(dwell_s, 0.5))
        if return_home:
            print("  Returning to home ...")
            self.home()

    def get_status(self) -> dict:
        s = self._arm.GetArmStatus()
        return {
            "ctrl_mode": s.arm_status.ctrl_mode,
            "arm_status": s.arm_status.arm_status,
            "motion_status": s.arm_status.motion_status,
            "err_status": s.arm_status.err_status,
        }

    def stop(self):
        self._arm.DisableArm(7)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _is_holding(self) -> bool:
        """Gripper is closed enough to be gripping a wing (not empty)."""
        return self.get_gripper_mm() < 50.0

    def _reached(self, x_m: float, y_m: float, z_m: float, tol: float = 0.015) -> bool:
        """Return True if the end-effector is within tol metres of the target."""
        p = self._arm.GetArmEndPoseMsgs().end_pose
        return (abs(p.X_axis / 1_000_000 - x_m) < tol and
                abs(p.Y_axis / 1_000_000 - y_m) < tol and
                abs(p.Z_axis / 1_000_000 - z_m) < tol)

    def _move_joints_simultaneous(self, target_deg: list, tolerance: float = 8.0, timeout: float = 40.0):
        """Send all 6 joint targets at once and wait until all arrive.

        Warns on timeout instead of raising so the sequence continues.
        """
        deadline = time.time() + timeout
        actual   = target_deg
        while time.time() < deadline:
            self._set_joint_mode()
            self._arm.JointCtrl(*[int(d * 1_000) for d in target_deg])
            time.sleep(0.01)
            actual = self._get_current_joints_deg()
            if all(abs(actual[i] - target_deg[i]) < tolerance for i in range(6)):
                return
        # Log which joints missed and by how much, then continue
        misses = [
            f"J{i+1} target={target_deg[i]:.1f}° actual={actual[i]:.1f}° err={abs(actual[i]-target_deg[i]):.1f}°"
            for i in range(6) if abs(actual[i] - target_deg[i]) >= tolerance
        ]
        print(f"  WARNING: timeout — continuing with {len(misses)} joint(s) off target:")
        for m in misses:
            print(f"    {m}")

    def _move_sequential(self, target: list):
        """Move joints one at a time toward target: J6 → J5 → J4 → J3 → J2 → J1."""
        for i in [5, 4, 3, 2, 1, 0]:
            current = self._get_current_joints_deg()  # fresh read each joint
            if abs(current[i] - target[i]) < 1.0:
                continue
            cmd = list(current)
            cmd[i] = target[i]
            self._move_joint_until_reached(cmd, i)

    def _get_current_joints_deg(self) -> list:
        j = self._arm.GetArmJointMsgs()
        return [
            j.joint_state.joint_1 / 1_000,
            j.joint_state.joint_2 / 1_000,
            j.joint_state.joint_3 / 1_000,
            j.joint_state.joint_4 / 1_000,
            j.joint_state.joint_5 / 1_000,
            j.joint_state.joint_6 / 1_000,
        ]

    def _send_end_pose_and_wait(self, x_m: float, y_m: float, z_m: float,
                                rx_deg: float, ry_deg: float, rz_deg: float,
                                label: str, timeout: float):
        """Send EndPoseCtrl and wait for completion.

        arm_status == 4 (TARGET_POS_EXCEEDS_LIMIT) is only treated as a real
        rejection if the arm hasn't moved from its pre-command position.
        A stale status == 4 from a previous operation is ignored.
        """
        p0 = self._arm.GetArmEndPoseMsgs().end_pose
        start_x, start_y, start_z = p0.X_axis, p0.Y_axis, p0.Z_axis

        self._arm.EndPoseCtrl(
            int(x_m * 1_000_000),
            int(y_m * 1_000_000),
            int(z_m * 1_000_000),
            int(rx_deg * 1_000),
            int(ry_deg * 1_000),
            int(rz_deg * 1_000),
        )
        time.sleep(0.4)
        last_z = None
        still_count = 0
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.05)
            s = self._arm.GetArmStatus().arm_status
            p = self._arm.GetArmEndPoseMsgs().end_pose
            cur_z = p.Z_axis

            if s.arm_status == 4:
                moved = (abs(p.X_axis - start_x) > 500 or   # >0.5 mm
                         abs(p.Y_axis - start_y) > 500 or
                         abs(p.Z_axis - start_z) > 500)
                if not moved:
                    self._report_joint_limits(label, s)
                    return

            if s.motion_status == 0:
                return
            if (abs(p.X_axis / 1_000_000 - x_m) < 0.005 and
                abs(p.Y_axis / 1_000_000 - y_m) < 0.005 and
                abs(p.Z_axis / 1_000_000 - z_m) < 0.005):
                return
            if last_z is not None and abs(cur_z - last_z) < 100:
                still_count += 1
                if still_count >= 5:
                    return
            else:
                still_count = 0
            last_z = cur_z
        print(f"  WARNING: {label} did not reach target within timeout — continuing")

    def _set_joint_mode(self):
        # ctrl=CAN instruction, move=MOVE J
        self._arm.MotionCtrl_2(0x01, 0x01, self._speed_pct)

    def _set_cartesian_mode(self):
        # ctrl=CAN instruction, move=MOVE L (linear)
        self._arm.MotionCtrl_2(0x01, 0x02, self._speed_pct)
        time.sleep(0.5)  # let firmware process mode switch before EndPoseCtrl

    def _set_move_p_mode(self):
        # ctrl=CAN instruction, move=MOVE P (point-to-point, firmware plans its own path)
        self._arm.MotionCtrl_2(0x01, 0x00, self._speed_pct)
        time.sleep(0.5)

    def _report_joint_limits(self, label: str, arm_status):
        """Print which joints are at their limit when TARGET_POS_EXCEEDS_LIMIT fires."""
        at_limit = [
            f"J{i + 1}" for i, flag in enumerate([
                arm_status.err_status.joint_1_angle_limit,
                arm_status.err_status.joint_2_angle_limit,
                arm_status.err_status.joint_3_angle_limit,
                arm_status.err_status.joint_4_angle_limit,
                arm_status.err_status.joint_5_angle_limit,
                arm_status.err_status.joint_6_angle_limit,
            ]) if flag
        ]
        joints = ", ".join(at_limit) if at_limit else "unknown (err_code=0)"
        print(f"  WARNING: {label} rejected — target exceeds joint limits  [{joints} at limit]")

    def _move_joint_until_reached(self, target_deg: list, joint_idx: int, tolerance: float = 1.0, timeout: float = 15.0):
        """Keep re-sending JointCtrl until the target joint is within tolerance.

        The arm has a watchdog — it drops torque if it stops receiving commands.
        Re-sending every 10 ms keeps it active while also serving as the motion command.
        Only the joint at joint_idx is checked for completion.
        """
        deadline = time.time() + timeout
        actual = target_deg
        while time.time() < deadline:
            self._set_joint_mode()
            self._arm.JointCtrl(*[int(d * 1_000) for d in target_deg])
            time.sleep(0.01)
            actual = self._get_current_joints_deg()
            if abs(actual[joint_idx] - target_deg[joint_idx]) < tolerance:
                return
        raise TimeoutError(f"Joint {joint_idx + 1} did not reach target within {timeout} s — target {target_deg[joint_idx]:.2f}°, actual {actual[joint_idx]:.2f}°")
