"""
AgileX Piper 6-DOF arm controller.

SDK unit conventions (verified from piper_sdk 0.6.1 source):
  EndPoseCtrl  — X/Y/Z in 0.001 mm   → metres × 1_000_000
  JointCtrl    — angles in 0.001 deg  → degrees × 1_000
  GripperCtrl  — angle in 0.001 mm   → mm × 1_000
                 effort in 0.001 N/m  → N/m × 1_000

Speed is set once via MotionCtrl_2 before each motion command.
"""

import time
import numpy as np
import yaml
from piper_sdk import C_PiperInterface


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
        self._gripper_effort = int(cfg["gripper_effort_nm"] * 1_000)
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
        self._arm.GripperCtrl(0, self._gripper_effort, 0x01, 0)
        time.sleep(0.5)

    def move_joints(self, degrees: list[float]):
        """Move to joint angles (degrees) one at a time: wrist first, base last."""
        if len(degrees) != 6:
            raise ValueError("Need exactly 6 joint angles")
        self._move_sequential(degrees)

    def move_cartesian(self, x_m: float, y_m: float, z_m: float,
                       rx_deg: float = 0.0, ry_deg: float = 0.0, rz_deg: float = 0.0):
        """Move end-effector to pose in robot base frame. Blocking."""
        self._set_cartesian_mode()
        deadline = time.time() + 30.0
        while time.time() < deadline:
            self._arm.EndPoseCtrl(
                int(x_m * 1_000_000),
                int(y_m * 1_000_000),
                int(z_m * 1_000_000),
                int(rx_deg * 1_000),
                int(ry_deg * 1_000),
                int(rz_deg * 1_000),
            )
            time.sleep(0.01)
            p = self._arm.GetArmEndPoseMsgs().end_pose
            if (abs(p.X_axis / 1_000_000 - x_m) < 0.005 and
                abs(p.Y_axis / 1_000_000 - y_m) < 0.005 and
                abs(p.Z_axis / 1_000_000 - z_m) < 0.005):
                return
        print("  WARNING: cartesian move did not reach target within 30 s — continuing")

    def pick_and_drop(self, robot_xyz: np.ndarray,
                      rx_deg: float = 0.0, ry_deg: float = 0.0, rz_deg: float = 0.0,
                      return_home: bool = True):
        """Full pick sequence for one wing.

        robot_xyz:   pick point in robot base frame (metres)
        rx/ry/rz_deg: gripper orientation at pick point (degrees)
        return_home: if False, arm stops at drop zone — caller handles the retreat path
        """
        x, y, z = float(robot_xyz[0]), float(robot_xyz[1]), float(robot_xyz[2])
        h = self._approach_m
        dx, dy, dz = self._drop_xyz.tolist()

        self.open_gripper()
        self.move_cartesian(x, y, z + h, rx_deg, ry_deg, rz_deg)  # approach
        self.move_cartesian(x, y, z,     rx_deg, ry_deg, rz_deg)  # descend
        self.close_gripper()
        time.sleep(0.3)
        self.move_cartesian(x, y, z + h, rx_deg, ry_deg, rz_deg)  # lift
        self.move_cartesian(dx, dy, dz + h)                        # move to drop
        self.open_gripper()
        time.sleep(0.2)
        if return_home:
            self.home()

    def get_joints_deg(self) -> list[float]:
        """Return current joint angles in degrees (J1–J6)."""
        return self._get_current_joints_deg()

    def get_gripper_mm(self) -> float:
        """Return current gripper opening in mm."""
        raw = self._arm.GetArmGripperMsgs()
        return raw.gripper_state.grippers_angle / 1_000

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

    def _set_joint_mode(self):
        # ctrl=CAN instruction, move=MOVE J
        self._arm.MotionCtrl_2(0x01, 0x01, self._speed_pct)

    def _set_cartesian_mode(self):
        # ctrl=CAN instruction, move=MOVE L (linear)
        self._arm.MotionCtrl_2(0x01, 0x02, self._speed_pct)

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
