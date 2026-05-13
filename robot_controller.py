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

        self._can = cfg["can_interface"]
        self._speed_pct = int(cfg["speed_pct"])
        self._approach_m = float(cfg["approach_clearance_m"])
        self._gripper_open = int(cfg["gripper_open_mm"] * 1_000)
        self._gripper_effort = int(cfg["gripper_effort_nm"] * 1_000)
        self._drop_xyz = np.array(cfg["drop_zone_xyz_m"], dtype=float)
        self._home_joints_deg = cfg["home_joints_deg"]

        # judge_flag=False: gs_usb is a third-party adapter, not AgileX's official module
        self._arm = C_PiperInterface(can_name=self._can, judge_flag=False)
        self._arm.ConnectPort()
        # Switch to slave mode so the arm responds to CAN commands
        self._arm.MasterSlaveConfig(0xFC, 0, 0, 0)
        time.sleep(0.1)
        self._arm.MotionCtrl_1(0x00, 0x00, 0x02)  # exit drag-teach mode if active
        time.sleep(0.1)
        self._arm.EnableArm(7)
        time.sleep(0.5)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def home(self):
        """Move all joints to the safe home position."""
        self._set_joint_mode()
        j = [int(d * 1_000) for d in self._home_joints_deg]
        self._arm.JointCtrl(*j)
        self._wait()

    def open_gripper(self):
        self._arm.GripperCtrl(self._gripper_open, self._gripper_effort, 0x01, 0)
        time.sleep(0.5)

    def close_gripper(self):
        self._arm.GripperCtrl(0, self._gripper_effort, 0x01, 0)
        time.sleep(0.5)

    def move_joints(self, degrees: list[float]):
        """Move to joint angles (degrees). List of 6 values."""
        if len(degrees) != 6:
            raise ValueError("Need exactly 6 joint angles")
        self._set_joint_mode()
        j = [int(d * 1_000) for d in degrees]
        self._arm.JointCtrl(*j)
        self._wait()

    def move_cartesian(self, x_m: float, y_m: float, z_m: float,
                       rx_deg: float = 0.0, ry_deg: float = 0.0, rz_deg: float = 0.0):
        """Move end-effector to pose in robot base frame. Blocking."""
        self._set_cartesian_mode()
        self._arm.EndPoseCtrl(
            int(x_m * 1_000_000),
            int(y_m * 1_000_000),
            int(z_m * 1_000_000),
            int(rx_deg * 1_000),
            int(ry_deg * 1_000),
            int(rz_deg * 1_000),
        )
        self._wait()

    def pick_and_drop(self, robot_xyz: np.ndarray):
        """Full pick sequence for one wing."""
        x, y, z = float(robot_xyz[0]), float(robot_xyz[1]), float(robot_xyz[2])
        h = self._approach_m
        dx, dy, dz = self._drop_xyz.tolist()

        self.open_gripper()
        self.move_cartesian(x, y, z + h)
        self.move_cartesian(x, y, z)
        self.close_gripper()
        time.sleep(0.3)
        self.move_cartesian(x, y, z + h)
        self.move_cartesian(dx, dy, dz + h)
        self.open_gripper()
        time.sleep(0.2)
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
        self.stop()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_joint_mode(self):
        # ctrl=CAN instruction, move=MOVE J
        self._arm.MotionCtrl_2(0x01, 0x01, self._speed_pct)

    def _set_cartesian_mode(self):
        # ctrl=CAN instruction, move=MOVE L (linear)
        self._arm.MotionCtrl_2(0x01, 0x02, self._speed_pct)

    def _wait(self, timeout: float = 15.0, poll: float = 0.05):
        """Block until arm reports motion complete (motion_status == 0)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            s = self._arm.GetArmStatus()
            if s.arm_status.motion_status == 0:
                return
            time.sleep(poll)
        raise TimeoutError("Motion did not complete within timeout")
