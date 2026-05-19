"""
Step 1: Verify CAN is up and the arm responds.
Run BEFORE any motion tests.

Usage:
    bash setup_can.sh && python3 test_connection.py
"""

import time
from piper_sdk import C_PiperInterface
from robot_logger import RobotLogger


def main():
    with RobotLogger("test_connection") as log:

        with log.test("CAN port opens"):
            arm = C_PiperInterface(can_name="can0", judge_flag=False)
            arm.ConnectPort()
            log.info("ConnectPort() returned without error")

        with log.test("Switch to slave mode"):
            arm.MasterSlaveConfig(0xFC, 0, 0, 0)
            time.sleep(0.1)
            log.info("MasterSlaveConfig sent")

        with log.test("Arm responds over CAN"):
            time.sleep(0.5)
            s  = arm.GetArmStatus()
            j  = arm.GetArmJointMsgs()
            as_ = s.arm_status
            log.info(
                f"ctrl={as_.ctrl_mode:#04x}  status={as_.arm_status:#04x}  "
                f"motion={as_.motion_status}  err={as_.err_status}"
            )
            joints = [
                j.joint_state.joint_1, j.joint_state.joint_2, j.joint_state.joint_3,
                j.joint_state.joint_4, j.joint_state.joint_5, j.joint_state.joint_6,
            ]
            log.info(
                "joints: " + "  ".join(f"J{i+1}={v/1000:.1f}°" for i, v in enumerate(joints))
            )
            log.check(
                "at least one joint non-zero",
                not all(v == 0 for v in joints),
                expected="any non-zero",
                actual=joints,
            )
            log.check(
                "ctrl mode is CAN (0x01) not teach (0x02)",
                as_.ctrl_mode == 0x01,
                expected="0x01", actual=f"{as_.ctrl_mode:#04x}",
            )


if __name__ == "__main__":
    main()
