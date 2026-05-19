"""
Verify CAN is up and the arm is ready to receive commands.
Run BEFORE any motion tests.

Usage:
    python tests/test_connection.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from piper_sdk import C_PiperInterface
from robot_logger import RobotLogger


def main():
    with RobotLogger("test_connection") as log:

        with log.test("CAN port opens"):
            arm = C_PiperInterface(can_name="can0", judge_flag=False)
            arm.ConnectPort()
            log.info("ConnectPort() returned without error")


        with log.test("Arm enables"):
            arm.MasterSlaveConfig(0xFC, 0, 0, 0)
            time.sleep(0.1)
            arm.MotionCtrl_1(0x00, 0x00, 0x02)
            time.sleep(0.1)
            arm.EnableArm(7)

            deadline = time.time() + 5.0
            enabled = False
            while time.time() < deadline:
                if all(arm.GetArmEnableStatus()):
                    enabled = True
                    break
                arm.EnableArm(7)
                time.sleep(0.1)

            status = arm.GetArmEnableStatus()
            log.info(f"enable status per motor: {list(status)}")
            log.check(
                "all motors enabled within 5 s",
                enabled,
                expected="all True", actual=list(status),
            )


if __name__ == "__main__":
    main()
