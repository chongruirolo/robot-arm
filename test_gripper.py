"""
Step 3: Gripper open/close test.

Usage:
    python test_gripper.py
"""

import time
from robot_controller import RobotController


def main():
    print("Gripper test — connecting ...")
    with RobotController() as arm:
        print("[1/4] Moving to home ...")
        arm.home()
        time.sleep(0.5)

        print("[2/4] Opening gripper ...")
        arm.open_gripper()
        time.sleep(1.0)

        print("[3/4] Closing gripper ...")
        arm.close_gripper()
        time.sleep(1.0)

        print("[4/4] Opening gripper again ...")
        arm.open_gripper()
        time.sleep(0.5)

    print("Done. Gripper test passed.")


if __name__ == "__main__":
    main()
