"""
Step 2: Safe joint-space motion test.

Moves the arm to home, then does a small joint-1 sweep (-15° → +15° → 0°).
Watch the arm — press Ctrl-C to abort at any time.

IMPORTANT: Clear the area around the arm before running.

Usage:
    python test_joints.py
"""

import sys
import time
from robot_controller import RobotController


def main():
    print("=" * 50)
    print("Joint motion test — CLEAR THE AREA NOW")
    print("Starting in 3 seconds  (Ctrl-C to abort)")
    print("=" * 50)
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1.0)

    with RobotController() as arm:
        print("\n[1/4] Moving to home position ...")
        arm.home()
        print("      Home reached.")
        time.sleep(1.0)

        print("[2/4] Rotating base joint to -15° ...")
        arm.move_joints([-15.0, 90.0, -90.0, 0.0, 0.0, 0.0])
        time.sleep(0.5)

        print("[3/4] Rotating base joint to +15° ...")
        arm.move_joints([15.0, 90.0, -90.0, 0.0, 0.0, 0.0])
        time.sleep(0.5)

        print("[4/4] Returning to home ...")
        arm.home()

    print("\nDone. Joint motion test passed.")


if __name__ == "__main__":
    main()
