"""
Step 3: Gripper open/close test.

Usage:
    python test_gripper.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from robot_controller import RobotController
from robot_logger import RobotLogger

OPEN_MM      = 70.0
OPEN_MIN_MM  = 50.0   # must reach at least this to count as open
CLOSE_MAX_MM =  5.0   # must reach at most this to count as closed


def main():
    with RobotLogger("test_gripper") as log:
        with RobotController() as arm:

            with log.test("Read gripper — plausible value"):
                mm = arm.get_gripper_mm()
                log.check("in range (0, 70) mm", 0 <= mm <= 70,
                          expected="(0, 70)", actual=f"{mm:.1f} mm")

            with log.test("Home position"):
                arm.home()
                time.sleep(0.5)
                log.info("home reached")

            with log.test("Open gripper"):
                arm.open_gripper()
                time.sleep(1.0)
                mm = arm.get_gripper_mm()
                log.check(f"opening >= {OPEN_MIN_MM} mm",
                          mm >= OPEN_MIN_MM,
                          expected=f">= {OPEN_MIN_MM}", actual=f"{mm:.1f} mm")

            with log.test("Close gripper"):
                arm.close_gripper()
                time.sleep(1.0)
                mm = arm.get_gripper_mm()
                log.check(f"opening <= {CLOSE_MAX_MM} mm",
                          mm <= CLOSE_MAX_MM,
                          expected=f"<= {CLOSE_MAX_MM}", actual=f"{mm:.1f} mm")

            with log.test("Re-open gripper"):
                arm.open_gripper()
                time.sleep(1.0)
                mm = arm.get_gripper_mm()
                log.check(f"opening >= {OPEN_MIN_MM} mm",
                          mm >= OPEN_MIN_MM,
                          expected=f">= {OPEN_MIN_MM}", actual=f"{mm:.1f} mm")


if __name__ == "__main__":
    main()
