"""
Test pick and drop using joint-space control.

Sequence
--------
  hover pick → descend → grip → hover pick → hover drop → descend → drop

Manually bring the arm near HOVER_PICK first, then run this.
Pauses before each step so you can verify before continuing.

Usage
-----
  python test_pick.py
"""

import time
from robot_controller import RobotController

HOVER_PICK  = [-56.69,  95.87,  -65.56,  0.00,  65.14,  -69.42]  # above wing
PICK        = [-53.43, 103.19,  -59.47, -2.23,  54.46,  -64.22]  # at wing
HOVER_DROP  = [-94.46, 119.02, -102.84,  0.00,  75.17, -105.07]  # above drop zone
DROP        = [-94.22, 127.89, -102.62,  0.41,  69.76, -107.08]  # at drop zone

print("Pick test — CLEAR THE AREA")
print("Starting in 3 seconds (Ctrl-C to abort)")
for i in range(3, 0, -1):
    print(f"  {i}...")
    time.sleep(1.0)

with RobotController() as arm:

    print("\n[1/8] Opening gripper ...")
    arm.open_gripper()

    print("[2/8] Moving to hover above pick point ...")
    arm._move_joints_simultaneous(HOVER_PICK)

    print("[3/8] Descending to pick point ...")
    arm._move_joints_simultaneous(PICK)
    time.sleep(0.5)

    print("[4/8] Closing gripper ...")
    arm.close_gripper()
    time.sleep(0.3)
    time.sleep(0.5)

    print("[5/8] Returning to hover above pick point ...")
    arm._move_joints_simultaneous(HOVER_PICK)

    print("[6/8] Rotating base to drop zone ...")
    mid = list(HOVER_PICK)
    mid[0] = HOVER_DROP[0]
    arm._move_joints_simultaneous(mid)

    print("[6b/8] Adjusting arm to hover above drop zone ...")
    arm._move_joints_simultaneous(HOVER_DROP)

    print("[7/8] Descending to drop zone ...")
    arm._move_joints_simultaneous(DROP)
    time.sleep(0.5)

    print("[8/8] Releasing gripper ...")
    arm.open_gripper()
    time.sleep(0.2)

print("\nDone.")
