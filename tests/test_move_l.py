"""
Test Move L (straight Cartesian line) with the corrected one-shot approach.

Moves the arm 4 cm straight down from its current position, then back up.
The arm must already be in a safe position before running.

Usage
-----
  python test_move_l.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from robot_controller import RobotController

print("Move L test — arm will move 4 cm straight DOWN then back UP")
print("Make sure the arm is in a safe position with 4 cm clearance below.")
input("Press Enter to start, Ctrl-C to abort ... ")

with RobotController() as arm:
    # Read current position
    p = arm._arm.GetArmEndPoseMsgs().end_pose
    x  = p.X_axis  / 1_000_000
    y  = p.Y_axis  / 1_000_000
    z  = p.Z_axis  / 1_000_000
    rx = p.RX_axis / 1_000
    ry = p.RY_axis / 1_000
    rz = p.RZ_axis / 1_000
    print(f"\nCurrent pose: x={x:.4f}  y={y:.4f}  z={z:.4f}")
    print(f"              rx={rx:.2f}°  ry={ry:.2f}°  rz={rz:.2f}°")

    input("\n[1/2] Move 20 cm straight DOWN — Press Enter to run ... ")
    arm.move_cartesian(x, y, z - 0.20)
    p2 = arm._arm.GetArmEndPoseMsgs().end_pose
    print(f"  After: z={p2.Z_axis/1_000_000:.4f}m  (expected ~{z-0.04:.4f}m)")

    input("\n[2/2] Move back UP to original height — Press Enter to run ... ")
    arm.move_cartesian(x, y, z)
    p3 = arm._arm.GetArmEndPoseMsgs().end_pose
    print(f"  After: z={p3.Z_axis/1_000_000:.4f}m  (expected ~{z:.4f}m)")

print("\nDone.")
