"""
Unit test for move_straight_line().

Moves the arm 10 cm straight down from its current position, then back up.
Logs X, Y, Z at every waypoint — if the path is truly straight, X and Y
should stay constant while Z decreases by 0.10 m.

Usage
-----
  python test_straight_line.py

The arm must already be in a safe position with 10 cm clearance below.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from robot_controller import RobotController
from arm_ik import PiperIK

print("move_straight_line() unit test")
print("Arm will move 10 cm DOWN then 10 cm UP.")
print("Make sure there is 10 cm clearance below the gripper.")
input("Press Enter to start, Ctrl-C to abort ... ")

ik = PiperIK()

with RobotController() as arm:
    # --- Read start state ---
    start_joints = arm.get_joints_deg()
    start_xyz = ik.fk(start_joints)
    x0, y0, z0 = start_xyz
    print(f"\nStart joints: {[f'{j:.2f}' for j in start_joints]}")
    print(f"Start XYZ:    x={x0:.4f}  y={y0:.4f}  z={z0:.4f} m")

    # --- Compute end joints (10 cm lower) using IK, preserving current orientation ---
    print("\nSolving IK for target 10 cm below ...")
    current_orientation = ik.get_orientation(start_joints)  # 3x3 rotation matrix
    end_joints = ik.solve(x0, y0, z0 - 0.10, seed_deg=start_joints, orientation=current_orientation)
    end_xyz = ik.fk(end_joints)
    print(f"End joints:   {[f'{j:.2f}' for j in end_joints]}")
    print(f"End XYZ:      x={end_xyz[0]:.4f}  y={end_xyz[1]:.4f}  z={end_xyz[2]:.4f} m")
    print(f"Expected Z drop: {(z0 - 0.10 - end_xyz[2] + 0.10):.4f} m  (should be ~0.10)")

    # --- Preview the path before moving ---
    print("\nComputing path (no movement yet) ...")
    waypoints = ik.cartesian_path(start_joints, end_joints, steps=20)
    print(f"\n{'Step':>4}  {'X (m)':>8}  {'Y (m)':>8}  {'Z (m)':>8}  {'dX mm':>7}  {'dY mm':>7}")
    print("-" * 55)
    for i, wp in enumerate(waypoints):
        xyz = ik.fk(wp)
        dx_mm = (xyz[0] - x0) * 1000
        dy_mm = (xyz[1] - y0) * 1000
        print(f"{i:>4}  {xyz[0]:>8.4f}  {xyz[1]:>8.4f}  {xyz[2]:>8.4f}  {dx_mm:>+7.2f}  {dy_mm:>+7.2f}")

    print("\nIf dX and dY columns stay near 0.00, the path is straight.")

    input("\n[1/2] Execute DOWN move — Press Enter to run, Ctrl-C to abort ... ")
    arm.move_straight_line(end_joints, steps=20)

    actual = arm.get_joints_deg()
    actual_xyz = ik.fk(actual)
    print(f"  After: x={actual_xyz[0]:.4f}  y={actual_xyz[1]:.4f}  z={actual_xyz[2]:.4f}")
    print(f"  Z dropped: {(z0 - actual_xyz[2])*100:.1f} cm  (expected ~10.0 cm)")

    input("\n[2/2] Execute UP move back to start — Press Enter to run, Ctrl-C to abort ... ")
    arm.move_straight_line(start_joints, steps=20)

    actual2 = arm.get_joints_deg()
    actual2_xyz = ik.fk(actual2)
    print(f"  After: x={actual2_xyz[0]:.4f}  y={actual2_xyz[1]:.4f}  z={actual2_xyz[2]:.4f}")
    print(f"  Z error vs start: {abs(actual2_xyz[2] - z0)*1000:.1f} mm  (expected ~0 mm)")

print("\nDone.")
