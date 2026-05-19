"""
Test hover_above() — arm hovers above given XY coordinates with gripper
pointing straight down at each stop.

Usage
-----
  python test_hover_above.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_controller import RobotController

# Recorded hover positions (joints) — gripper known to be pointing down
HOVER_PICK_JOINTS = [-46.75, 99.63,  -74.69, 0.40, 69.93, -107.93]

# XY coordinates to visit — arm maintains its starting height throughout
TEST_POINTS = [
    (0.2257, -0.2390, "above pick zone"),
    (0.2634, -0.2849, "midpoint"),
    (0.3011, -0.3307, "above drop zone"),
]

print("hover_lateral() test — arm moves to each XY at constant height, gripper should point DOWN.")
input("Press Enter to start, Ctrl-C to abort ... ")

with RobotController() as arm:
    arm.set_gripper_down_reference(HOVER_PICK_JOINTS)

    print("\n[0] Moving to hover_pick start position ...")
    arm._move_joints_simultaneous(HOVER_PICK_JOINTS)

    p0 = arm._arm.GetArmEndPoseMsgs().end_pose
    start_z = p0.Z_axis / 1_000_000
    print(f"  Start height locked: z={start_z:.4f} m — all moves will target this height.")

    for i, (x, y, label) in enumerate(TEST_POINTS):
        input(f"\n[{i+1}/{len(TEST_POINTS)}] hover_lateral to: {label}  ({x:.3f}, {y:.3f})"
              f"\n   Press Enter to run, Ctrl-C to abort ... ")
        arm.hover_lateral(x, y, seed_deg=HOVER_PICK_JOINTS)

        p = arm._arm.GetArmEndPoseMsgs().end_pose
        j = arm.get_joints_deg()
        z_err = (p.Z_axis / 1_000_000 - start_z) * 1000
        print(f"  Actual XYZ:  x={p.X_axis/1e6:.4f}  y={p.Y_axis/1e6:.4f}  z={p.Z_axis/1e6:.4f}")
        print(f"  Z error vs start: {z_err:+.1f} mm")
        print(f"  J5 = {j[4]:.2f}°")
        print(f"  >> Check visually: is the gripper pointing straight DOWN?")

print("\nDone.")
