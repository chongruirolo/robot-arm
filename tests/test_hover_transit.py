"""
Test straight-line transit between hover_pick and hover_drop.

Verifies:
  1. Path is a straight line (X, Y, Z all linear — no parabola)
  2. Gripper orientation stays constant (pointing down) throughout

Paste your recorded joints below, then run:
  python test_hover_transit.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from robot_controller import RobotController
from arm_ik import PiperIK

# --- Paste your recorded joints here ---
HOVER_PICK = [-46.75, 99.63,  -74.69, 0.40, 69.93, -107.93]
HOVER_DROP = [-48.26, 123.00, -109.95, 2.91, 75.86, -102.75]
# ----------------------------------------

STEPS = 30

ik = PiperIK()


def orientation_error_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """Angle (degrees) between two rotation matrices."""
    R_diff = R1.T @ R2
    cos_angle = (np.trace(R_diff) - 1) / 2
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle))


def preview_path(label: str, start_deg: list, end_deg: list):
    print(f"\n{'='*60}")
    print(f"PATH PREVIEW: {label}")
    print(f"{'='*60}")

    waypoints = ik.cartesian_path(start_deg, end_deg, steps=STEPS, fix_orientation=True)
    start_xyz = ik.fk(start_deg)
    start_R   = ik.get_orientation(start_deg)

    print(f"\n{'Step':>4}  {'X (m)':>8}  {'Y (m)':>8}  {'Z (m)':>8}  {'Orient err':>10}")
    print(f"      {'':>8}  {'dX mm':>8}  {'dY mm':>8}  {'dZ mm':>8}  {'(deg)':>10}")
    print("-" * 60)

    for i, wp in enumerate(waypoints):
        xyz = ik.fk(wp)
        R   = ik.get_orientation(wp)
        dx  = (xyz[0] - start_xyz[0]) * 1000
        dy  = (xyz[1] - start_xyz[1]) * 1000
        dz  = (xyz[2] - start_xyz[2]) * 1000
        err = orientation_error_deg(start_R, R)
        print(f"{i:>4}  {xyz[0]:>8.4f}  {xyz[1]:>8.4f}  {xyz[2]:>8.4f}  {err:>10.2f}°")
        print(f"      {'':>8}  {dx:>+8.2f}  {dy:>+8.2f}  {dz:>+8.2f}")

    end_xyz = ik.fk(waypoints[-1])
    total_dist = np.linalg.norm(np.array(end_xyz) - np.array(start_xyz))
    print(f"\nTotal distance: {total_dist*100:.1f} cm")
    print("Orient err column: degrees the gripper drifts from start orientation.")
    print("  < 1°  → orientation locked correctly ✓")
    print("  > 5°  → gripper is tilting during transit ✗")

    return waypoints


print("Hover transit test — previewing path before any movement.")
print(f"HOVER_PICK: {HOVER_PICK}")
print(f"HOVER_DROP: {HOVER_DROP}")

# Preview pick → drop
waypoints_fwd = preview_path("hover_pick → hover_drop", HOVER_PICK, HOVER_DROP)

# Preview drop → pick (return path)
waypoints_rev = preview_path("hover_drop → hover_pick", HOVER_DROP, HOVER_PICK)

input("\nPath looks good? Press Enter to execute on the arm, Ctrl-C to abort ... ")

with RobotController() as arm:
    print("\n[1/3] Moving to hover_pick (all joints simultaneously) ...")
    arm._move_joints_simultaneous(HOVER_PICK)

    input("\n[2/3] Transit hover_pick → hover_drop — Press Enter to run ... ")
    arm.move_straight_line(HOVER_DROP)

    input("\n[3/3] Return hover_drop → hover_pick — Press Enter to run ... ")
    arm.move_straight_line(HOVER_PICK)

print("\nDone.")
