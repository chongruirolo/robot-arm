"""
Grasp test — hardcoded pick coordinate, no vision required.

Sequence
--------
  1. Play approach.csv  → arm arrives safely in pick zone
  2. pick_and_drop at the hardcoded wing coordinate
  3. Play retreat.csv   → arm returns home safely

Usage
-----
  python test_grasp.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import numpy as np
from robot_controller import RobotController
from sequence_utils import load_positions, seq_path

PICK_X = 0.1223  # metres, robot base frame
PICK_Y = -0.2903
PICK_Z = 0.1553


def go(description: str):
    input(f"\n[NEXT] {description}\nPress Enter to continue, Ctrl-C to abort ... ")


def main():
    print("=== Grasp test ===")
    print(f"Pick point: x={PICK_X:.4f}  y={PICK_Y:.4f}  z={PICK_Z:.4f} m")
    print("Counting down — Ctrl-C to abort")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1.0)

    approach = load_positions(seq_path("approach"))
    retreat  = load_positions(seq_path("retreat"))

    safe_joints = approach[-1]["joints_deg"]  # approach endpoint = safe transit position

    with RobotController() as arm:
        go(f"Approach sequence ({len(approach)} waypoints) — arm moves to pick zone")
        arm.play_sequence(approach, return_home=False)
        print("Arm is at approach endpoint (safe position).")

        pick_z  = PICK_Z - arm._descend_sink_m
        drop_x, drop_y, drop_z = arm._drop_xyz.tolist()
        go(
            f"pick_and_drop:\n"
            f"  descend → ({PICK_X:.4f}, {PICK_Y:.4f}, {pick_z:.4f})  [Move L straight down]\n"
            f"  lift/verify, return to approach endpoint  [Move J]\n"
            f"  drop at ({drop_x:.4f}, {drop_y:.4f}, {drop_z:.4f})"
        )
        success = arm.pick_and_drop(np.array([PICK_X, PICK_Y, PICK_Z]),
                                    safe_joints=safe_joints, return_home=False)

        if not success:
            print("\nGrasp failed — returning home, skipping retreat sequence.")
            arm.home()
            return

        print("\nPick successful.")
        go(f"Retreat sequence ({len(retreat)} waypoints) — arm returns home safely")
        arm.play_sequence(retreat, return_home=False)
        print("Done.")


if __name__ == "__main__":
    main()
