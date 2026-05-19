"""
Step-by-step pick and drop test.

Positions
---------
  home          — resting joint config, always start and end here
  safe          — end of approach sequence; arm is above pick zone,
                  obstacle cleared, gripper pointing down
  pick_up       — XYZ where the gripper closes on the wing
  hover_pick_up — pick_up with Z + 9 cm; arm hovers here before descending
  drop          — XYZ where the gripper opens to release the wing
  hover_drop    — drop with Z + 9 cm; arm hovers here before descending

Motion flow
-----------
  home
  → [approach sequence] → safe
  → transit safe → hover_pick_up    (IK + joint-space, J1 first)
  → open gripper
  → descend → pick_up               (IK + joint-space)
  → close gripper
  → ascend → hover_pick_up          (IK + joint-space)
  → transit → hover_drop            (IK + joint-space, J1 first)
  → descend → drop                  (IK + joint-space)
  → open gripper
  → ascend → hover_drop             (IK + joint-space)
  → transit → safe                  (J1 first)
  → [retreat sequence] → home

Usage
-----
  python tests/test_pick.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import termios
import numpy as np
from robot_controller import RobotController
from sequence_utils import load_positions, seq_path

# ---------------------------------------------------------------------------
# Set your two points here (metres, robot base frame)
# ---------------------------------------------------------------------------
# Known-good pick point recorded 2026-05-19:
PICK_UP = np.array([0.1548, -0.1743, 0.1568])

# Known-good drop point recorded 2026-05-19:
DROP    = np.array([-0.0321, -0.4006, 0.1619])
# ---------------------------------------------------------------------------

APPROACH_CSV = seq_path("approach")
RETREAT_CSV  = seq_path("retreat")


def prompt(step: int, what: str):
    print(f"\n[{step}/13] {what}")
    termios.tcflush(sys.stdin, termios.TCIFLUSH)  # discard any buffered keystrokes
    input("  Press Enter to run, Ctrl-C to abort ... ")


def run(pick: np.ndarray, drop: np.ndarray):
    pick_x, pick_y, pick_z = float(pick[0]), float(pick[1]), float(pick[2])
    drop_x, drop_y, drop_z = float(drop[0]), float(drop[1]), float(drop[2])

    print("Pick and drop test — CLEAR THE AREA BEFORE STARTING")
    input("Press Enter to connect, Ctrl-C to abort ... ")

    with RobotController() as arm:
        try:
            hover_pick_z = pick_z + arm._approach_m   # pick_up Z + 9 cm
            hover_drop_z = drop_z + arm._approach_m   # drop Z + 9 cm

            approach = load_positions(APPROACH_CSV)
            retreat  = load_positions(RETREAT_CSV)

            print(f"\n  pick_up       : x={pick_x:.4f}  y={pick_y:.4f}  z={pick_z:.4f} m")
            print(f"  hover_pick_up : x={pick_x:.4f}  y={pick_y:.4f}  z={hover_pick_z:.4f} m")
            print(f"  drop          : x={drop_x:.4f}  y={drop_y:.4f}  z={drop_z:.4f} m")
            print(f"  hover_drop    : x={drop_x:.4f}  y={drop_y:.4f}  z={hover_drop_z:.4f} m")

            # [1] home
            prompt(1, "home")
            arm.home()

            # [2] home → safe, via approach sequence around obstacle
            prompt(2, f"approach sequence ({len(approach)} positions)  →  safe")
            arm.play_sequence(approach, return_home=False)

            # [3] safe → hover_pick_up  (IK + joint-space, J1 first)
            print("\n  Solving IK for hover_pick_up ...")
            hover_pick_joints = arm._ik.solve_down(pick_x, pick_y, hover_pick_z, seed_deg=arm._get_current_joints_deg())
            print(f"  hover_pick_up joints: {[f'{j:.1f}' for j in hover_pick_joints]}")
            prompt(3, "transit  safe → hover_pick_up  (J1 first)")
            arm.transit(hover_pick_joints)

            # [4] open gripper fully before descending
            prompt(4, "open gripper")
            arm.open_gripper()

            # [5] hover_pick_up → pick_up  (IK + joint-space, gripper pointing down)
            prompt(5, f"descend  hover_pick_up → pick_up  z: {hover_pick_z:.4f} → {pick_z - arm._descend_sink_m:.4f} m")
            arm.move_vertical(pick_x, pick_y, pick_z - arm._descend_sink_m)

            # [6] grip the wing
            prompt(6, "close gripper  (soft contact → full squeeze)")
            arm.close_gripper()

            # [7] pick_up → hover_pick_up  (IK + joint-space, gripper pointing down)
            prompt(7, f"ascend  pick_up → hover_pick_up  z: {pick_z:.4f} → {hover_pick_z:.4f} m")
            arm.move_vertical(pick_x, pick_y, hover_pick_z)

            # [8] hover_pick_up → hover_drop  (IK + joint-space, J1 first)
            print("\n  Solving IK for hover_drop ...")
            hover_drop_joints = arm._ik.solve_down(drop_x, drop_y, hover_drop_z, seed_deg=arm._get_current_joints_deg())
            print(f"  hover_drop joints: {[f'{j:.1f}' for j in hover_drop_joints]}")
            prompt(8, "transit  hover_pick_up → hover_drop  (J1 first)")
            arm.transit(hover_drop_joints)

            # [9] hover_drop → drop  (IK + joint-space, gripper pointing down)
            prompt(9, f"descend  hover_drop → drop  z: {hover_drop_z:.4f} → {drop_z - arm._descend_sink_m:.4f} m")
            arm.move_vertical(drop_x, drop_y, drop_z - arm._descend_sink_m)

            # [10] release wing
            prompt(10, "open gripper  — release wing")
            arm.open_gripper()

            # [11] drop → hover_drop  (IK + joint-space — clear table before lateral move)
            prompt(11, f"ascend  drop → hover_drop  z: {drop_z:.4f} → {hover_drop_z:.4f} m")
            arm.move_vertical(drop_x, drop_y, hover_drop_z)

            # [12] hover_drop → safe  (all joints simultaneously)
            safe_joints = approach[-1]["joints_deg"]
            print(f"\n  Current joints : {[f'{j:.1f}' for j in arm._get_current_joints_deg()]}")
            print(f"  Safe joints    : {[f'{j:.1f}' for j in safe_joints]}")
            prompt(12, "transit  hover_drop → safe  (all joints)")
            arm._move_joints_simultaneous(safe_joints)

            # [13] safe → home via retreat sequence around obstacle
            prompt(13, f"retreat sequence ({len(retreat)} positions)  →  home")
            arm.play_sequence(retreat, return_home=True)

        except KeyboardInterrupt:
            print("\n\nAborted — arm holding current position.")

    print("\nComplete.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Pick and drop test")
    parser.add_argument("--pick", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="Pick-up point in metres, e.g. --pick 0.1548 -0.1743 0.1568")
    parser.add_argument("--drop", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="Drop point in metres,    e.g. --drop -0.0321 -0.4006 0.1619")
    args = parser.parse_args()

    pick = np.array(args.pick) if args.pick else PICK_UP
    drop = np.array(args.drop) if args.drop else DROP
    run(pick, drop)
