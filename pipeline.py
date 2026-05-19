"""
CV-guided pick pipeline.

Flow
----
  home
  → approach.csv    (fixed safe path in, avoids the pole)
  → pick_and_drop() (CV-guided, orientation from YOLOv8)
  → retreat.csv     (fixed safe path back out)
  → home

Setup (one-time)
----------------
  Record the approach path:
    python test_sequence.py record approach

  Record the retreat path (reverse of approach):
    python test_sequence.py record retreat

Usage
-----
  python pipeline.py
"""

import numpy as np
from robot_controller import RobotController
from sequence_utils import load_positions, seq_path


# ---------------------------------------------------------------------------
# Stub — replace with your actual YOLOv8 + camera pipeline
# ---------------------------------------------------------------------------

def get_pick_target() -> tuple[np.ndarray, float, float, float]:
    """Return (xyz_metres, rx_deg, ry_deg, rz_deg) from CV pipeline.

    xyz is in the robot base frame.
    rx/ry/rz is the gripper orientation at the pick point.
    """
    # TODO: call your YOLOv8 model and transform pixel coords → robot base frame
    xyz   = np.array([0.30, 0.05, 0.08])   # placeholder
    rx, ry, rz = 0.0, 0.0, 0.0             # placeholder
    return xyz, rx, ry, rz


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def main():
    approach = load_positions(seq_path("approach"))
    retreat  = load_positions(seq_path("retreat"))

    xyz, rx, ry, rz = get_pick_target()
    print(f"Pick target: xyz={xyz}  rx={rx}°  ry={ry}°  rz={rz}°")

    with RobotController() as arm:
        input("\n[1/4] About to run APPROACH sequence. Clear the area. Press Enter to continue ... ")
        arm.play_sequence(approach, return_home=False)

        input("\n[2/4] Approach done. Arm is above pick zone. Press Enter to run PICK AND DROP ... ")
        arm.pick_and_drop(xyz, rx_deg=rx, ry_deg=ry, rz_deg=rz, return_home=False)

        input("\n[3/4] Pick done. Press Enter to run RETREAT sequence ... ")
        arm.play_sequence(retreat, return_home=False)

        input("\n[4/4] Retreat done. Press Enter to return to HOME ... ")
        arm.home()

        print("\n[4/4] Done.")


if __name__ == "__main__":
    main()
