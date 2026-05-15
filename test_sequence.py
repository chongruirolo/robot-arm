"""
Sequential position test.

Three modes
-----------
  python test_sequence.py          # replay from positions.csv (must exist)
  python test_sequence.py record   # record poses → save to positions.csv → replay
  python test_sequence.py record myfile   # same but use a custom CSV name

Unit note
---------
  This controller works in DEGREES (SDK raw value ÷ 1000).
  The Hackster / piper_sdk 1.0 reference code uses RADIANS from
  get_joint_states().  To convert: degrees = radians × (180 / π)
"""

import sys
import time
from robot_controller import RobotController
from robot_logger import RobotLogger
from sequence_utils import load_positions, save_positions, seq_path, SEQ_DIR

DEFAULT_CSV = seq_path("positions")
DWELL_S     = 0.8


def record_interactive(arm: RobotController) -> list[dict]:
    print("\n--- RECORD MODE ---")
    print("Put the arm into teach mode (press the physical teach button).")
    print("Press Enter to capture a position, type 'q' + Enter to finish.\n")

    positions: list[dict] = []
    count = 1
    while True:
        raw = input(f"Position {count} — press Enter to capture, 'q' to stop: ")
        if raw.strip().lower() == "q":
            break
        pos = arm.record_position()
        positions.append(pos)
        print(f"  Captured: joints={[f'{d:.1f}°' for d in pos['joints_deg']]}  "
              f"gripper={pos['gripper_mm']:.1f} mm")
        count += 1

    print(f"\nRecorded {len(positions)} position(s).")
    print("Exit teach mode on the arm now (press teach button again).\n")
    input("Press Enter when teaching mode is off and arm is locked ... ")
    return positions


def main():
    args        = [a for a in sys.argv[1:] if a != "--no-home"]
    no_home     = "--no-home" in sys.argv
    record_mode = len(args) > 0 and args[0] == "record"
    if record_mode:
        csv_path = seq_path(args[1]) if len(args) > 1 else DEFAULT_CSV
    else:
        csv_path = seq_path(args[0]) if len(args) > 0 else DEFAULT_CSV

    with RobotLogger("test_sequence") as log:
        with RobotController() as arm:

            if record_mode:
                with log.test("Record positions interactively"):
                    positions = record_interactive(arm)
                    log.check("at least one position captured",
                              len(positions) > 0,
                              expected="> 0", actual=len(positions))
                    if not positions:
                        return
                    save_positions(positions, csv_path)
                    log.info(f"saved to {csv_path}")
            else:
                with log.test(f"Load positions from {csv_path}"):
                    positions = load_positions(csv_path)
                    log.check("positions loaded",
                              len(positions) > 0,
                              expected="> 0", actual=len(positions))

            with log.test(f"Play {len(positions)}-position sequence"):
                if no_home:
                    input("Enter teach mode, guide arm to roughly near the starting position,\n"
                          "then exit teach mode. Press Enter when arm is locked and ready ... ")
                else:
                    input("Enter teach mode, guide arm to roughly near home, exit teach mode.\n"
                          "Press Enter when arm is locked ... ")
                    arm.home()
                    input("Arm moved to exact home. Press Enter to replay ... ")
                arm.play_sequence(positions, dwell_s=DWELL_S, return_home=not no_home)
                log.info(f"Sequence complete — {len(positions)} position(s) played"
                         + ("" if no_home else ", returned to home"))


if __name__ == "__main__":
    main()
