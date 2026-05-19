"""
Test straight-line Cartesian transit between hover_pick and hover_drop.

Press Enter before each step. Ctrl-C at any point to abort.

Usage
-----
  python test_pick.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from robot_controller import RobotController

# Recorded at hover height — used for the straight-line transit
SEED_HOVER_PICK = [-37.63, 110.2,  -89.78, 3.72, 75.74, -55.21]
SEED_HOVER_DROP = [-71.72, 112.77, -92.64, 0.0,  75.05, -82.57]


def step(n, total, description):
    input(f"\n[{n}/{total}] {description}\n         Press Enter to run, Ctrl-C to abort ... ")


print("Move L transit test — CLEAR THE AREA")
print("Starting in 3 seconds (Ctrl-C to abort)")
for i in range(3, 0, -1):
    print(f"  {i}...")
    time.sleep(1.0)

with RobotController() as arm:

    step(1, 2, "Move to hover_pick")
    arm.move_joints(SEED_HOVER_PICK)

    step(2, 2, "Straight-line transit to hover_drop")
    arm.move_straight_line(SEED_HOVER_DROP, steps=50)

print("\nDone.")
