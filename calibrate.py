"""
Camera-to-robot calibration.

Collects N point correspondences by having you:
  1. Touch the robot tip to a physical point in the workspace.
  2. Enter the XYZ that your vision system reports for that same point.

Solves for the rigid transform R, t such that:
    robot_xyz = R @ camera_xyz + t

Saves the result to calibration.yaml.

Usage
-----
    python calibrate.py

Recommended workflow
--------------------
  - Use at least 6 points spread across the full pick workspace (not collinear).
  - Include points at different heights if the vision system has depth.
  - Re-run any time the camera is physically moved or re-mounted.
  - After calibration, run calibrate.py --verify to check residuals on new points.
"""

import argparse
import math
import time
import numpy as np
import yaml
from piper_sdk import C_PiperInterface


# ──────────────────────────────────────────────────────────────────────────────
# Arm helpers (no RobotController dependency — calibration runs standalone)
# ──────────────────────────────────────────────────────────────────────────────

def connect_arm(can: str = "can0") -> C_PiperInterface:
    arm = C_PiperInterface(can_name=can, judge_flag=False)
    arm.ConnectPort()
    arm.MasterSlaveConfig(0xFC, 0, 0, 0)  # slave mode so we can read state
    time.sleep(0.2)
    return arm


def read_robot_xyz(arm: C_PiperInterface) -> np.ndarray:
    p = arm.GetArmEndPoseMsgs().end_pose
    return np.array([
        p.X_axis / 1_000_000,
        p.Y_axis / 1_000_000,
        p.Z_axis / 1_000_000,
    ])


# ──────────────────────────────────────────────────────────────────────────────
# SVD rigid-body solver
# ──────────────────────────────────────────────────────────────────────────────

def solve_rigid_transform(cam_pts: np.ndarray,
                          robot_pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (R, t) minimising ||robot_pts - (R @ cam_pts.T).T - t||.

    Uses the Umeyama / Procrustes SVD method. Guaranteed to return a proper
    rotation (det = +1), not a reflection.
    """
    assert cam_pts.shape == robot_pts.shape and cam_pts.shape[1] == 3

    cam_bar   = cam_pts.mean(axis=0)
    robot_bar = robot_pts.mean(axis=0)

    cam_c   = cam_pts   - cam_bar
    robot_c = robot_pts - robot_bar

    H = cam_c.T @ robot_c           # 3×3 cross-covariance
    U, _, Vt = np.linalg.svd(H)

    # Fix reflection (det < 0 means reflection, not rotation)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T

    t = robot_bar - R @ cam_bar
    return R, t


def residuals(R, t, cam_pts, robot_pts) -> np.ndarray:
    predicted = (R @ cam_pts.T).T + t
    return np.linalg.norm(robot_pts - predicted, axis=1)


# ──────────────────────────────────────────────────────────────────────────────
# Collection loop
# ──────────────────────────────────────────────────────────────────────────────

def prompt_camera_xyz(idx: int) -> np.ndarray:
    """Ask the user for the vision system's XYZ for this calibration point."""
    print(f"\n  Enter the camera XYZ for point {idx} (metres, space-separated).")
    print("  Example:  0.123 -0.045 0.612")
    while True:
        raw = input("  camera x y z > ").strip()
        parts = raw.replace(",", " ").split()
        if len(parts) == 3:
            try:
                return np.array([float(p) for p in parts])
            except ValueError:
                pass
        print("  Invalid — enter three numbers separated by spaces.")


def collect_points(arm: C_PiperInterface,
                   n_points: int) -> tuple[np.ndarray, np.ndarray]:
    """Interactive loop: user touches robot tip to N physical points."""
    cam_list   = []
    robot_list = []

    print(f"\nCalibration: collecting {n_points} point correspondences.")
    print("For each point:")
    print("  1. Enter teach mode and touch the robot tip to a physical target.")
    print("  2. Press Enter — the robot XYZ is read automatically.")
    print("  3. Enter the XYZ that your vision system reports for that same target.")
    print()

    for i in range(1, n_points + 1):
        input(f"[{i}/{n_points}] Touch robot tip to target, then press Enter ... ")
        robot_xyz = read_robot_xyz(arm)
        print(f"  Robot XYZ: x={robot_xyz[0]:.4f}  y={robot_xyz[1]:.4f}  z={robot_xyz[2]:.4f}  (metres)")
        cam_xyz = prompt_camera_xyz(i)
        cam_list.append(cam_xyz)
        robot_list.append(robot_xyz)
        print(f"  Point {i} saved.")

    return np.array(cam_list), np.array(robot_list)


# ──────────────────────────────────────────────────────────────────────────────
# Save / load
# ──────────────────────────────────────────────────────────────────────────────

def save_calibration(R: np.ndarray, t: np.ndarray,
                     mean_residual_mm: float,
                     path: str = "calibration.yaml"):
    data = {
        "calibration": {
            "R_camera_to_robot": R.tolist(),
            "t_camera_to_robot": t.tolist(),
            "mean_residual_mm":  round(float(mean_residual_mm), 3),
        }
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)
    print(f"\nCalibration saved to {path}")


def load_calibration(path: str = "calibration.yaml") -> tuple[np.ndarray, np.ndarray]:
    with open(path) as f:
        data = yaml.safe_load(f)["calibration"]
    return np.array(data["R_camera_to_robot"]), np.array(data["t_camera_to_robot"])


# ──────────────────────────────────────────────────────────────────────────────
# Verify mode — check residuals on new points without re-solving
# ──────────────────────────────────────────────────────────────────────────────

def verify_mode(arm: C_PiperInterface, n_points: int):
    print("\n── Verify mode ─────────────────────────────────────────────────────")
    print("Checks reprojection error on new points using the saved calibration.")
    try:
        R, t = load_calibration()
    except FileNotFoundError:
        print("ERROR: calibration.yaml not found — run calibrate.py without --verify first.")
        return

    cam_pts, robot_pts = collect_points(arm, n_points)
    errs = residuals(R, t, cam_pts, robot_pts) * 1000  # → mm

    print("\n── Residuals (mm) ──────────────────────────────────────────────────")
    for i, e in enumerate(errs):
        print(f"  Point {i+1}: {e:.2f} mm")
    print(f"  Mean:  {errs.mean():.2f} mm")
    print(f"  Max:   {errs.max():.2f} mm")
    if errs.mean() < 5.0:
        print("  OK — calibration looks good.")
    else:
        print("  WARNING — mean error > 5 mm. Consider re-calibrating with more points.")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Camera-to-robot calibration")
    parser.add_argument("--points", type=int, default=8,
                        help="Number of calibration points to collect (default: 8, minimum: 4)")
    parser.add_argument("--verify", action="store_true",
                        help="Verify existing calibration on new points instead of re-solving")
    parser.add_argument("--can", default="can0", help="CAN interface name (default: can0)")
    args = parser.parse_args()

    if args.points < 4:
        print("ERROR: need at least 4 points for a reliable calibration.")
        return

    print("Connecting to arm ...")
    arm = connect_arm(args.can)
    print("Connected.")

    if args.verify:
        verify_mode(arm, args.points)
        return

    # ── Calibration ──────────────────────────────────────────────────────────
    cam_pts, robot_pts = collect_points(arm, args.points)

    print("\nSolving rigid transform (SVD) ...")
    R, t = solve_rigid_transform(cam_pts, robot_pts)

    errs = residuals(R, t, cam_pts, robot_pts) * 1000  # → mm
    mean_err = errs.mean()

    print("\n── Results ─────────────────────────────────────────────────────────")
    print(f"  Rotation matrix R:\n{np.round(R, 6)}")
    print(f"  Translation t (m): {np.round(t, 6)}")
    print()
    print("  Per-point residuals (mm):")
    for i, e in enumerate(errs):
        flag = "  <-- large error, consider re-touching this point" if e > 10 else ""
        print(f"    Point {i+1}: {e:.2f} mm{flag}")
    print(f"  Mean: {mean_err:.2f} mm   Max: {errs.max():.2f} mm")

    if mean_err > 10.0:
        print("\n  WARNING: mean error > 10 mm — check that you entered camera coords")
        print("  in the same units (metres) and that the vision system is stable.")
        ans = input("  Save anyway? [y/N] ").strip().lower()
        if ans != "y":
            print("Calibration not saved.")
            return
    elif mean_err > 5.0:
        print("\n  CAUTION: mean error > 5 mm. Acceptable for coarse picking;")
        print("  add more points or re-touch outliers for better accuracy.")

    save_calibration(R, t, mean_err)

    # Quick sanity check: show what one point maps to
    print("\n── Sanity check ────────────────────────────────────────────────────")
    print("  Applying transform to each calibration point:")
    for i in range(len(cam_pts)):
        predicted = R @ cam_pts[i] + t
        actual    = robot_pts[i]
        print(f"  Point {i+1}: predicted={np.round(predicted,4)}  actual={np.round(actual,4)}")


if __name__ == "__main__":
    main()
