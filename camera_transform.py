"""
Camera-to-robot coordinate transform.

Loads the rigid transform (R, t) solved by calibrate.py and converts
camera-frame XYZ to robot-base-frame XYZ.

Usage
-----
    from camera_transform import CameraTransform

    tf = CameraTransform()                     # loads calibration.yaml
    robot_xyz = tf.to_robot(cam_x, cam_y, cam_z)
    arm.pick_and_drop(robot_xyz)
"""

import numpy as np
import yaml
import os

CALIBRATION_PATH = os.path.join(os.path.dirname(__file__), "calibration.yaml")


class CameraTransform:
    def __init__(self, path: str = CALIBRATION_PATH):
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"No calibration file found at {path}. "
                "Run calibrate.py first to generate it."
            )
        with open(path) as f:
            data = yaml.safe_load(f)["calibration"]

        self._R = np.array(data["R_camera_to_robot"])   # 3×3
        self._t = np.array(data["t_camera_to_robot"])   # 3,
        self.residual_mm = data.get("mean_residual_mm")

    def to_robot(self, x: float, y: float, z: float) -> np.ndarray:
        """Convert a single camera-frame point to robot-base-frame (metres)."""
        cam = np.array([x, y, z], dtype=float)
        return self._R @ cam + self._t

    def to_robot_array(self, points: np.ndarray) -> np.ndarray:
        """Convert Nx3 camera-frame points to robot-base-frame (metres)."""
        return (self._R @ points.T).T + self._t
