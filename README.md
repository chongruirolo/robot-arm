# robot-arm

CV-guided pick-and-place pipeline for the AgileX Piper 6-DOF arm over CAN.

## Hardware

- AgileX Piper 6-DOF arm
- USB-CAN adapter (`can0`, 1 Mbps)
- Depth camera with YOLOv8 pose estimation

## Setup

**1. Install dependencies**

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
```

**2. Bring up CAN interface** (re-run after unplugging the adapter)

```bash
bash setup_can.sh
```

**3. Camera calibration** (one-time, or after moving the camera)

```bash
python calibrate.py
```

Touch the robot tip to ≥6 points spread across the pick workspace and enter the camera-reported XYZ for each. Saves `calibration.yaml`.

## Configuration

All tunable parameters live in [config.yaml](config.yaml):

| Key | Default | Description |
|---|---|---|
| `can_interface` | `can0` | CAN interface name |
| `speed_pct` | `10` | Move speed 0–100. Start low. |
| `approach_clearance_m` | `0.05` | Hover height above target before descending |
| `gripper_open_mm` | `70.0` | Gripper open distance |
| `gripper_contact_mm` | `30.0` | Soft-contact position before full squeeze |
| `gripper_effort_nm` | `2.0` | Grip force (0–5 N/m) |
| `descend_sink_m` | `0.004` | Extra depth below vision-estimated Z for jaw contact |
| `verify_lift_m` | `0.03` | Partial lift height for grasp verification |
| `drop_zone_xyz_m` | `[-0.007, -0.480, 0.153]` | Drop position in robot base frame |
| `home_joints_deg` | `[0, 2, 0, 0, 27, -110]` | Safe rest position |

## Running the pipeline

```bash
python pipeline.py
```

Flow: home → `sequences/approach.csv` → CV-guided pick + drop → `sequences/retreat.csv` → home

The approach/retreat sequences are fixed paths that route around the pole obstacle in the workspace.

## Recording sequences

```bash
python test_sequence.py record approach
python test_sequence.py record retreat
```

Move the arm through the path in teach mode; waypoints are saved to `sequences/`.

## Key scripts

| Script | Purpose |
|---|---|
| [pipeline.py](pipeline.py) | Full pick-and-place pipeline |
| [robot_controller.py](robot_controller.py) | Arm controller (joint, Cartesian, gripper) |
| [arm_ik.py](arm_ik.py) | IK (ikpy) and Cartesian path planning (roboticstoolbox) |
| [camera_transform.py](camera_transform.py) | Camera → robot frame coordinate transform |
| [calibrate.py](calibrate.py) | Collect point correspondences, solve R/t, save `calibration.yaml` |
| [read_pose.py](read_pose.py) | Print current end-effector XYZ in teach mode |
| [sequence_utils.py](sequence_utils.py) | Load/save joint-position sequences from CSV |
| [robot_logger.py](robot_logger.py) | Structured logging to `logs/` |
| [diagnose.py](diagnose.py) | CAN connectivity and joint-state diagnostics |
| [setup_can.sh](setup_can.sh) | Bring up `can0` at 1 Mbps |

## Tests

| Script | What it tests |
|---|---|
| [test_connection.py](test_connection.py) | CAN connect and ping |
| [test_pick.py](test_pick.py) | Single pick at a hard-coded pose |
| [test_grasp.py](test_grasp.py) | Gripper open/close cycle |
| [test_gripper.py](test_gripper.py) | Gripper force and position |
| [test_hover_above.py](test_hover_above.py) | Move to hover position above a target |
| [test_hover_transit.py](test_hover_transit.py) | Approach + retreat without grasping |
| [test_move_l.py](test_move_l.py) | Straight-line Cartesian move |
| [test_straight_line.py](test_straight_line.py) | Multi-waypoint Cartesian path |
| [test_sequence.py](test_sequence.py) | Record and replay joint sequences |

## SDK notes

See [SDK_NOTES.md](SDK_NOTES.md) for unit conventions and gotchas with `piper_sdk 0.6.1`.

Joint limits:

| Joint | Min | Max |
|---|---|---|
| J1 | −150° | 150° |
| J2 | 0° | 180° |
| J3 | −170° | 0° |
| J4 | −100° | 100° |
| J5 | −70° | 70° |
| J6 | −120° | 120° |
