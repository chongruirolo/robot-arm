# Piper SDK Notes

Single class: `C_PiperInterface` (singleton per CAN channel).

```python
from piper_sdk import C_PiperInterface
arm = C_PiperInterface(can_name="can0")
arm.ConnectPort()
arm.EnablePiper()
```

---

## Units (critical)

| Parameter | SDK unit | Multiply by |
|-----------|----------|-------------|
| X/Y/Z position | 0.001 mm | `metres × 1_000_000` |
| Joint angles | 0.001 deg | `degrees × 1_000` |
| Gripper position | 0.001 mm | `mm × 1_000` |
| Gripper effort | 0.001 N/m | `N·m × 1_000` |
| Speed | % (0–100) | set via `MotionCtrl_2` before motion |

---

## Startup sequence

```
ConnectPort() → EnablePiper() → MotionCtrl_2(ctrl_mode, move_mode, speed%) → motion commands
```

`MotionCtrl_2` must be called before every motion command to set speed and move mode.

---

## Connection

| Method | What it does |
|--------|-------------|
| `ConnectPort()` | Opens CAN bus, starts read thread |
| `DisconnectPort()` | Closes CAN bus |
| `isOk()` | True if CAN is receiving data |
| `GetCanFps()` | CAN message rate |

---

## Enable / Disable

| Method | What it does |
|--------|-------------|
| `EnablePiper()` | Enable all 6 joints + gripper |
| `DisablePiper()` | Disable all |
| `EnableArm(motor_num)` | Enable single motor (1–6) |
| `DisableArm(motor_num)` | Disable single motor |

---

## Motion control

| Method | What it does |
|--------|-------------|
| `MotionCtrl_2(ctrl_mode, move_mode, speed%)` | Set control mode + move type + speed. Call before every move. |
| `EndPoseCtrl(X,Y,Z,RX,RY,RZ)` | Move to Cartesian end-effector pose |
| `JointCtrl(j1,j2,j3,j4,j5,j6)` | Move all joints by angle |
| `GripperCtrl(pos, effort, speed%, ...)` | Control gripper |
| `JointMitCtrl(motor_num, ...)` | Low-level MIT torque/position control per motor |
| `MoveCAxisUpdateCtrl()` | Update circular move axis |

### `MotionCtrl_2` key params
- `ctrl_mode`: `0x01` = CAN control (normal use)
- `move_mode`: `0x00`=MOVE P, `0x01`=MOVE J, `0x02`=MOVE L, `0x03`=MOVE C
- `move_spd_rate_ctrl`: 0–100 (%)

---

## Emergency / Reset

| Method | What it does |
|--------|-------------|
| `EmergencyStop(0x01)` | Immediate stop, arm loses power |
| `EmergencyStop(0x02)` | Resume after stop |
| `ResetPiper()` | Full reset — clears errors, arm drops |

---

## Read state (feedback)

| Method | Returns |
|--------|---------|
| `GetArmStatus()` | Error codes, ctrl mode, motion status |
| `GetArmJointMsgs()` | All 6 joint angles + velocities |
| `GetArmEndPoseMsgs()` | End-effector X/Y/Z/RX/RY/RZ | R is rotation/orientation (info on end state basically)
| `GetArmGripperMsgs()` | Gripper position + effort |
| `GetArmEnableStatus()` | List of enable state per motor | enable on means Powered  on AND accept commands
| `GetArmHighSpdInfoMsgs()` | Motor current/voltage (high frequency) | used when detecting overload, diagnostic, implement safety limit, etc
| `GetArmLowSpdInfoMsgs()` | Motor temp/error (low frequency) |
| `GetMotorStates()` | Per-motor enable/error state | checking error state -> turned off for specific reasons VS motor powered off (enable state)
| `GetDriverStates()` | Driver-level diagnostics |
| `GetFK(mode)` | Forward kinematics result (`"feedback"` or `"control"`) |

---

## Config / Limits (set once, rarely changed)

| Method | What it does |
|--------|-------------|
| `SearchMotorMaxAngleSpdAccLimit(motor_num)` | Query limits for one motor |
| `SearchAllMotorMaxAngleSpd()` | Query angle+speed limits all motors |
| `MotorAngleLimitMaxSpdSet(motor_num, ...)` | Set angle limits + max speed |
| `JointConfig(motor_num, ...)` | Zero-point / direction config |
| `JointMaxAccConfig(motor_num, max_acc)` | Set max acceleration |
| `EndSpdAndAccParamSet(...)` | Set end-effector speed/acc params |
| `CrashProtectionConfig(...)` | Set collision detection sensitivity |
| `SetSDKJointLimitParam(...)` | Software joint limits enforced by SDK |
| `SetSDKGripperRangeParam(...)` | Software gripper range enforced by SDK |

---

## Teach / Trajectory mode

| Method | What it does |
|--------|-------------|
| `MotionCtrl_1(grag_teach_ctrl=0x01)` | Start drag-teach recording |
| `MotionCtrl_1(grag_teach_ctrl=0x02)` | Stop drag-teach recording |
| `MotionCtrl_1(grag_teach_ctrl=0x03)` | Replay recorded trajectory |
| `MotionCtrl_1(track_ctrl=0x01)` | Pause current trajectory |
| `MotionCtrl_1(track_ctrl=0x02)` | Resume trajectory |
| `ReqMasterArmMoveToHome(mode)` | Move master arm to home position |

---

## Misc

| Method | What it does |
|--------|-------------|
| `PiperInit()` | Software init (called internally by ConnectPort) |
| `GetCurrentSDKVersion()` | SDK version string |
| `SearchPiperFirmwareVersion()` | Request firmware version from arm |
| `GetPiperFirmwareVersion()` | Read firmware version (after Search) |
| `MasterSlaveConfig(...)` | Configure master/slave linkage mode |
| `ArmParamEnquiryAndConfig(...)` | Low-level param read/write |
