"""
Diagnostic: print full arm status, then attempt a tiny joint move
and watch what happens to motion_status in real time.
"""
import time
from piper_sdk import C_PiperInterface

arm = C_PiperInterface(can_name="can0", judge_flag=False)
arm.ConnectPort()
time.sleep(0.5)

def print_status(label):
    s = arm.GetArmStatus().arm_status
    enables = arm.GetArmEnableStatus()
    j = arm.GetArmJointMsgs().joint_state
    print(f"\n--- {label} ---")
    print(f"  ctrl_mode:     {s.ctrl_mode}")
    print(f"  arm_status:    {s.arm_status}")
    print(f"  motion_status: {s.motion_status}")
    print(f"  mode_feed:     {s.mode_feed}")
    print(f"  err_status:    {s.err_status}")
    print(f"  enables:       {enables}")
    print(f"  joints (deg):  {[round(getattr(j, f'joint_{i}')/1000, 2) for i in range(1,7)]}")

print_status("INITIAL STATE")

# Enable
arm.MasterSlaveConfig(0xFC, 0, 0, 0)
time.sleep(0.1)
arm.MotionCtrl_1(0x00, 0x00, 0x02)
time.sleep(0.1)
arm.EnableArm(7)
deadline = time.time() + 5.0
while not all(arm.GetArmEnableStatus()):
    if time.time() > deadline:
        print("ERROR: enable timeout")
        break
    arm.EnableArm(7)
    time.sleep(0.1)

print_status("AFTER ENABLE")

# Set mode
arm.MotionCtrl_2(0x01, 0x01, 50)
time.sleep(0.1)
print_status("AFTER MotionCtrl_2")

# Read current joints and command same position (no movement — just to see status response)
j = arm.GetArmJointMsgs().joint_state
current = [getattr(j, f'joint_{i}') for i in range(1, 7)]
print(f"\nSending JointCtrl with current position (no movement): {current}")
arm.JointCtrl(*current)

for i in range(20):
    time.sleep(0.1)
    s = arm.GetArmStatus().arm_status
    print(f"  t={i*0.1:.1f}s  motion_status={s.motion_status}  arm_status={s.arm_status}")
