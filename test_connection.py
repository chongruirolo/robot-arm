"""
Step 1: Verify CAN is up and the arm responds.
Run BEFORE any motion tests.

Usage:
    python test_connection.py
"""

import sys
import time
from piper_sdk import C_PiperInterface


def main():
    print("Connecting to Piper on can0 ...")
    try:
        # judge_flag=False: gs_usb is a third-party adapter
        arm = C_PiperInterface(can_name="can0", judge_flag=False)
        arm.ConnectPort()
    except Exception as e:
        print(f"FAILED to open CAN: {e}")
        print("\nMake sure you ran:  bash setup_can.sh")
        sys.exit(1)

    # Switch to slave mode — arm won't respond to commands in master mode
    arm.MasterSlaveConfig(0xFC, 0, 0, 0)
    time.sleep(0.1)

    print("Connected. Reading joint angles and status for 3 seconds ...")
    time.sleep(0.5)

    for i in range(6):
        s = arm.GetArmStatus()
        j = arm.GetArmJointMsgs()
        as_ = s.arm_status
        print(
            f"  [{i}] ctrl={as_.ctrl_mode:#04x}  status={as_.arm_status:#04x}  "
            f"motion={as_.motion_status}  err={as_.err_status}"
        )
        print(f"       joints: {j.joint_state.joint_1 / 1000:.1f}° "
              f"{j.joint_state.joint_2 / 1000:.1f}° "
              f"{j.joint_state.joint_3 / 1000:.1f}° "
              f"{j.joint_state.joint_4 / 1000:.1f}° "
              f"{j.joint_state.joint_5 / 1000:.1f}° "
              f"{j.joint_state.joint_6 / 1000:.1f}°")
        time.sleep(0.5)

    arm.DisableArm(7)
    print("\nOK — arm is responding over CAN.")
    print("Next: run  python test_joints.py  to test motion.")


if __name__ == "__main__":
    main()
