"""Read the current end-effector pose and joint angles."""

import time
from piper_sdk import C_PiperInterface

arm = C_PiperInterface(can_name="can0", judge_flag=False)
arm.ConnectPort()
arm.MasterSlaveConfig(0xFC, 0, 0, 0)
time.sleep(0.2)

input("Move arm to position. Press Enter to read pose ... ")

# End-effector XYZ + orientation
p    = arm.GetArmEndPoseMsgs().end_pose
x_m  = p.X_axis  / 1_000_000
y_m  = p.Y_axis  / 1_000_000
z_m  = p.Z_axis  / 1_000_000
rx   = p.RX_axis / 1_000
ry   = p.RY_axis / 1_000
rz   = p.RZ_axis / 1_000

# Joint angles
j    = arm.GetArmJointMsgs().joint_state
joints = [
    j.joint_1 / 1_000,
    j.joint_2 / 1_000,
    j.joint_3 / 1_000,
    j.joint_4 / 1_000,
    j.joint_5 / 1_000,
    j.joint_6 / 1_000,
]

print(f"\nEnd-effector pose:")
print(f"  x={x_m:.4f} m  y={y_m:.4f} m  z={z_m:.4f} m")
print(f"  rx={rx:.2f}°  ry={ry:.2f}°  rz={rz:.2f}°")

print(f"\nJoint angles:")
for i, deg in enumerate(joints):
    print(f"  J{i+1} = {deg:.2f}°")

print(f"\n  {[round(d, 2) for d in joints]}")
