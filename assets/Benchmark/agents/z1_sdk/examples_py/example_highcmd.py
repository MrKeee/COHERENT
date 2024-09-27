import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

# 1. highcmd_basic : armCtrlInJointCtrl
arm.labelRun("forward")
arm.startTrack(armState.JOINTCTRL)
jnt_speed = 1.0
for i in range(0, 1000):
    arm.jointCtrlCmd(np.array([0,0,0,-1,0,0,-1]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)

# 2. highcmd_basic : armCtrlByFSM
arm.labelRun("forward")
gripper_pos = 0.0
jnt_speed = 2.0
arm.MoveJ(np.array([0.5,0.1,0.1,0.5,-0.2,0.5]), gripper_pos, jnt_speed)
gripper_pos = -1.0
cartesian_speed = 0.5
arm.MoveL(np.array([0,0,0,0.45,-0.2,0.2]), gripper_pos, cartesian_speed)
gripper_pos = 0.0
arm.MoveC(np.array([0,0,0,0.45,0,0.4]), np.array([0,0,0,0.45,0.2,0.2]), gripper_pos, cartesian_speed)

# 3. highcmd_basic : armCtrlInCartesian
arm.labelRun("forward")
arm.startTrack(armState.CARTESIAN)
angular_vel = 0.3
linear_vel = 0.3
for i in range(0, 1000):
    arm.cartesianCtrlCmd(np.array([0,0,0,0,0,-1,-1]), angular_vel, linear_vel)
    time.sleep(arm._ctrlComp.dt)

arm.backToStart()
arm.loopOff()

