import sys
sys.path.append("../lib")
import unitree_arm_interface
import numpy as np

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper = True)
armModel = arm._ctrlComp.armModel

print('--------------------------FK & IK------------------------')
q_FORWARD = np.array([0, 1.5, -1, 0.54, 0, 0])
q_near_forward = np.array([0, 1.49, -1, 0.57, 0, 0])
# 1. FK
T_forward = armModel.forwardKinematics(q_FORWARD, 6)
# 2. IK, q_result doesn't need to be near qPast
hasIK, q_forward = armModel.inverseKinematics(T_forward, np.zeros(6), True)
if hasIK:
    print("The joint angles corresponding to position FORWRAD:")
    print(q_forward)
else:
    print("no IK")
# 2. IK, q_result should to be near qPast
hasIK, q_forward = armModel.inverseKinematics(T_forward, np.zeros(6), False)
print(hasIK)
hasIK, q_forward = armModel.inverseKinematics(T_forward, q_near_forward, False)
print(hasIK, '\n')

print('--------------------------ID------------------------')
tau = armModel.inverseDynamics(np.zeros(6), np.zeros(6), np.zeros(6), np.zeros(6))
print("The torque required by the z1 arm at the homo position to resist gravity:")
print(tau, '\n')

print('--------------------------jacobian------------------------')
print("compute: V = J * qd")
J = armModel.CalcJacobian(q_FORWARD)
spatialtwist = np.array([0, 0, 0, 1., 0, 0])
qd = np.linalg.inv(J).dot(spatialtwist)
print("solved by jacobian: ", qd)
qd = armModel.solveQP(spatialtwist, q_near_forward, arm._ctrlComp.dt)
print("solved by QP:       ", qd)
