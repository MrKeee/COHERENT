#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

/* This program aims to control two z1 robots
 * one named z1_ctrl1 uses UDP to communicate with robot
 * the other named z1_ctrl2 communicates with ROS simulation
 * and open main.cpp in z1_ctrl2, then change ARMSDK port to 
 * ARMSDK(..., 8074, 8073, ...)
 */

int main()
{
    //robot 1
    auto ctrlComp1 = new CtrlComponents();
    ctrlComp1->dt = 0.002;//500HZ
    ctrlComp1->udp = new UDPPort("127.0.0.1", 8071, 8072, RECVSTATE_LENGTH, BlockYN::NO, 500000);
    ctrlComp1->armModel = new Z1Model();// no UnitreeGripper
    ctrlComp1->armModel->addLoad(0.03);// add 0.03kg payload to the end joint
    auto arm1 = new unitreeArm(ctrlComp1);

    //robot 2
    auto ctrlComp2 = new CtrlComponents();
    ctrlComp2->dt = 0.002;//500HZ
    ctrlComp2->udp = new UDPPort("127.0.0.1", 8073, 8074, RECVSTATE_LENGTH, BlockYN::NO, 500000);
    ctrlComp2->armModel = new Z1Model(Vec3(0.0382, 0.0, 0.0),0.80225,
        Vec3(0.0037, 0.0014, -0.0003), Vec3(0.00057593, 0.00099960, 0.00106337).asDiagonal());// no UnitreeGripper
    ctrlComp2->armModel->addLoad(0.03);// add 0.03kg payload to the end joint
    auto arm2 = new unitreeArm(ctrlComp2);

    //switch to state_lowcmd
    arm1->sendRecvThread->start();
    arm1->setFsm(ArmFSMState::PASSIVE);
    arm1->setFsm(ArmFSMState::LOWCMD);
    arm1->sendRecvThread->shutdown();
    arm2->sendRecvThread->start();
    arm2->setFsm(ArmFSMState::PASSIVE);
    arm2->setFsm(ArmFSMState::LOWCMD);
    arm2->sendRecvThread->shutdown();


    Vec6 targetQ, lastArm1Q, lastArm2Q;
    lastArm1Q = arm1->lowstate->getQ();
    lastArm2Q = arm2->lowstate->getQ();
    targetQ << 1, 0, 0, 0, 0, 0;
    int duration = 1000;

    Timer timer(0.002);
    for(int i(0); i<duration; i++)
    {
        //robot1
        arm1->q = lastArm1Q*(1-(double)i/duration) + targetQ*((double)i/duration);
        arm1->qd = (targetQ-lastArm1Q)/(duration*0.002);
        arm1->tau.setZero();
        arm1->setArmCmd(arm1->q, arm1->qd, arm1->tau);
        arm1->setGripperCmd(arm1->gripperQ, arm1->gripperW, arm1->gripperTau);

        //robot2
        arm2->q = lastArm2Q*(1-(double)i/duration) + targetQ*((double)i/duration);
        arm2->qd = (targetQ-lastArm1Q)/(duration*0.002);
        arm2->tau.setZero();
        arm2->setArmCmd(arm2->q, arm2->qd, arm2->tau);
        arm2->setGripperCmd(arm2->gripperQ, arm2->gripperW, arm2->gripperTau);

        arm1->sendRecv();
        arm2->sendRecv();
        timer.sleep();
    }

    delete arm1;
    delete arm2;
    return 0;
}