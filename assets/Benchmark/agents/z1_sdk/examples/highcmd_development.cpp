#include "unitree_arm_sdk/control/unitreeArm.h"

int main()
{
    UNITREE_ARM::unitreeArm arm(true);
    arm.sendRecvThread->start();
    arm.backToStart();
    arm.startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);

    double duration = 1000.;
    Vec6 targetPos, lastPos;
    lastPos = arm.lowstate->getQ();
    targetPos << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;

    UNITREE_ARM::Timer timer(arm._ctrlComp->dt);
    for(int i=0; i<duration; i++)
    {
        arm.q = lastPos*(1-i/duration) + targetPos*(i/duration);
        arm.qd = (targetPos-lastPos)/(duration*arm._ctrlComp->dt);
        timer.sleep();
    }

    arm.backToStart();
    arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}