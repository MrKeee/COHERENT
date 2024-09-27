#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

int main(int argc, char *argv[]) {
    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = true;
    unitreeArm arm(hasGripper);
    arm.sendRecvThread->start();

    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);

    std::vector<double> KP, KW;
    KP = arm._ctrlComp->lowcmd->kp;
    KW = arm._ctrlComp->lowcmd->kd;
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);
    arm.sendRecvThread->shutdown();

    Vec6 initQ = arm.lowstate->getQ();

    double duration = 1000;
    Vec6 targetQ;
    targetQ << 0, 1.5, -1, -0.54, 0, 0;
    Timer timer(arm._ctrlComp->dt);
    for(int i(0); i<duration; i++){
        arm.q = initQ * (1-i/duration) + targetQ * (i/duration);
        arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
        arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.gripperQ = -(i/duration);
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer.sleep();
    }

    arm.sendRecvThread->start();

    arm.setFsm(ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}