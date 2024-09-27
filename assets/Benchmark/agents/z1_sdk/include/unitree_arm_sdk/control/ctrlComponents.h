#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "unitree_arm_sdk/message/arm_common.h"
#include "unitree_arm_sdk/message/LowlevelCmd.h"
#include "unitree_arm_sdk/message/LowlevelState.h"
#include "unitree_arm_sdk/message/udp.h"
#include "unitree_arm_sdk/utilities/loop.h"
#include "unitree_arm_sdk/model/ArmModel.h"

namespace UNITREE_ARM {
struct CtrlComponents{
public:
    CtrlComponents();
    CtrlComponents(double deltaT, bool hasUnitreeGripper);
    ~CtrlComponents();
/*
 * Function: send udp message to z1_ctrl and receive udp message from it
 * Input:    None
 * Output:   None
 * Description: The function will call udp->send() to send datas in lowcmd to z1_ctrl 
 *              and call udp->recv() to store datas from z1_ctrl into lowstate
 */
    void sendRecv();


    LowlevelCmd *lowcmd;
    LowlevelState *lowstate;
    double dt;// default: 0.002
    SendCmd sendCmd; // udp command to control the arm
    RecvState recvState; // the arm state receive from udp
    ArmModel* armModel;
    UDPPort *udp;
};

}
#endif