#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include <vector>
#include "unitree_arm_sdk/math/mathTypes.h"
#include "unitree_arm_sdk/message/arm_common.h"

namespace UNITREE_ARM {

Vec6 PosturetoVec6(const Posture p);
Posture Vec6toPosture(const Vec6 p);

struct LowlevelState{
private:
    size_t _dof = 6;
public:
    LowlevelState();
    ~LowlevelState(){};

    Vec6 endPosture;
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> tau;

    std::vector<int> temperature;
    
    /* 0x01 : phase current is too large
     * 0x02 : phase leakage
     * 0x04 : motor winding overheat or temperature is too large
     * 0x20 : parameters jump
     * 0x40 : Ignore
     */
    std::vector<uint8_t> errorstate;

    /*
     * 0: OK
     * 1: communication between lower computer and motor disconnect once
     * 2: communication between lower computer and motor has CRC erro once
     */
    std::vector<uint8_t> isMotorConnected;

    Vec6 getQ();
    Vec6 getQd();
    Vec6 getQdd();
    Vec6 getTau();
    double getGripperQ() {return q.at(q.size()-1);}
    double getGripperQd() {return dq.at(q.size()-1);}
    double getGripperTau() {return tau.at(tau.size()-1);}
};

}
#endif  //LOWLEVELSTATE_HPP
