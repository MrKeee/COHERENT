#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "unitree_arm_sdk/math/mathTypes.h"
#include <vector>
#include <iostream>

namespace UNITREE_ARM {
struct LowlevelCmd{
private:
    size_t _dof = 6;
public:
    Vec6 twsit;
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<double> kp;
    std::vector<double> kd;
    Vec6 posture;

    LowlevelCmd();
    ~LowlevelCmd(){}

/* dq = {0} */
void setZeroDq();

/* tau = {0} */
void setZeroTau();

/* kp = {0} */
void setZeroKp();

/* kd = {0} */
void setZeroKd();

/*
 * set default arm kp & kd (Only used in State_LOWCMD)
 * kp = [20, 30, 30, 20, 15, 10]
 * kd = [2000, 2000, 2000, 2000, 2000, 2000]
 */
void setControlGain();

/* kp = KP, kd = KW */
void setControlGain(std::vector<double> KP, std::vector<double> KW);

/* q = qInput */
void setQ(std::vector<double> qInput);

/* q = qInput */
void setQ(VecX qInput);

/* dq = qDInput */
void setQd(VecX qDInput);

/* tau = tauInput */
void setTau(VecX tauInput);

/*
 * setZeroDq()
 * setZeroTau()
 * setZeroKp()
 * kd = [10, 10, 10, 10, 10, 10]
 */
void setPassive();

/*
 * set default gripper kp & kd (Only used in State_LOWCMD)
 * kp.at(kp.size()-1) = 20
 * kd.at(kd.size()-1) = 2000
 */
void setGripperGain();

/*
 * kp.at(kp.size()-1) = KP
 * kd.at(kd.size()-1) = KW
 */
void setGripperGain(double KP, double KW);

/* set gripper kp&kd to zero */
void setGripperZeroGain();

/* q.at(q.size()-1) = qInput; */
void setGripperQ(double qInput);
double getGripperQ() {return q.at(q.size()-1);}
void setGripperQd(double qdInput) {dq.at(dq.size()-1) = qdInput;}
double getGripperQd() {return dq.at(dq.size()-1);}
void setGripperTau(double tauInput) {tau.at(tau.size()-1) = tauInput;}
double getGripperTau() {return tau.at(tau.size()-1);}

/* return Vec6 from std::vector<double> q, without gripper */
Vec6 getQ();

/* return Vec6 from std::vector<double> dq, without gripper */
Vec6 getQd();
};

}
#endif  //LOWLEVELCMD_H
