#ifndef _UNITREE_ARM_ARM_COMMON_H_
#define _UNITREE_ARM_ARM_COMMON_H_

#include <stdint.h>

#pragma pack(1)

namespace UNITREE_ARM {
// 4 Byte
enum class ArmFSMState{
    INVALID,
    PASSIVE,
    JOINTCTRL,
    CARTESIAN,
    MOVEJ,
    MOVEL,
    MOVEC,
    TRAJECTORY,
    TOSTATE,
    SAVESTATE,
    TEACH,
    TEACHREPEAT,
    CALIBRATION,
    SETTRAJ,
    BACKTOSTART,
    NEXT,
    LOWCMD
};

enum class TrajType{
    MoveJ,
    MoveL,
    MoveC,
    Stop
};

// 20 Byte
struct JointCmd{
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
};

typedef struct{
    uint8_t reserved : 6 ;
    uint8_t state    : 2 ;//whether motor is connected; 0-ok, 1-disconnected, 2-CRC error
}Motor_Connected;

typedef struct{
    int8_t temperature;
    /* 0x01: phase current is too large
     * 0x02: phase leakage
     * 0x04: overheat(including the motor windings and the motor shell)
     * 0x20: jumped
     * 0x40: nothing
     */
    uint8_t error;
    Motor_Connected isConnected;
}Motor_State;

struct JointState{
    float T;
    float W;
    float Acc;
    float Pos;
    Motor_State state[2];
};

struct Posture{
    double rx;
    double ry;
    double rz;
    double x;
    double y;
    double z;
};

struct TrajCmd{
    TrajType trajType;
    Posture posture[2];
    double gripperPos;
    double maxSpeed;
    double stopTime;
    int trajOrder;
};

union ValueUnion{
    char name[10];
    JointCmd jointCmd[7];
    TrajCmd trajCmd;
};

struct SendCmd{
    uint8_t head[2];
    ArmFSMState state;
    bool track;// whether let arm track jointCmd in State_JOINTCTRL or posture[0] in State_CARTESIAN
    ValueUnion valueUnion;
};


struct RecvState{
    uint8_t head[2];
    ArmFSMState state;
    JointState jointState[7];
    Posture cartesianState;
};

constexpr int SENDCMD_LENGTH    = (sizeof(SendCmd));
constexpr int RECVSTATE_LENGTH  = (sizeof(RecvState));
constexpr int JointCmd_LENGTH   = (sizeof(JointCmd));
constexpr int JointState_LENGTH = (sizeof(JointState));

#pragma pack()
}
#endif  // _UNITREE_ARM_ARM_MSG_H_