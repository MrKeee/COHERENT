#ifndef ARMMODEL_H
#define ARMMODEL_H

#include <vector>
#include "unitree_arm_sdk/thirdparty/robotics.h"
#include "unitree_arm_sdk/thirdparty/quadProgpp/QuadProg++.hh"


namespace UNITREE_ARM {
using namespace robo;
class ArmModel{
public:
    ArmModel(Vec3 endPosLocal, double endEffectorMass, Vec3 endEffectorCom, Mat3 endEffectorInertia);
    ~ArmModel(){};

    
/*
 * Function: compute end effector frame (used for current spatial position calculation)
 * Inputs: q: current joint angles
 *         index: it can set as 0,1,...,6
 *           if index ==  6, then compute end efftor frame,
 *           else compute joint_i frame
 * Returns: Transfomation matrix representing the end-effector frame when the joints are
 *				at the specified coordinates
 */
HomoMat forwardKinematics(Vec6 q, int index = 6);


/*
 * Function: Computes inverse kinematics in the space frame with the irerative approach
 * Inputs: TDes: The desired end-effector configuration
 *         qPast: An initial guess and result output of joint angles that are close to
 *                satisfying TDes
 *         checkInWorkSpace: whether q_result shoule be around qPast
 *                 eaxmple: there is a postion defined by q_temp which is within the C-space 
 *                          if qPast == Vec6::Zero(), 
 *                          the function will return false while checkInWorkSpace is false,
 *                          and return true while checkInWorkSpace is true.
 *                          Normally, you can use qPast = Vec6::Zero() and checkInWorkSpace == true
 *                          to check whether q_temp has inverse kinematics sloutions
 * Returns: success: A logical value where TRUE means that the function found
 *                   a solution and FALSE means that it ran through the set
 *                   number of maximum iterations without finding a solution
 *          q_result: Joint angles that achieve T within the specified tolerances,
 */
virtual bool inverseKinematics(HomoMat TDes, Vec6 qPast, Eigen::Ref<Vec6> q_result, bool checkInWorkSpace = false);


/*
 * Function: Gives the space Jacobian
 * Inputs: q: current joint angles
 * Returns: 6x6 Spatial Jacobian
 */
Mat6 CalcJacobian(Vec6 q);


/*
 * Function: This function uses forward-backward Newton-Euler iterations to caculate inverse dynamics
 * Inputs: q: joint angles
 *         qd: joint velocities
 *         qdd: joint accelerations
 *         Ftip: Spatial force applied by the end-effector
 * Returns: required joint forces/torques
 */
Vec6 inverseDynamics(Vec6 q, Vec6 qd, Vec6 qdd, Vec6 Ftip);
virtual void solveQP(Vec6 twist, Vec6 qPast, Eigen::Ref<Vec6> qd_result, double dt) = 0;
virtual bool checkInSingularity(Vec6 q) = 0;


/*
 * Function: Limit q & qd inputs to valid values
 * Inputs: q: set in range[_jointQMin, _jointQMax]
 *         qd: set in range[-_jointSpeedMax, _jointSpeedMax]
 * Returns: None
 */
void jointProtect(Eigen::Ref<Vec6> q, Eigen::Ref<Vec6> qd);
std::vector<double> getJointQMax() {return _jointQMax;}
std::vector<double> getJointQMin() {return _jointQMin;}
std::vector<double> getJointSpeedMax() {return _jointSpeedMax;}


/*
 * Function: The load is applied to the end joint in equal proportion 
             and caculates the correspoding dynamic parameters
 * Inputs: load: unit:kg, set in z1_controller/config/config.xml
 * Returns: None
 */
void addLoad(double load);

    const size_t dof = 6;
protected:
    bool _checkAngle(Vec6 );
    void _buildModel();
    // initial parameters
    HomoMat _M; //End posture at the home position
    std::vector<HomoMat> _Mlist;// List of link frames {i} relative to {i-1} at the home position
    Vec3 _gravity;
    Mat6 _Slist;// spatial twist at home position
    std::vector<Mat6> _Glist;
    std::vector<Vec3> _jointAxis;// omega
    std::vector<Vec3> _jointPos;// p_0
    std::vector<double> _jointQMax;
    std::vector<double> _jointQMin;
    std::vector<double> _jointSpeedMax;
    Vec3 _endPosLocal; // based on mount postion
    Vec3 _endMountPosGlobal;

    std::vector<Vec3> _linkPosLocal;
    std::vector<double> _disJoint;//distance between joint
    std::vector<double> _linkMass;
    std::vector<Vec3> _linkCom; // center of mass
    std::vector<Mat3> _linkInertia;
    double _endEffectorMass;
    Vec3 _endEffectorCom;
    Mat3 _endEffectorInertia;

    double _load;
};

class Z1Model : public ArmModel{
public:
    Z1Model(Vec3 endPosLocal = Vec3::Zero(), double endEffectorMass = 0.0,
            Vec3 endEffectorCom = Vec3::Zero(), Mat3 endEffectorInertia = Mat3::Zero());
    ~Z1Model(){};


/*
 * Function: Check whether joint1 and joint5 is coaxial
 *           x5^2 + y5^2 < 0.1^2
 * Inputs: q: current joint variables
 * Returns: bool
 */
bool checkInSingularity(Vec6 q);


/*
 * Function: Computes inverse kinematics in the space frame with the analytical approach
 * Inputs: TDes: The desired end-effector configuration
 *         qPast: An initial guess and result output of joint angles that are close to
 *                satisfying TDes
 *         checkInWorkSpace: whether q_result shoule be around qPast
 *                 eaxmple: there is a postion defined by q_temp which is within the C-space 
 *                          if qPast == Vec6::Zero(), 
 *                          the function will return false while checkInWorkSpace is false,
 *                          and return true while checkInWorkSpace is true.
 *                          Normally, you can use qPast = Vec6::Zero() and checkInWorkSpace = true
 *                          to check whether q_temp has inverse kinematics sloutions
 * Returns: success: A logical value where TRUE means that the function found
 *                   a solution and FALSE means that it ran through the set
 *                   number of maximum iterations without finding a solution
 *          q_result: Joint angles that achieve T within the specified tolerances,
 */
bool inverseKinematics(HomoMat TDes, Vec6 qPast, Eigen::Ref<Vec6> q_result, bool checkInWorkSpace = false);


/*
 * Function: The function use quadprog++ to slove equation: qd = J.inverse() * twist, even if J has no inverse 
 * Inputs: twist: spatial velocity [R_dot, p_dot]
 *         qPast: current joint angles
 *         dt : compute period
 * Returns: qd_result: joint velocity that are corresponding to twist
 */
void solveQP(Vec6 twist, Vec6 qPast, Eigen::Ref<Vec6> qd_result, double dt);


private:
    void setParam_V3_6();
    double _theta2Bias;
};
}
#endif