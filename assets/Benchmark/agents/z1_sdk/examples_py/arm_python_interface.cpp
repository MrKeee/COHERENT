#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

class ArmInterface : public unitreeArm
{
public:
    ArmInterface(bool hasGripper):unitreeArm(hasGripper){};
    ~ArmInterface(){};
    void loopOn() { sendRecvThread->start();}
    void loopOff() { sendRecvThread->shutdown();}
    void setFsmLowcmd()
    {
        sendRecvThread->start();
        setFsm(ArmFSMState::PASSIVE);
        setFsm(ArmFSMState::LOWCMD);
        sendRecvThread->shutdown();
    }
    ArmFSMState getCurrentState()
    {
        return _ctrlComp->recvState.state;
    }
};

namespace py = pybind11;
PYBIND11_MODULE(unitree_arm_interface, m){
    using rvp = py::return_value_policy;

    m.def("postureToHomo", &postureToHomo);
    m.def("homoToPosture", &homoToPosture);

    py::enum_<ArmFSMState>(m, "ArmFSMState")
        .value("INVALID", ArmFSMState::INVALID)
        .value("PASSIVE", ArmFSMState::PASSIVE)
        .value("JOINTCTRL", ArmFSMState::JOINTCTRL)
        .value("CARTESIAN", ArmFSMState::CARTESIAN)
        .value("MOVEJ", ArmFSMState::MOVEJ)
        .value("MOVEC", ArmFSMState::MOVEC)
        .value("MOVEL", ArmFSMState::MOVEL)
        .value("TEACH", ArmFSMState::TEACH)
        .value("TEACHREPEAT", ArmFSMState::TEACHREPEAT)
        .value("TOSTATE", ArmFSMState::TOSTATE)
        .value("SAVESTATE", ArmFSMState::SAVESTATE)
        .value("TRAJECTORY", ArmFSMState::TRAJECTORY)
        .value("LOWCMD", ArmFSMState::LOWCMD)
        ;

    py::class_<LowlevelState>(m, "LowlevelState")
        .def("getQ", &LowlevelState::getQ, rvp::reference_internal)
        .def("getQd", &LowlevelState::getQd, rvp::reference_internal)
        .def("getQdd", &LowlevelState::getQdd, rvp::reference_internal)
        .def("getQTau", &LowlevelState::getTau, rvp::reference_internal)
        ;

    py::class_<CtrlComponents>(m, "CtrlComponents")
        .def_readwrite("armModel", &CtrlComponents::armModel)
        .def_readonly("dt", &CtrlComponents::dt)
        ;

    py::class_<Z1Model>(m, "Z1Model")
        .def(py::init<Vec3, double, Vec3, Mat3>())
        .def("checkInSingularity", &Z1Model::checkInSingularity)
        .def("jointProtect", [](Z1Model& self, Vec6 q, Vec6 qd){
            self.jointProtect(q, qd);
            return std::make_pair(q, qd);
        })
        .def("getJointQMax", &Z1Model::getJointQMax, rvp::reference_internal)
        .def("getJointQMin", &Z1Model::getJointQMin, rvp::reference_internal)
        .def("getJointSpeedMax", &Z1Model::getJointSpeedMax, rvp::reference_internal)
        .def("inverseKinematics", [](Z1Model& self, HomoMat Tdes, Vec6 qPast, bool checkInWorkSpace){
            Vec6 q_result;
            bool hasIK = self.inverseKinematics(Tdes, qPast, q_result, checkInWorkSpace);
            return std::make_pair(hasIK, q_result);
        })
        .def("forwardKinematics", &Z1Model::forwardKinematics)
        .def("inverseDynamics", &Z1Model::inverseDynamics)
        .def("CalcJacobian", &Z1Model::CalcJacobian)
        .def("solveQP", [](Z1Model& self, Vec6 twist, Vec6 qPast, double dt){
            Vec6 qd_result;
            self.solveQP(twist, qPast, qd_result, dt);
            return qd_result;
        })
        ;

    py::class_<ArmInterface>(m, "ArmInterface")
        .def(py::init<bool>(), py::arg("hasGripper")=true)
        .def_readwrite("q", &ArmInterface::q)
        .def_readwrite("qd", &ArmInterface::qd)
        .def_readwrite("tau", &ArmInterface::tau)
        .def_readwrite("gripperQ", &ArmInterface::gripperQ)
        .def_readwrite("gripperQd", &ArmInterface::gripperW)
        .def_readwrite("gripperTau", &ArmInterface::gripperTau)
        .def_readwrite("lowstate", &ArmInterface::lowstate)
        .def_readwrite("_ctrlComp", &ArmInterface::_ctrlComp)
        .def("setFsmLowcmd", &ArmInterface::setFsmLowcmd)
        .def("getCurrentState", &ArmInterface::getCurrentState)
        .def("loopOn", &ArmInterface::loopOn)
        .def("loopOff", &ArmInterface::loopOff)
        .def("setFsm", &ArmInterface::setFsm)
        .def("backToStart", &ArmInterface::backToStart)
        .def("labelRun", &ArmInterface::labelRun)
        .def("labelSave", &ArmInterface::labelSave)
        .def("teach", &ArmInterface::teach)
        .def("teachRepeat", &ArmInterface::teachRepeat)
        .def("calibration", &ArmInterface::calibration)
        .def("MoveJ", py::overload_cast<Vec6, double>(&ArmInterface::MoveJ))
        .def("MoveJ", py::overload_cast<Vec6, double, double>(&ArmInterface::MoveJ))
        .def("MoveL", py::overload_cast<Vec6, double>(&ArmInterface::MoveL))
        .def("MoveL", py::overload_cast<Vec6, double, double>(&ArmInterface::MoveL))
        .def("MoveC", py::overload_cast<Vec6, Vec6, double>(&ArmInterface::MoveC))
        .def("MoveC", py::overload_cast<Vec6, Vec6, double, double>(&ArmInterface::MoveC))
        .def("startTrack", &ArmInterface::startTrack)
        .def("sendRecv", &ArmInterface::sendRecv)
        .def("setWait", &ArmInterface::setWait)
        .def("jointCtrlCmd", &ArmInterface::jointCtrlCmd)
        .def("cartesianCtrlCmd", &ArmInterface::cartesianCtrlCmd)
        .def("setArmCmd", &ArmInterface::setArmCmd)
        .def("setGripperCmd", &ArmInterface::setGripperCmd)
        ;
}