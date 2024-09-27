# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import omni
import omni.kit.commands
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path, define_prim
from omni.isaac.sensor import _sensor

from omni.isaac.core.utils.stage import get_current_stage, get_stage_units


from omni.isaac.core.robots.robot import Robot
import omni.isaac.dynamic_control._dynamic_control as omni_dc

from omni.isaac.quadrotor.controller import CFController
from omni.isaac.quadrotor.classutils import CFState, CFMeasurement, CFCommand

from pxr import Gf
from typing import Optional
import numpy as np
from collections import deque
import carb
from typing import Optional, List
from omni.isaac.core.prims import RigidPrimView

from copy import deepcopy
from scipy.spatial.transform import Rotation as R
import numpy as np

from omni.isaac.quadruped.utils.rot_utils import get_xyz_euler_from_quaternion


DOF_DRIVE_MODE = {
    "force": int(omni_dc.DriveMode.DRIVE_FORCE),
    "acceleration": int(omni_dc.DriveMode.DRIVE_ACCELERATION),
}
"""Mapping from drive mode names to  drive mode in DC Toolbox type."""

DOF_CONTROL_MODE = {"position": 0, "velocity": 1, "effort": 2}
"""Mapping between control modes to integers."""


CONTACT_THRESH = 5

INCLINATION = 15

class Crazyflie(Robot):
    """Generic Quadruped Class"""

    def __init__(
        self,
        prim_path: str,
        name: str = "quadruped",
        physics_dt: Optional[float] = 1 / 400.0,
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """initialize robot, set up sensors and controller
        
        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot
        
        """
        self.physics_dt = physics_dt


        self._stage = get_current_stage()
        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)

        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets server")
                asset_path = assets_root_path + "/Isaac/Robots/Crazyflie/cf2x.usd"

                carb.log_warn("asset path is: " + asset_path)
                prim.GetReferences().AddReference(asset_path)

        scale = [3, 3, 3]
        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale, articulation_controller=None
        )

        

        self.meters_per_unit = get_stage_units()

        self._measurement = CFMeasurement()
        self._command = CFCommand()
        self._state = CFState()


        self._prev_fly_flag = None
        self._fly_flag = 0
        self._fly_flag_dict = {0: "hover", 1: "takeoff", 2: "landing", 3: "forward", 4: "backward", 5: "left", 6: "right", 
                               7: "clockwise", 8: "counterclockwise"}
        self.move_velocity = [1.0, 1.0, 1.0, 1.0]
        self.static_velocity = [0.0, 0.0, 0.0, 0.0]



        # contact sensor setup
        self._cs = _sensor.acquire_contact_sensor_interface()


        self.body_path = self._prim_path + "/body"
        add_contact_sensor, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/sensor",
            parent=self.body_path,
            min_threshold=0,
            max_threshold=1000000,
            color=(1, 0, 0, 1),
            radius=0.03,
            sensor_period=physics_dt,
            visualize=False,
        )

        if not add_contact_sensor:
            carb.log_error(self.body_path + " contact sensor not added")

        self.contact_force = 0
        self.contact_bool = False
        self.enable_contact_filter = True
        self._FILTER_WINDOW_SIZE = 20
        self._contact_filter = deque()

        # imu sensor setup
        self._is = _sensor.acquire_imu_sensor_interface()
        self.imu_path = self._prim_path + "/imu_link"

        add_imu_sensor, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor",
            parent=self.imu_path,
            sensor_period=physics_dt,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=False,
        )

        if not add_imu_sensor:
            carb.log_error("failed to add IMU sensor")

        self._init_position = np.array(position) if position is not None else np.array([0.0, 0.0, 0.0])
        self._init_orientation = np.array(orientation) if orientation is not None else np.array([0.0, 0.0, 0.0, 1.0])


        self.base_lin = np.zeros(3)
        self.ang_vel = np.zeros(3)

        
    @property
    def fly_state(self):
        return self._fly_flag

    @property
    def fly_state_name(self):
        return self._fly_flag_dict[self._fly_flag]
    
    @fly_state.setter
    def fly_state(self, _fly_state):
        self._fly_flag = _fly_state

    #########################
    ## initialize & post_reset
    def initialize(self, physics_sim_view=None) -> None:
        """[summary]

        initialize dc interface, set up drive mode and initial robot state
        """
        super().initialize(physics_sim_view=physics_sim_view)


        dc_order = self.dof_names

        qp_order = self.joint_names
        print(dc_order)
        print(qp_order)
        assert dc_order == qp_order, "Inconsistent"
        
        

        self.set_dof_drive_mode(drive="force")


        # kp: stiffness, kd: damping
        self.set_dof_control(control="effort", kp=0.0, kd=0.0, drive="force")
        
        self.set_state(self.default_state)
        return
    
    def post_reset(self) -> None:
        """[summary]

        post reset articulation and controller
        """
        super().post_reset()
        self.set_state(self.default_state)
        return


    #############################
    def check_dc_interface(self) -> None:
        """[summary]
        
        Checks the DC interface handle of the robot
        
        Raises:
            RuntimeError: When the DC Toolbox interface has not been configured. 
        """
        if self._handle == omni_dc.INVALID_HANDLE or self._handle is None:
            raise RuntimeError(f"Failed to obtain articulation handle at: '{self._prim_path}'")
        return True

    def set_dof_drive_mode(self, drive) -> None:
        """[summary]
        
        Set drive mode of the quadruped to force or acceleration
        
        Args:
            drive {List[str]} -- drive mode of the robot, can be either "force" or "acceleration"

        """
        self.check_dc_interface()
        dof_props = self._dc_interface.get_articulation_dof_properties(self._handle)
        if not isinstance(drive, list):
            drive = [drive] * self.num_dof
        if not len(drive) == self.num_dof:
            msg = f"Insufficient number of DOF drive modes specified. Expected: {self.num_dof}. Received: {len(drive)}."
            carb.log_error(msg)
        for index, drive_mode in enumerate(drive):
            # set drive mode
            try:
                dof_props["driveMode"][index] = DOF_DRIVE_MODE[drive_mode]
            except AttributeError:
                msg = f"Invalid articulation drive mode '{drive_mode}'. Supported drive types: {DOF_DRIVE_MODE.keys()}"
                raise ValueError(msg)
        # Set the properties into simulator
        self._dc_interface.set_articulation_dof_properties(self._handle, dof_props)

    def set_dof_control(self, control, kp, kd, drive) -> None:
        """[summary]
        
        Set dof control to position, velocity or effort
        
        Args:
            control {int or List[int]}: DOF control mode, can be  {"position": 0, "velocity": 1, "effort": 2}
            kp {float or List[float]}: proportional constant
            kd {float or List[float]}: derivative constant
            drive {int or List[int]}: DOF drive mode, can be "force": int(omni_dc.DriveMode.DRIVE_FORCE) or "acceleration": int(omni_dc.DriveMode.DRIVE_ACCELERATION)
        """
        self.check_dc_interface()
        # Extend to list if values provided
        if not isinstance(control, list):
            control = [control] * self.num_dof
        if not isinstance(kp, list):
            kp = [kp] * self.num_dof
        if not isinstance(kd, list):
            kd = [kd] * self.num_dof

        # Check that lists are of the correct size
        if not len(control) == self.num_dof:
            msg = f"Insufficient number of DOF control modes specified. Expected: {self.num_dof}. Received: {len(control)}."
            raise ValueError(msg)
        if not len(kp) == self.num_dof:
            msg = f"Insufficient number of DOF stiffness specified. Expected: {self.num_dof}. Received: {len(kp)}."
            raise ValueError(msg)
        if not len(kd) == self.num_dof:
            msg = f"Insufficient number of DOF damping specified. Expected: {self.num_dof}. Received: {len(kd)}."
            raise ValueError(msg)

        dof_props = self._dc_interface.get_articulation_dof_properties(self._handle)
        for index, (control_mode, stiffness, damping) in enumerate(zip(control, kp, kd)):
            # set control mode
            try:
                control_value = DOF_CONTROL_MODE[control_mode]
                self._dof_control_modes.append(control_value)
            except AttributeError:
                msg = f"Invalid articulation control mode '{control_mode}'. Supported control types: {DOF_CONTROL_MODE.keys()}"
                raise ValueError(msg)

            # set drive mode
            dof_props["driveMode"][index] = DOF_DRIVE_MODE[drive]
            # set the gains
            if stiffness is not None:
                dof_props["stiffness"][index] = stiffness
            if damping is not None:
                dof_props["damping"][index] = damping

        # Set the properties into simulator
        self._dc_interface.set_articulation_dof_properties(self._handle, dof_props)
        return

    def set_state(self, state: CFState) -> None:
        """[Summary]
        
        Set the kinematic state of the robot.

        Args:
            state {CFState} -- The state of the robot to set.

        Raises:
            RuntimeError: When the DC Toolbox interface has not been configured.
        """
        # self._dc_interface: dynamic control, 
        self.check_dc_interface()

        # set base state
        # quat: scalar-back
        base_pose = omni_dc.Transform(state.base_frame.pos, state.base_frame.quat)
        self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, state.base_frame.lin_vel)
        self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, state.base_frame.ang_vel)
        # cast joint state to numpy float32
        dof_state = self._dc_interface.get_articulation_dof_states(self._handle, omni_dc.STATE_ALL)
        
        # we convert controller order to DC order for setting state
        dc_joint_pos = state.joint_pos
        dc_joint_vel = state.joint_vel
        # for (name, pos) in zip(self.dof_names, dc_joint_pos):
        #     print("%s: %.2f" % (name, pos))
        dof_state["pos"] = np.asarray(dc_joint_pos, dtype=np.float32)
        dof_state["vel"] = np.asarray(dc_joint_vel, dtype=np.float32)
        dof_state["effort"] = 0.0
        # set joint state
        status = self._dc_interface.set_articulation_dof_states(self._handle, dof_state, omni_dc.STATE_ALL)
        if not status:
            raise RuntimeError("Unable to set the DOF state properly.")

    def get_state(self) -> CFState:        
        self.check_dc_interface()
        self.update()

        return deepcopy(self._state)



    def set_body_pose_from_state(self, state):# self._dc_interface: dynamic control, 
        self.check_dc_interface()

        # set base state
    
        base_pose = omni_dc.Transform( state.base_frame.pos, state.base_frame.quat)
        self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, state.base_frame.lin_vel)
        self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, state.base_frame.ang_vel)


    def update_contact_sensor_data(self) -> None:
        """[summary]
        
        Updates processed contact sensor data from the robot body, store them in member variable contact
        """
        
        reading = self._cs.get_sensor_sim_reading(self.body_path + "/sensor")
        if reading.value is None:
            carb.log_warn("reading missing from body contact sensor")

        if self.enable_contact_filter:
            self._contact_filter.append(float(reading.value) * self.meters_per_unit)
            if len(self._contact_filter) > self._FILTER_WINDOW_SIZE:
                self._contact_filter.popleft()
            self.contact_force = np.mean(self._contact_filter)

        else:
            self.contact_force = float(reading.value) * self.meters_per_unit

        self.contact_bool = self.contact_force > CONTACT_THRESH

    def update_imu_sensor_data(self) -> None:

        """[summary]
        
        Updates processed imu sensor data from the robot body, store them in member variable base_lin and ang_vel
        """
        reading = self._is.get_sensor_readings(self.imu_path + "/imu_sensor")
        if reading.shape[0]:
            # linear acceleration
            self.base_lin[0] = float(reading[-1]["lin_acc_x"]) * self.meters_per_unit
            self.base_lin[1] = float(reading[-1]["lin_acc_y"]) * self.meters_per_unit
            self.base_lin[2] = float(reading[-1]["lin_acc_z"]) * self.meters_per_unit

            # angular velocity
            self.ang_vel[0] = float(reading[-1]["ang_vel_x"])
            self.ang_vel[1] = float(reading[-1]["ang_vel_y"])
            self.ang_vel[2] = float(reading[-1]["ang_vel_z"])
        else:
            self.base_lin = np.zeros(3)
            self.ang_vel = np.zeros(3)
        return

    def update(self) -> None:
        """[summary]
        
        update robot sensor variables, state variables in A1ArmMeasurement
        """
        self.update_contact_sensor_data()
        self.update_imu_sensor_data()

        # joint pos and vel from the DC interface
        self.joint_state = super().get_joints_state()

        self._state.joint_pos = np.asarray(self.joint_state.positions, dtype=np.float32)
        self._state.joint_vel = np.asarray(self.joint_state.velocities, dtype=np.float32)

        if self._root_handle == omni_dc.INVALID_HANDLE:
            raise RuntimeError(f"Failed to obtain articulation handle at: '{self._prim_path}'")

        # base frame
        base_pose = self._dc_interface.get_rigid_body_pose(self._root_handle)
        self._state.base_frame.pos = np.asarray(base_pose.p)
        self._state.base_frame.quat = np.asarray(base_pose.r)
        self._state.base_frame.lin_vel = (
            np.asarray(self._dc_interface.get_rigid_body_linear_velocity(self._root_handle)) * self.meters_per_unit
        )
        self._state.base_frame.ang_vel = np.asarray(
            self._dc_interface.get_rigid_body_angular_velocity(self._root_handle)
        )

        # assign to _measurement obj
        self._measurement.state = self._state
        self._measurement.base_ang_vel = np.asarray(self.ang_vel)
        self._measurement.base_lin_acc = np.asarray(self.base_lin)
        self._measurement.contact_force = self.contact_force
        self._measurement.contact_bool = self.contact_bool
        return
    
    def get_root_x_y_yaw(self):
        self.update()
        _root_quat = np.zeros(4)
        _root_quat[0] = self._state.base_frame.quat[3]  # w
        _root_quat[1] = self._state.base_frame.quat[0]  # x
        _root_quat[2] = self._state.base_frame.quat[1]  # y
        _root_quat[3] = self._state.base_frame.quat[2]  # z

        if _root_quat[0] < 0:
            _root_quat = -_root_quat

        _yaw = get_xyz_euler_from_quaternion(_root_quat)[2]

        _root_pos = self._state.base_frame.pos
        
        return np.array([_root_pos[0], _root_pos[1], _yaw])


    ###############################
    ## default
    @property
    def joint_num(self):
        return 4
    
    @property
    def link_names(self):
        return [
            "body",
            "m1_prop",
            "m2_prop",
            "m3_prop",
            "m4_prop",
            "imu_link",
            "basket",
            "basket_container_link",
            "attach_link"
        ]
    @property
    def attach_link_names(self):
        return ["attach_link"]
    @property
    def joint_names(self):
        return [
            'm1_joint',
            'm2_joint',
            'm3_joint',
            'm4_joint'
        ]
    
    @property
    def default_joint_pos(self):
        return np.array([0.0, 0.0, .0, 0.0])
    
    
    @property
    def default_hover_height(self):
        return 1.5

    @property
    def default_land_height(self):
        return 0.05
    
    @property
    def default_state(self):
        _default_state = CFState()
        _default_state.base_frame.pos = self._init_position
        
        _default_state.base_frame.quat = np.array([self._init_orientation[1], self._init_orientation[2], self._init_orientation[3], self._init_orientation[0]])
        _default_state.base_frame.ang_vel = np.zeros(3)
        _default_state.base_frame.lin_vel = np.zeros(3)
        _default_state.joint_pos = self.default_joint_pos
        _default_state.joint_vel = np.zeros(self.joint_num)
        
        return _default_state
        
