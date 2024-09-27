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
from omni.isaac.quadruped.quadruped import Quadruped

from omni.isaac.quadruped.utils.a1_classes import A1Command
from omni.isaac.quadruped.utils.a1arm_classes import A1ArmState, A1ArmMeasurement, A1ArmCommand
from omni.isaac.quadruped.controllers import A1QPController

import omni.isaac.dynamic_control._dynamic_control as omni_dc
from pxr import Gf
from typing import Optional, List
from collections import deque
import numpy as np
import carb
import math
from copy import deepcopy

from omni.isaac.manipulators.grippers.gripper import Gripper
from typing import List, Callable
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.quadruped.utils.rot_utils import get_xyz_euler_from_quaternion


class UnitreeArm(Quadruped):
    """For unitree based quadrupeds (A1 or Go1)"""

    def __init__(
        self,
        prim_path: str,
        name: str = "unitree_quadruped",
        physics_dt: Optional[float] = 1 / 400.0,
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        model: Optional[str] = "A1",
        end_effector_prim_name: Optional[str] = None,
        gripper_open_position: Optional[np.ndarray] = None,
        gripper_closed_position: Optional[np.ndarray] = None,
        deltas: Optional[np.ndarray] = None,
    ) -> None:
        """
        [Summary]
        
        initialize robot, set up sensors and controller
        
        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            physics_dt {float} -- physics downtime of the controller
            usd_path {str} -- robot usd filepath in the directory
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot
            model {str} -- robot model (can be either A1 or Go1)
            way_points {np.ndarray} -- waypoint and heading of the robot
        
        """


        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)

        self._end_effector_prim_name = end_effector_prim_name
        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets server")
                if model == "A1":
                    asset_path = assets_root_path + "/Isaac/Robots/Unitree/a1.usd"
                else:
                    asset_path = assets_root_path + "/Isaac/Robots/Unitree/go1.usd"

                carb.log_warn("asset path is: " + asset_path)
                prim.GetReferences().AddReference(asset_path)
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/gripperStator"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
            if gripper_open_position is None:
                gripper_open_position = np.array([-math.pi/2]) / get_stage_units()
            if gripper_closed_position is None:
                gripper_closed_position = np.array([-math.pi/4])
        else:
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/gripperStator"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
            if gripper_open_position is None:
                gripper_open_position = np.array([-math.pi/2]) / get_stage_units()
            if gripper_closed_position is None:
                gripper_closed_position = np.array([-math.pi/4])
        gripper_open_position = np.array([-math.pi/2]) / get_stage_units()
        gripper_closed_position = np.array([-math.pi/4])

        
        self._init_position = np.array(position) if position is not None else np.array([0.0, 0.0, 0.0])
        self._init_orientation = np.array(orientation) if orientation is not None else np.array([0.0, 0.0, 0.0, 1.0])

        self.base_lin = np.zeros(3)
        self.ang_vel = np.zeros(3)

        super().__init__(prim_path=self._prim_path, name=name, position=position, orientation=orientation)

        # Gripper setup
        if deltas is None:
            deltas = np.array([-math.pi/16]) / get_stage_units()
        self._gripper = Z1ArmGripper(
            end_effector_prim_path=self._end_effector_prim_path,
            joint_prim_names=self.gripper_joint_names,
            joint_opened_positions=gripper_open_position,
            joint_closed_positions=gripper_closed_position,
            action_deltas=deltas,
        )
            

        self._measurement = A1ArmMeasurement()
        self._command = A1Command()
        self._state = A1ArmState()
        
        
        self.meters_per_unit = get_stage_units()

        # contact sensor setup
        self._cs = _sensor.acquire_contact_sensor_interface()
        self.feet_order = ["FL", "FR", "RL", "RR"]
        self.feet_path = [
            self._prim_path + "/FL_foot",
            self._prim_path + "/FR_foot",
            self._prim_path + "/RL_foot",
            self._prim_path + "/RR_foot",
        ]


        self.color = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (1, 1, 0, 1)]

        for i in range(4):
            add_contact_sensor, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/sensor",
                parent=self.feet_path[i],
                min_threshold=0,
                max_threshold=1000000,
                color=self.color[i],
                radius=0.03,
                sensor_period=physics_dt,
                visualize=False,
            )

            if not add_contact_sensor:
                carb.log_error(self.feet_path[i] + " contact sensor not added")

        self.foot_force = np.zeros(4)
        self.enable_foot_filter = True
        self._FILTER_WINDOW_SIZE = 20
        self._foot_filters = [deque(), deque(), deque(), deque()]

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

    #########################
    ## initialize & post_reset
    def initialize(self, physics_sim_view=None) -> None:
        """[summary]

        initialize dc interface, set up drive mode and initial robot state
        """
        super().initialize(physics_sim_view=physics_sim_view)
        # self._root_handle = self._dc_interface.get_object('/World/Aliengo/trunk')


        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,  # all dof in this articulation (Articulation.dof_names)
        )


        joint_names_dc_order = self.dof_names

        
        joint_names_qp_order = self.joint_names

        #, 'jointGripper'
        assert len(joint_names_dc_order) == len(joint_names_qp_order)
        self.to_qp_order = []
        self.to_dc_order = []
        for i in joint_names_qp_order:
            self.to_qp_order.append(joint_names_dc_order.index(i))
        
        for i in joint_names_dc_order:
            self.to_dc_order.append(joint_names_qp_order.index(i))
        self.to_qp_order = np.array(self.to_qp_order, dtype=np.uint8)
        self.to_dc_order = np.array(self.to_dc_order, dtype=np.uint8)



        # kp: stiffness, kd: damping
        self.set_dof_drive_mode(drive="force")
        _control = []
        _kp = []
        _kd = []
        for _i in self.dof_names:
            if _i in (self.arm_joint_names + self.gripper_joint_names):
                _control.append("position")
                _kp.append(10000000.)
                _kd.append(100000.)
            else:
                _control.append("effort")
                _kp.append(0.0)
                _kd.append(0.0)

        self.set_dof_control(control=_control, 
                             kp=_kp, 
                             kd=[0.0]*self.joint_num, 
                             drive="force")
        
        
        self.set_state(self.default_a1_state)
        return

    def post_reset(self) -> None:
        """[summary]

        post reset articulation
        """
        super().post_reset()
        
        self.set_state(self.default_a1_state)
        return
     
    

    ###############################
    ## state operation
    def set_state(self, state: A1ArmState) -> None:
        """[Summary]
        
        Set the kinematic state of the robot.

        Args:
            state {A1State} -- The state of the robot to set.

        Raises:
            RuntimeError: When the DC Toolbox interface has not been configured.
        """
        # self._dc_interface: dynamic control, 
        self.check_dc_interface()

        # set base state
        base_pose = omni_dc.Transform(state.base_frame.pos, state.base_frame.quat)
        self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, state.base_frame.lin_vel)
        self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, state.base_frame.ang_vel)
        # cast joint state to numpy float32
        dof_state = self._dc_interface.get_articulation_dof_states(self._handle, omni_dc.STATE_ALL)
        
        # we convert controller order to DC order for setting state
        dc_joint_pos = state.joint_pos[self.to_dc_order]
        dc_joint_vel = state.joint_vel[self.to_dc_order]
        # for (name, pos) in zip(self.dof_names, dc_joint_pos):
        #     print("%s: %.2f" % (name, pos))
        dof_state["pos"] = np.asarray(dc_joint_pos, dtype=np.float32)
        dof_state["vel"] = np.asarray(dc_joint_vel, dtype=np.float32)
        dof_state["effort"] = 0.0
        # set joint state
        status = self._dc_interface.set_articulation_dof_states(self._handle, dof_state, omni_dc.STATE_ALL)
        if not status:
            raise RuntimeError("Unable to set the DOF state properly.")

    def get_state(self):        
        self.check_dc_interface()
        self.update()

        return deepcopy(self._state)
    

    def set_body_pose_from_state(self, state):# self._dc_interface: dynamic control, 
        self.check_dc_interface()

        # set base state
    
        base_pose = omni_dc.Transform( state.base_frame.pos, state.base_frame.quat)
        self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        # self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, state.base_frame.lin_vel)
        # self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, state.base_frame.ang_vel)

    
    def update_contact_sensor_data(self) -> None:
        """[summary]
        
        Updates processed contact sensor data from the robot feets, store them in member variable foot_force
        """
        # Order: FL, FR, BL, BR
        for i in range(len(self.feet_path)):
            reading = self._cs.get_sensor_sim_reading(self.feet_path[i] + "/sensor")
            if reading.value is None:
                carb.log_warn("reading missing from" + self.feet_order[i])
                continue

            if self.enable_foot_filter:
                self._foot_filters[i].append(float(reading.value) * self.meters_per_unit)
                if len(self._foot_filters[i]) > self._FILTER_WINDOW_SIZE:
                    self._foot_filters[i].popleft()
                self.foot_force[i] = np.mean(self._foot_filters[i])

            else:
                self.foot_force[i] = float(reading.value) * self.meters_per_unit

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

        # we convert DC order to controller order for joint info
        qp_joint_pos = self.joint_state.positions[self.to_qp_order]
        qp_joint_vel = self.joint_state.velocities[self.to_qp_order]

        self._state.joint_pos = np.asarray(qp_joint_pos, dtype=np.float32)
        self._state.joint_vel = np.asarray(qp_joint_vel, dtype=np.float32)


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
        self._measurement.foot_forces = np.asarray(self.foot_force)
        self._measurement.base_ang_vel = np.asarray(self.ang_vel)
        self._measurement.base_lin_acc = np.asarray(self.base_lin)
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


    

    def get_arm_joint_positions(self):
        return self.get_joint_positions()[self.joint_arm_indices]

    ###############################
    ## default

    @property
    def default_begin_to_move_height(self):
        return 0.4
    
    @property
    def default_begin_to_manipulate_height(self):
        return 0.4
    
    @property
    def gripper(self):
        """[summary]

        Returns:
            Z1ArmGripper: [description]
        """
        return self._gripper       
    
    @property
    def default_dog_joint_pos(self):
        return np.array([0.0, 1.2, -1.8, 0, 1.2, -1.8, 0.0, 1.2, -1.8, 0, 1.2, -1.8])
    
    @property
    def default_arm_joint_pos(self):
        return np.array([0.0, 0.5, -0.5, 0.0, 0.1, 0.1]) # np.zeros(self.joint_num_arm)

    @property
    def default_finger_joint_pos(self):
        return np.zeros(self.joint_num_finger)

    @property
    def default_bugfix_joint_pos(self):
        return np.zeros(self.joint_num_bugfix)

    @property
    def default_a1_state(self):
        a1_state = A1ArmState()

        a1_state.base_frame.pos = self._init_position

        a1_state.base_frame.quat = np.array([self._init_orientation[1], self._init_orientation[2], self._init_orientation[3], self._init_orientation[0]])
        a1_state.base_frame.ang_vel = np.zeros(3)
        a1_state.base_frame.lin_vel = np.zeros(3)
        
        a1_state.joint_pos = np.concatenate((self.default_dog_joint_pos, self.default_arm_joint_pos[:self.joint_num_arm], self.default_finger_joint_pos[:self.joint_num_finger], self.default_bugfix_joint_pos))
        
        a1_state.joint_vel = np.zeros(self.joint_num)

        return a1_state
        
    
    @property
    def joint_num(self):
        return self.joint_num_dog + self.joint_num_arm + self.joint_num_finger + self.joint_num_bugfix
    
    @property
    def joint_num_dog(self):
        return 12
    
    @property
    def joint_num_arm(self):
        return 6
    
    @property
    def joint_num_finger(self):
        return 1


    @property
    def joint_num_bugfix(self):
        return 0


    @property
    def link_names(self):
        link_names = self.bugfix_link_names[:self.joint_num_bugfix] + self.dog_link_names
        if self.joint_num_arm:
            link_names += self.arm_link_names + self.finger_link_names
        link_names += self.imu_link_names 
        link_names += self.attach_link_names 
        return link_names

    @property
    def bugfix_link_names(self):
        bugfix_link = []
        for i in range(self.joint_num_bugfix):
            if i:
                bugfix_link.append('bugfix_base%d' % i)
            else:
                bugfix_link.append('bugfix_base')

        return bugfix_link

    @property
    def dog_link_names(self):
        if self.joint_num_bugfix:
            return [
                "trunk",
                "FL_hip",
                "FL_thigh_shoulder",
                "FL_thigh",
                "FL_calf",
                "FL_foot",
                "FR_hip",
                "FR_thigh_shoulder",
                "FR_thigh",
                "FR_calf",
                "FR_foot",
                "RL_hip",
                "RL_thigh_shoulder",
                "RL_thigh",
                "RL_calf",
                "RL_foot",
                "RR_hip",
                "RR_thigh_shoulder",
                "RR_thigh",
                "RR_calf",
                "RR_foot",
            ]
        else:
            return [
                "trunk",
                "FL_hip",
                # "FL_thigh_shoulder",
                "FL_thigh",
                "FL_calf",
                "FL_foot",
                "FR_hip",
                # "FR_thigh_shoulder",
                "FR_thigh",
                "FR_calf",
                "FR_foot",
                "RL_hip",
                # "RL_thigh_shoulder",
                "RL_thigh",
                "RL_calf",
                "RL_foot",
                "RR_hip",
                # "RR_thigh_shoulder",
                "RR_thigh",
                "RR_calf",
                "RR_foot",
            ]


    @property
    def arm_link_names(self):
        return [
            "link00",
            "link01",
            "link02",
            "link03",
            "link04",
            "link05",
            "link06",
        ]
    
    @property
    def eef_link_names(self):
        return "gripperStator"

    @property
    def finger_link_names(self):
        return ["gripperStator", "gripperMover"]
    
    @property
    def imu_link_names(self):
        return ["imu_link"]
    @property
    def attach_link_names(self):
        return ["attach_link"]
        
    
    @property
    def joint_names(self):
        return self.dog_joint_names[:self.joint_num_dog] + \
            self.arm_joint_names[:self.joint_num_arm] + \
            self.gripper_joint_names[:self.joint_num_finger] + \
            self.bugfix_joint_names[:self.joint_num_bugfix]

    @property
    def joint_dog_indices(self):
        begin = 0
        return np.arange(len(self.joint_names))[begin:begin+self.joint_num_dog]

    @property
    def joint_arm_indices(self):
        begin = self.joint_num_dog
        return np.arange(len(self.joint_names))[begin:begin+self.joint_num_arm]
    
    @property
    def joint_finger_indices(self):
        begin = self.joint_num_dog + self.joint_num_arm
        return np.arange(len(self.joint_names))[begin:begin+self.joint_num_finger]
    

    @property
    def bugfix_joint_names(self):
        bugfix_joint = []
        for i in range(self.joint_num_bugfix):
            bugfix_joint.append('bugfix_joint%d' % (i+1))
        return bugfix_joint


    @property
    def dog_joint_names(self):
        return ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FR_hip_joint', 
                'FR_thigh_joint', 'FR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint',
                'RL_calf_joint', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
    
    @property
    def arm_joint_names(self):
        return [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

    @property
    def gripper_joint_names(self):
        return ["jointGripper"]
    




########################################################################################
########################################################################################
########################################################################################
class Z1ArmGripper(Gripper):

    def __init__(
        self,
        end_effector_prim_path: str,
        joint_prim_names: List[str],
        joint_opened_positions: np.ndarray,
        joint_closed_positions: np.ndarray,
        action_deltas: np.ndarray = None,
    ) -> None:
        
        Gripper.__init__(self, end_effector_prim_path=end_effector_prim_path)
        self._joint_prim_names = joint_prim_names
        self._joint_dof_indicies = np.array([None])
        self._joint_opened_positions = joint_opened_positions
        self._joint_closed_positions = joint_closed_positions
        self._get_joint_positions_func = None
        self._set_joint_positions_func = None
        self._action_deltas = action_deltas
        self._articulation_num_dofs = None
        return
    
    def initialize(
        self,
        articulation_apply_action_func: Callable,
        get_joint_positions_func: Callable,
        set_joint_positions_func: Callable,
        dof_names: List,
        physics_sim_view: omni.physics.tensors.SimulationView = None,
    ) -> None:
        """Create a physics simulation view if not passed and creates a rigid prim view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            articulation_apply_action_func (Callable): apply_action function from the Articulation class.
            get_joint_positions_func (Callable): get_joint_positions function from the Articulation class.
            set_joint_positions_func (Callable): set_joint_positions function from the Articulation class.
            dof_names (List): dof names from the Articulation class.
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None

        Raises:
            Exception: _description_
        """
        Gripper.initialize(self, physics_sim_view=physics_sim_view)
        self._get_joint_positions_func = get_joint_positions_func
        self._articulation_num_dofs = len(dof_names)
        for index in range(len(dof_names)):
            if self._joint_prim_names[0] == dof_names[index]:
                self._joint_dof_indicies[0] = index
        # make sure that all gripper dof names were resolved
        if self._joint_dof_indicies[0] is None:
            raise Exception("Not all gripper dof names were resolved to dof handles and dof indices.")
        self._articulation_apply_action_func = articulation_apply_action_func
        current_joint_positions = get_joint_positions_func()
        if self._default_state is None:
            self._default_state = np.array(
                [
                    current_joint_positions[self._joint_dof_indicies[0]]
                ]
            )
        self._set_joint_positions_func = set_joint_positions_func
        return
    
    @property
    def joint_opened_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively when opened.
        """
        return self._joint_opened_positions

    @property
    def joint_closed_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively when closed.
        """
        return self._joint_closed_positions

    @property
    def joint_dof_indicies(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint dof indices in the articulation of the left finger joint and the right finger joint respectively.
        """
        return self._joint_dof_indicies

    @property
    def joint_prim_names(self) -> List[str]:
        """
        Returns:
            List[str]: the left finger joint prim name and the right finger joint prim name respectively.
        """
        return self._joint_prim_names

    

    def open(self) -> None:
        """Applies actions to the articulation that opens the gripper (ex: to release an object held).
        """
        self._articulation_apply_action_func(self.forward(action="open"))
        return

    def close(self) -> None:
        """Applies actions to the articulation that closes the gripper (ex: to hold an object).
        """
        self._articulation_apply_action_func(self.forward(action="close"))
        return

    def set_action_deltas(self, value: np.ndarray) -> None:
        """
        Args:
            value (np.ndarray): deltas to apply for finger joint positions when openning or closing the gripper. 
                               [left, right]. Defaults to None.
        """
        self._action_deltas = value
        return

    def get_action_deltas(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: deltas that will be applied for finger joint positions when openning or closing the gripper. 
                        [left, right]. Defaults to None.
        """
        return self._action_deltas

    def set_default_state(self, joint_positions: np.ndarray) -> None:
        """Sets the default state of the gripper

        Args:
            joint_positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively.
        """
        self._default_state = joint_positions
        return

    def get_default_state(self) -> np.ndarray:
        """Gets the default state of the gripper

        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._default_state

    def post_reset(self):
        Gripper.post_reset(self)
        self._set_joint_positions_func(
            positions=self._default_state, joint_indices=[self._joint_dof_indicies[0], self._joint_dof_indicies[1]]
        )
        return

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """
        Args:
            positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively.
        """
        self._set_joint_positions_func(
            positions=positions, joint_indices=[self._joint_dof_indicies[0]]
        )
        return

    def get_joint_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._get_joint_positions_func(joint_indices=[self._joint_dof_indicies[0]])




    def forward(self, action: str) -> ArticulationAction:
        """calculates the ArticulationAction for all of the articulation joints that corresponds to "open"
           or "close" actions.

        Args:
            action (str): "open" or "close" as an abstract action.

        Raises:
            Exception: _description_

        Returns:
            ArticulationAction: articulation action to be passed to the articulation itself
                                (includes all joints of the articulation).
        """
        if action == "open":
            target_joint_positions = [None] * self._articulation_num_dofs
            if self._action_deltas is None:
                target_joint_positions[self._joint_dof_indicies[0]] = self._joint_opened_positions[0]
            else:
                current_joint_positions = self._get_joint_positions_func()
                current_left_finger_position = current_joint_positions[self._joint_dof_indicies[0]]
                target_joint_positions[self._joint_dof_indicies[0]] = (
                    current_left_finger_position + self._action_deltas[0]
                )
        elif action == "close":
            target_joint_positions = [None] * self._articulation_num_dofs
            if self._action_deltas is None:
                target_joint_positions[self._joint_dof_indicies[0]] = self._joint_closed_positions[0]
            else:
                current_joint_positions = self._get_joint_positions_func()
                current_left_finger_position = current_joint_positions[self._joint_dof_indicies[0]]
                # target_joint_positions[self._joint_dof_indicies[0]] = (
                #     current_left_finger_position - self._action_deltas[0]
                # )
                target_joint_positions[self._joint_dof_indicies[0]] = self._joint_closed_positions[0]
        else:
            raise Exception("action {} is not defined for ParallelGripper".format(action))
        return ArticulationAction(joint_positions=target_joint_positions)

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """Applies actions to all the joints of an articulation that corresponds to the ArticulationAction of the finger joints only.

        Args:
            control_actions (ArticulationAction): ArticulationAction for the left finger joint and the right finger joint respectively.
        """
        joint_actions = ArticulationAction()
        if control_actions.joint_positions is not None:
            joint_actions.joint_positions = [None] * self._articulation_num_dofs
            joint_actions.joint_positions[self._joint_dof_indicies[0]] = control_actions.joint_positions[0]
        if control_actions.joint_velocities is not None:
            joint_actions.joint_velocities = [None] * self._articulation_num_dofs
            joint_actions.joint_velocities[self._joint_dof_indicies[0]] = control_actions.joint_velocities[0]
        if control_actions.joint_efforts is not None:
            joint_actions.joint_efforts = [None] * self._articulation_num_dofs
            joint_actions.joint_efforts[self._joint_dof_indicies[0]] = control_actions.joint_efforts[0]
        self._articulation_apply_action_func(control_actions=joint_actions)
        return
    