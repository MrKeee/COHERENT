import numpy as np
import os
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import get_stage_units, add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
import carb
import omni
from omnigibson.robots.robot_base import BaseRobot



###
import omnigibson as og
from constants import *
from agents.unitree_arm import UnitreeArm
from agents.unitree_arm_controller import KinematicsSolver
from omnigibson.prims.xform_prim import XFormPrim
from copy import deepcopy
from typing import Optional, List
from collections.abc import Iterable
from omnigibson.sensors import create_sensor, SENSOR_PRIMS_TO_SENSOR_CLS, ALL_SENSOR_MODALITIES, VisionSensor, ScanSensor
from omni.isaac.quadruped.controllers import A1QPController
from omni.isaac.quadruped.utils.rot_utils import get_xyz_euler_from_quaternion, get_quaternion_from_euler

import omni.isaac.dynamic_control._dynamic_control as omni_dc
from scipy.spatial.transform import Rotation as R

import omni.usd
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf





#############################################################################################
## Aliengo
#############################################################################################
class AliengoAgent(XFormPrim, UnitreeArm):
    def __init__(
            self, 
            name,
            prim_path=None, 
            # BasePrim
            scale=None,
            visible=True,
            visual_only=False,
            self_collisions=False,
            load_config=None,
            fixed_base=False,
            prim_type=0,  # PrimType.RIGID

            # Aliengo
            model: Optional[str] = "A1",
            physics_dt: Optional[float] = 1 / 400.0,
            position: Optional[np.ndarray] = None,
            orientation: Optional[np.ndarray] = None,  # scalar-rear
            end_effector_prim_name: Optional[str] = None,
            gripper_dof_names: Optional[List[str]] = None,
            gripper_open_position: Optional[np.ndarray] = None,
            gripper_closed_position: Optional[np.ndarray] = None,
            deltas: Optional[np.ndarray] = None,
            way_points: Optional[np.ndarray] = None,
            keyboard_mode=False,

            # Sensor
            obs_modalities="all",
            proprio_obs="default",
            cam_height=256,
            cam_width=256
        ):

        self._dog_active = False
        self._arm_active = False

        self.keyboard_mode = keyboard_mode
        
        self.name = name
        self._prim_path = "/World/" + self.name if prim_path is None else prim_path
        self._end_effector_prim_name =  "gripperStator" if end_effector_prim_name is None else self._prim_path + end_effector_prim_name
        self._gripper_dof_names = self.gripper_joint_names if gripper_dof_names is None else gripper_dof_names
        
        # full open is 0.05, cost 5 sim step to fully open a gripper when it is fully closed
        self._gripper_action_deltas = np.array([0.01]) / get_stage_units() if deltas is None else deltas

        self.physics_dt = physics_dt
        self.unit_move_velocity = 5
        self.unit_turn_velocity = 5
        self._default_velocity_command = [0.0, 0.0, 0.0, 0]


        self._obs_modalities = obs_modalities
        self._proprio_obs = proprio_obs
        self._proprio_obs = self.default_proprio_obs if proprio_obs == "default" else list(proprio_obs)


        self._cam_height = cam_height
        self._cam_width = cam_width
        self._sensor_prim_list = []


        self._init_position = np.array(position)
        self._init_orientation = np.array(orientation)[[3, 0, 1, 2]]  # scalar-first

        self._links = {}
        self._joints = {}
        self._first_view_cam = None
        self._third_view_cam = None
        self._cam_height = cam_height
        self._cam_width = cam_width

        self.attached_prim_path = None
        self.attached_prim_list = []

        self._action_type = "continuous"

        # Run super init
        # Create load config from inputs
        load_config = dict() if load_config is None else load_config
        load_config["scale"] = np.array(scale) if isinstance(scale, Iterable) else scale
        load_config["visible"] = visible
        load_config["visual_only"] = visual_only
        load_config["self_collisions"] = self_collisions
        load_config["prim_type"] = prim_type

        XFormPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            load_config=load_config
        )

        # s_dc_interface, _articulation_view, _articulation_controller
        UnitreeArm.__init__(
            self,
            prim_path=prim_path,
            name=name,
            physics_dt=physics_dt,
            usd_path=self.usd_path,
            position=self._init_position,
            orientation=self._init_orientation,
            end_effector_prim_name=self._end_effector_prim_name,
            gripper_open_position=gripper_open_position,
            gripper_closed_position=gripper_closed_position,
            deltas=deltas,
        )

        self._armbase_prim = get_prim_at_path(self.prim_path+"/"+self.arm_link_names[0])
        self._agent_attach_prim = get_prim_at_path(self.prim_path+"/"+self.attach_link_names[0])
        # Controller
        # controller for dog （effots control） & arm (position control)
        # BUT! self._qp_controller.advance only compute the inverse dynamics for dog
        if way_points:
            self._qp_controller = A1QPController(model, physics_dt, way_points) 
        else:
            self._qp_controller = A1QPController(model, physics_dt)
        self._qp_controller.setup()
        
        # controller for arm
        self._ik_solver = KinematicsSolver(robot_articulation=self, 
                                           end_effector_frame_name=self.eef_link_names, 
                                           robot_description_path=self.robot_arm_descriptor_yamls, 
                                           urdf_path=self.urdf_path)

        
        self._dof_control_modes: List[int] = list()

        # # Inverse Kinematics Solver
        # # self._ik_solver = RMPFlowController(name="franka_ik_controller", robot_articulation=self)
        # self._ik_solver = KinematicsSolver(robot_articulation=self)
        # self.ee_dummy = self.create_ee_dummy()

    #########################
    @property
    def arm_active(self):
        return self._arm_active
    @arm_active.setter
    def arm_active(self, flag):
        if flag:
            self.dog_active = False
        self._arm_active = flag
    @property
    def dog_active(self):
        return self._qp_controller._ctrl_states._init_transition == 1
    @dog_active.setter
    def dog_active(self, flag):
        if flag:
            self.arm_active = False
            self.dog_stand_begin_to_move()
        else:
            self.dog_stand_still()
    

    #########################
    ## load, initialize, remove
    # 添加到Stage并获取prim，BasePrim load(), Franka 无load()
    def _load(self):
        """
        Load the object into pybullet and set it to the correct pose
        """
    
        prim = get_prim_at_path(self.prim_path)
        return prim
    
    
    def _iterate_prim_children_to_find_sensors(self, root_prim):
        for prim in root_prim.GetChildren():
            prim_type = prim.GetPrimTypeInfo().GetTypeName()
            # when "Camera" or "Lidar"
            if prim_type in SENSOR_PRIMS_TO_SENSOR_CLS:
                self._sensor_prim_list.append(prim)
            self._iterate_prim_children_to_find_sensors(prim)


    # 获取sensors
    def _post_load(self):
        # Run super post load first
        super()._post_load()
        
        self.cache_links_without_check()

        # Search for any sensors this robot might have attached to any of its links
        self._sensor_prim_list = []
        self._sensors = dict()
        obs_modalities = set()


        for link_name, link in self._links.items():
            # link: {"prim": xxx, "name": xxx, "prim_path": xxx, "index": xxx}
            # Search through all children prims and see if we find any sensor
            self._iterate_prim_children_to_find_sensors(link["prim"])
            
        
        for index, sensor_prim in enumerate(self._sensor_prim_list):
            prim_name = sensor_prim.GetName()
            prim_type = sensor_prim.GetPrimTypeInfo().GetTypeName()
            prim_name = prim_name if "first_view" in prim_name or 'third_view' in prim_name else \
                        f"{self.name}:sensor{index}_{prim_type}"
            # Infer what obs modalities to use for this sensor
            sensor_cls = SENSOR_PRIMS_TO_SENSOR_CLS[prim_type]
            modalities = sensor_cls.all_modalities if self._obs_modalities == "all" else \
                sensor_cls.all_modalities.intersection(self._obs_modalities)
            obs_modalities = obs_modalities.union(modalities)

            sensor_kwargs = {'image_height': self.cam_height, 'image_width': self.cam_width, 'viewport_name': None}

            # Create the sensor and store it internally
            sensor = create_sensor(
                sensor_type=prim_type,
                prim_path=str(sensor_prim.GetPrimPath()),
                name=prim_name,
                modalities=modalities,
                sensor_kwargs=sensor_kwargs
            )
            self._sensors[sensor.name] = sensor

        # Since proprioception isn't an actual sensor, we need to possibly manually add it here as well
        if self._obs_modalities == "all" or "proprio" in self._obs_modalities:
            obs_modalities.add("proprio")

        # Update our overall obs modalities
        self._obs_modalities = obs_modalities

    # 删除prim和sensors，BasePrim remove(), Franka 无remove()
    def remove(self):
        XFormPrim.remove(self)
        self._post_remove()

    def _post_remove(self):
        # Remove all sensors
        for sensor in self._sensors.values():
            sensor.remove()

    def update_physics_sim_view(self, physics_sim_view=None):
        # initialize _handle &　_root_handle

        self._articulation_view._is_initialized = False
        UnitreeArm.initialize(self, physics_sim_view)
        self.post_reset()
        

    
    # invoked by og.sim.play() and cannot invoke og.sim.stop() after initialization
    def initialize(self, physics_sim_view=None):
        # Run super first
        # initialize _handle &　_root_handle
        UnitreeArm.initialize(self, physics_sim_view=physics_sim_view)
        self.post_reset()
        # invoke dump_state
        XFormPrim.initialize(self)


        # if not self.keyboard_mode:
        #     self.wake_deault_gait_control()
        # else:
        #     self.control_aliengo_keyboard()

        # Initialize all sensors
        for sensor in self._sensors.values():
            sensor.initialize()
        
        # Cache links
        self.cache_links_with_check()
        # Cache joints
        self.cache_joints_with_check()


    def _dump_state(self):
        # We don't call super, instead, this state is simply the root link state and all joint states
        return self.default_a1_state
        

    def _load_state(self, state):
        self.set_state(state)
        
    def _serialize(self, state):
        # We serialize by first flattening the root link state and then iterating over all joints and
        # adding them to the a flattened array
        
        base_state = [state.base_frame.pos, state.base_frame.quat, state.base_frame.lin_vel, state.base_frame.ang_vel]
        joint_state = [state.joint_pos, state.joint_vel]
        state_flat = base_state + joint_state

        return np.concatenate(state_flat).astype(float)

    def post_reset(self) -> None:
        """[summary]

        post reset articulation and qp_controller
        """
        super().post_reset()
        # reset
        self.arm_active = False
        self.dog_active = False
        
        self._qp_controller.reset()
        self.set_state(self.default_a1_state)
        return
    #######################
    ## 
    def get_ee_position_orientation(self):
        ee_pos, ee_mat = self._ik_solver.compute_end_effector_pose()
        quat_back = R.from_matrix(ee_mat).as_quat()
        ee_quat = np.array([quat_back[3], quat_back[0], quat_back[1], quat_back[2]])
        # scalar-first
        return ee_pos, ee_quat
    

    def get_arm_base_position_orientation(self):
        # tmp_pose = self._dc_interface.get_rigid_body_pose(self._links[self.arm_link_names[0]]["handle"])
        # arm_base_pos = np.asarray(tmp_pose.p, dtype=np.float32)
        # quat_back = np.asarray(tmp_pose.r, dtype=np.float32)
        # arm_base_ori = quat_back[[3, 0, 1, 2]]
        # # scalar-first
        # # print(arm_base_pos, "---------------------------------------------")
        # return arm_base_pos, arm_base_ori
    


        # armbase2base_pos = self._armbase_prim.GetAttribute('xformOp:translate').Get()
        # armbase2base_orientaton = self._armbase_prim.GetAttribute("xformOp:orient").Get()
        # base2world_pos = self._prim.GetAttribute('xformOp:translate').Get()
        # base2world_orientaton = self._prim.GetAttribute("xformOp:orient").Get()
        

        # armbase2base = Gf.Matrix4d(1.0)
        # armbase2base = armbase2base.SetTranslateOnly(armbase2base_pos)
        # # print(armbase2base.SetTranslate(armbase2base_pos))
        # armbase2base = armbase2base.SetRotateOnly(armbase2base_orientaton)
        # # print(armbase2base.SetRotate(armbase2base_orientaton))
        # # print(armbase2base)


        # base2world = Gf.Matrix4d(1.0)
        # base2world = base2world.SetTranslateOnly(base2world_pos)
        # base2world = base2world.SetRotateOnly(base2world_orientaton)
        # # print(base2world)

        # armbase2world = base2world * armbase2base
        armbase2world = omni.usd.get_world_transform_matrix(self._armbase_prim)
        pos = np.array(armbase2world.ExtractTranslation())
        
        # print(armbase2base)
        # quatd = Gf.Quatd(armbase2world.rotmx)
        quatd = armbase2world.ExtractRotationQuat()
        orientaton = np.array([quatd.real, 
                               quatd.imaginary[0], 
                               quatd.imaginary[1], 
                               quatd.imaginary[2]])



        return pos, orientaton

    def set_attach_prim(self, attached_prim_path):
        # self.attached_prim_path = attached_prim_path
        self.attached_prim_list.append(attached_prim_path)
        self.attached_prim_list.append(attached_prim_path+'/base_link')

    def attach(self):
        # if self.attached_prim_path is not None:
        #     tmp_pose = self._dc_interface.get_rigid_body_pose(self._links[self.attach_link_names[0]]["handle"])
        #     attach_pos = np.asarray(tmp_pose.p, dtype=np.float32).tolist()
        #     # quat_back = np.asarray(tmp_pose.r, dtype=np.float32)
        #     # arm_base_ori = quat_back[[3, 0, 1, 2]]
        #     xform = get_current_stage().GetPrimAtPath(self.attached_prim_path)
        #     translate = xform.GetAttribute('xformOp:translate').Set(Gf.Vec3d(attach_pos[0], attach_pos[1], attach_pos[2]))
        #     print("Attach!!!!!!!!!!!!!!!!!!")
        if len(self.attached_prim_list):
            # tmp_pose = self._dc_interface.get_rigid_body_pose(self._links[self.attach_link_names[0]]["handle"])
            # attach_pos = np.asarray(tmp_pose.p, dtype=np.float32).tolist()
            # quat_back = np.asarray(tmp_pose.r, dtype=np.float32)
            # arm_base_ori = quat_back[[3, 0, 1, 2]]
            # for prim_path in self.attached_prim_list:
                # print(prim_path)


            # armattach2base_pos = self._armattach_prim.GetAttribute('xformOp:translate').Get()
            # armattach2base_orientaton = self._armattach_prim.GetAttribute("xformOp:orient").Get()
            # base2world_pos = self._prim.GetAttribute('xformOp:translate').Get()
            # base2world_orientaton = self._prim.GetAttribute("xformOp:orient").Get()
            

            # armattach2base = Gf.Matrix4d(1.0)
            # armattach2base = armattach2base.SetTranslateOnly(armattach2base_pos)
            # armattach2base = armattach2base.SetRotateOnly(armattach2base_orientaton)
            
            # print(armattach2base)
            # print(omni.usd.get_world_transform_matrix(self._armattach_prim))


            # base2world = Gf.Matrix4d(1.0)
            # base2world = base2world.SetTranslateOnly(base2world_pos)
            # base2world = base2world.SetRotateOnly(base2world_orientaton)
            # print(base2world)
            # print(omni.usd.get_world_transform_matrix(self._prim))

            armattach2world = omni.usd.get_world_transform_matrix(self._agent_attach_prim)
            


            xform = get_prim_at_path(self.attached_prim_list[0])
            # print("111111111111111111111:", armattach2world.ExtractTranslation())
            xform.GetAttribute('xformOp:translate').Set(armattach2world.ExtractTranslation())
            # print("222222222222222222222:", xform.GetAttribute('xformOp:translate').Get())

            xform = get_prim_at_path(self.attached_prim_list[1])
            xform.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0, 0, 0))
            # print("Attach!!!!!!!!!!!!!!!!!!")
            

        





    def detach(self):
        self.attached_prim_path = None
        self.attached_prim_list = []


    

    ########################
    ## Alticulation
    @property
    def links(self):
        return self._links

    @property
    def joints(self):
        return self._joints
    

    def disable_arm_gravity(self) -> None:
        """Keep gravity from affecting the robot
        """
        for body_index in range(self._dc_interface.get_articulation_body_count(self._handle)):
            body = self._dc_interface.get_articulation_body(self._handle, body_index)

            if self._dc_interface.get_rigid_body_name(body) in self.arm_link_names + self.eef_link_names + self.finger_link_names:
                self._dc_interface.set_rigid_body_disable_gravity(body, True)
        return


    def cache_links_without_check(self):
        # We iterate over all children of this object's prim,
        # and grab any that are presumed to be rigid bodies (i.e.: other Xforms)
        self._links = dict()
        joint_children = set()

        link_num = 0
        for prim in self._prim.GetChildren():
            link_name = prim.GetName()
            link_prim_path = prim.GetPath()
            if prim.GetPrimTypeInfo().GetTypeName() == "Xform":
                self._links[link_name] = {
                    "link_name": link_name,
                    "prim": prim,
                    "prim_path": link_prim_path
                }
                assert link_name == self.link_names[link_num], \
                    "Inconsistent link order"
                print("Cache link prim at ", link_prim_path)
                link_num += 1
                
                for child_prim in prim.GetChildren():
                    if "joint" in child_prim.GetPrimTypeInfo().GetTypeName().lower():
                        # Store the child target of this joint
                        relationships = {r.GetName(): r for r in child_prim.GetRelationships()}
                        # Only record if this is NOT a fixed link tying us to the world (i.e.: no target for body0)
                        if len(relationships["physics:body0"].GetTargets()) > 0:
                            joint_children.add(relationships["physics:body1"].GetTargets()[0].pathString.split("/")[-1])


        # Infer the correct root link name -- this corresponds to whatever link does not have any joint existing
        # in the children joints
        valid_root_links = list(set(self._links.keys()) - joint_children)

        self._root_link_name = "trunk"
        
    def cache_links_with_check(self):
        assert self._handle is not None, "Articulation is not initialized yet"

        # We iterate over all children of this object's prim,
        # and grab any that are presumed to be rigid bodies (i.e.: other Xforms)
        self._links = dict()
        joint_children = set()

        link_num = 0
        for prim in self._prim.GetChildren():
            link_name = prim.GetName()
            link_prim_path = prim.GetPath()
            if prim.GetPrimTypeInfo().GetTypeName() == "Xform":
                self._links[link_name] = {
                    "link_name": link_name,
                    "prim": prim,
                    "prim_path": link_prim_path,
                    "index": self._dc_interface.find_articulation_body_index(self._handle, link_name),
                    "handle": self._dc_interface.find_articulation_body(self._handle, link_name)
                }
                assert link_name == self.link_names[link_num], \
                    "Inconsistent link order"
                print("Cache link prim at ", link_prim_path)
                link_num += 1
                
                for child_prim in prim.GetChildren():
                    if "joint" in child_prim.GetPrimTypeInfo().GetTypeName().lower():
                        # Store the child target of this joint
                        relationships = {r.GetName(): r for r in child_prim.GetRelationships()}
                        # Only record if this is NOT a fixed link tying us to the world (i.e.: no target for body0)
                        if len(relationships["physics:body0"].GetTargets()) > 0:
                            joint_children.add(relationships["physics:body1"].GetTargets()[0].pathString.split("/")[-1])


        # Infer the correct root link name -- this corresponds to whatever link does not have any joint existing
        # in the children joints
        valid_root_links = list(set(self._links.keys()) - joint_children)

        self._root_link_name = "trunk"
        
        assert self._dc_interface.get_rigid_body_name(self._root_handle) == self._root_link_name, \
            "Inconsistent root prim"


    def cache_joints_with_check(self):
        assert self._handle is not None and self._dc_interface is not None, "Articulation is not initialized yet"


        # Initialize joints dictionary
        self._joints = dict()

        n_dof = self._dc_interface.get_articulation_dof_count(self._handle)
        assert n_dof == self.joint_num, "Inconsistent joint num"

        # Additionally grab DOF info if we have non-fixed joints
        if n_dof > 0:
            for i in range(n_dof):
                # dof is equivalent to non-fixed joint
                joint_handle = self._dc_interface.get_articulation_dof(self._handle, i)
                joint_name = self._dc_interface.get_dof_name(joint_handle)
                joint_path = self._dc_interface.get_dof_path(joint_handle)
                joint_prim = get_prim_at_path(joint_path)

                # assert joint_name == (self.arm_joint_names + self.gripper_joint_names)[i], \
                #     "Inconsistent joint order"

                self._joints[joint_name] = {
                    "joint_name": joint_name,
                    "prim": joint_prim,
                    "prim_path": joint_path,
                    "index": i,
                    "handle": self._dc_interface.find_articulation_dof(self._handle, joint_name)
                }
                print("Cache joint prim at ", joint_path)

    ########################
    ## gripper
    def is_gripper_fully_opened(self):

        current_position = self.gripper.get_joint_positions()
        target_position = self.gripper.joint_opened_positions
        if np.linalg.norm(current_position-target_position, ord=1) < 0.03:
            return True
        else:
            return False



    ########################
    ## sensors
    @property
    def cam_height(self):
        return self._cam_height

    @property
    def cam_width(self):
        return self._cam_width

    def get_obs(self):
        """
        Grabs all observations from the robot. This is keyword-mapped based on each observation modality
            (e.g.: proprio, rgb, etc.)

        Returns:
            dict: Keyword-mapped dictionary mapping observation modality names to
                observations (usually np arrays)
        """
        # Our sensors already know what observation modalities it has, so we simply iterate over all of them
        # and grab their observations, processing them into a flat dict
        obs_dict = dict()
        for sensor_name, sensor in self._sensors.items():
            sensor_obs = sensor.get_obs()
            for obs_modality, obs in sensor_obs.items():
                obs_dict[f"{sensor_name}_{obs_modality}"] = obs

        # Have to handle proprio separately since it's not an actual sensor
        if "proprio" in self._obs_modalities:
            obs_dict["proprio"] = self.get_proprioception()

        return obs_dict

    def get_proprioception(self):
        """
        Returns:
            n-array: numpy array of all robot-specific proprioceptive observations.
        """
        proprio_dict = self._get_proprioception_dict()
        return np.concatenate([proprio_dict[obs] for obs in self._proprio_obs])

    def _get_proprioception_dict(self):
        """
        Returns:
            dict: keyword-mapped proprioception observations available for this robot.
                Can be extended by subclasses
        """
        joint_positions = self.get_joint_positions()
        joint_velocities = self.get_joint_velocities()
        joint_efforts = self.get_joint_efforts()
        pos, ori = self.get_position(), self.get_rpy()
        return dict(
            joint_qpos=joint_positions,
            joint_qpos_sin=np.sin(joint_positions),
            joint_qpos_cos=np.cos(joint_positions),
            joint_qvel=joint_velocities,
            joint_qeffort=joint_efforts,
            robot_pos=pos,
            robot_ori=ori,
            robot_ori_cos=np.cos(ori),
            robot_ori_sin=np.sin(ori),
            robot_lin_vel=self.get_linear_velocity(),
            robot_ang_vel=self.get_angular_velocity(),
        )
    
        
    
    @property
    def default_proprio_obs(self):
        obs_keys = [
                "robot_pos",
                "robot_ori",
                "joint_qpos",
                "joint_qvel",
                "joint_qeffort",
                # "gripper_{}_qpos",
                # "grasp_{}".format(arm),
            ]
        return obs_keys


    ######################
    ## dog gaits control

    def wake_deault_gait_control(self):
        # still 
        self.dog_stand_still()
        og.sim.add_physics_callback("aliengo_move", callback_fn=self.on_physics_step)

    def on_physics_step(self, dt):
        # self._qp_controller._ctrl_states._init_transition = 0

        self.move(dt, goal_velocity=[0, 0, 0, 0], path_follow=False, auto_start=False)
    
    def dog_stand_still(self):
        self._qp_controller._ctrl_states._init_transition = 0
        self._qp_controller._ctrl_states._prev_transition = 1

    
    def dog_stand_begin_to_move(self):
        self._qp_controller._ctrl_states._init_transition = 1
        self._qp_controller._ctrl_states._prev_transition = 0

    def move(self, dt=None, goal_velocity=None, path_follow=False, auto_start=True) -> np.ndarray:
        """[summary]
        
        compute desired torque and set articulation effort to robot joints
        
        Argument:
        dt {float} -- Timestep update in the world.
        goal {List[int]} -- x velocity, y velocity, angular velocity, state switch
        path_follow {bool} -- true for following coordinates, false for keyboard control
        auto_start {bool} -- true for start trotting after 1 sec, false for start trotting after switch mode function is called

        Returns:
        np.ndarray -- The desired joint torques for the robot.
        """
        
        self.check_dc_interface()
        if dt is None:
            dt = self.physics_dt
        if goal_velocity is None:
            goal_velocity = self._default_velocity_command
        else:
            self._goal_velocity = goal_velocity

        self._dc_interface.wake_up_articulation(self._handle)
        self.update()
        self._qp_controller.set_target_command(goal_velocity)

        # dim: (self.joint_num_dog, )
        self._command.desired_joint_torque = self._qp_controller.advance(dt, self._measurement, path_follow, auto_start)
        # print(self._command.desired_joint_torque)
        # we convert controller order to DC order for command torque
        # array([-18.21165053, -21.91257812, -13.85122029,  16.57268513,
        # 13.72532217, -53.48756261,  -7.62225414, -20.8077847 ,
        # 45.04128496,  94.7842871 ,  39.34298501,  28.92729254])

        dc_toque_reorder = np.zeros(self.joint_num, dtype=np.float32)
        dc_toque_reorder[self.to_qp_order[self.joint_dog_indices]] = self._command.desired_joint_torque
        # self._dc_interface.set_articulation_dof_efforts(self._handle, np.asarray(torque_reorder, dtype=np.float32))
        
        # only set dog's joint efforts (omni.isaac.core.articulations.Articulation)
        # self.set_joint_efforts(efforts=np.asarray(self._command.desired_joint_torque, dtype=np.float32), joint_indices=self.to_qp_order[:self.joint_num_dog])
        self.set_joint_efforts(efforts=np.asarray(dc_toque_reorder, dtype=np.float32))
        # self._dc_interface.set_articulation_dof_efforts(self._handle, np.asarray(dc_toque_reorder, dtype=np.float32))

        
        return self._command

    ## dog pose control
    def set_dog_pose(self, pose):
        self.check_dc_interface()

        # scalar-back
        tmp_pose = self._dc_interface.get_rigid_body_pose(self._root_handle)
        pos = np.asarray(tmp_pose.p, dtype=np.float32)
        quat_back = np.asarray(tmp_pose.r, dtype=np.float32)
        pos[:2] = pose[:2]
        quat_first = quat_back[[3, 0, 1, 2]]
        euler = get_xyz_euler_from_quaternion(quat_first)
        euler[2] = pose[2]
        quat_first = get_quaternion_from_euler(euler)
        quat_back = quat_first[[1, 2, 3, 0]]
        # set base state
        base_pose = omni_dc.Transform(pos, quat_back)
        self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, np.zeros(3))
        self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, np.zeros(3))

        # position, orientation = self._articulation_view.get_world_poses()
        # position[0, :2] = pose[:2]

        # # scalar-first (w, x, y, z)
        # quat_first = orientation[0]
        # euler = get_xyz_euler_from_quaternion(quat_first)
        # euler[2] = pose[2]
        # quat_first = get_quaternion_from_euler(euler)
        # orientation[0] = quat_first
        # self._articulation_view.set_world_poses(position, orientation)
    
    def get_dog_pose(self):

        return self.get_root_x_y_yaw()
    

    # 6D
    def set_curr_position_orientation(self, position, orientation):

        # if position is not None:
        #     position = self._backend_utils.convert(position, self._device)
        #     position = self._backend_utils.expand_dims(position, 0)
        # if orientation is not None:
        #     orientation = self._backend_utils.convert(orientation, self._device)
        #     orientation = self._backend_utils.expand_dims(orientation, 0)
        # self._articulation_view.set_world_poses(position, orientation)

        # print(og.sim.is_playing())
        # handle = self._dc_interface.get_articulation(self.prim_path)
        # # orientation: scalar-first
        # quat_back = orientation[[1, 2, 3, 0]]
        # # set base state
        # base_pose = omni_dc.Transform(position, quat_back)
        # self._dc_interface.set_rigid_body_pose(handle, base_pose)
        # self._dc_interface.set_rigid_body_linear_velocity(handle, np.zeros(3))
        # self._dc_interface.set_rigid_body_angular_velocity(handle, np.zeros(3))
        

        position = np.asarray(position, dtype=np.float32).tolist()
        # xform = get_prim_at_path(self.prim_path)
        self._prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*position))
        # self._prim.GetAttribute('xformOp:translate').Set(translation)
        
        if self._prim.GetAttribute("xformOp:orient").GetTypeName() == "quatf":
            orientation = np.asarray(orientation, dtype=np.float16).tolist()
            rotq = Gf.Quatf(*orientation)
        else:
            orientation = np.asarray(orientation, dtype=np.float32).tolist()
            # print(*orientation)
            rotq = Gf.Quatd(*orientation)
        self._prim.GetAttribute("xformOp:orient").Set(rotq)
        
        # # orientation: scalar-first
        # quat_back = orientation[[1, 2, 3, 0]]
        # # set base state
        # base_pose = omni_dc.Transform(position, quat_back)
        # self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        # self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, np.zeros(3))
        # self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, np.zeros(3))


    # 6D
    def get_curr_position_orientation(self):
        # tmp_pose = self._dc_interface.get_rigid_body_pose(self._root_handle)
        # pos = np.asarray(tmp_pose.p, dtype=np.float32)
        # quat_back = np.asarray(tmp_pose.r, dtype=np.float32)
        # quat_first = quat_back[[3, 0, 1, 2]]
        # return pos, quat_first
        pos = np.array(self._prim.GetAttribute('xformOp:translate').Get())
        Gf_Quatd = self._prim.GetAttribute("xformOp:orient").Get()
        orientaton = np.array([Gf_Quatd.real, Gf_Quatd.imaginary[0], Gf_Quatd.imaginary[1], Gf_Quatd.imaginary[2]])

        return pos, orientaton
    
    def always_z_up(self, quat):
        _mat = R.from_quat(quat).as_matrix()
        _mat[:, 2] = np.array([0,0,1])
        _mat[2, 0] = _mat[2, 1] = 0
        _mat[:, 0] = _mat[:, 0] * np.linalg.norm(_mat[:, 0])
        _mat[:, 1] = _mat[:, 1] * np.linalg.norm(_mat[:, 1])
        _quat = R.from_matrix(_mat).as_quat()
        return _quat
    ######################
    ## control with keyboard
    def control_aliengo_keyboard(self, waypoint_pose=None):
        self._enter_toggled = 0
        self._base_command = [0.0, 0.0, 0.0, 0]
        self._event_flag = False

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [self.unit_move_velocity, 0.0, 0.0],
            "UP": [self.unit_move_velocity, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-self.unit_move_velocity, 0.0, 0.0],
            "DOWN": [-self.unit_move_velocity, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -self.unit_move_velocity, 0.0],
            "RIGHT": [0.0, -self.unit_move_velocity, 0.0],
            # right command
            "NUMPAD_4": [0.0, self.unit_move_velocity, 0.0],
            "LEFT": [0.0, self.unit_move_velocity, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, self.unit_turn_velocity],
            "N": [0.0, 0.0, self.unit_turn_velocity],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -self.unit_turn_velocity],
            "M": [0.0, 0.0, -self.unit_turn_velocity],
        }

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        og.sim.add_physics_callback("aliengo_move_keyboard", callback_fn=self.on_physics_step_keyboard)

        if waypoint_pose is None:
            self._path_follow = False
        else:
            self._qp_controller.waypoint_pose = waypoint_pose
            self._path_follow = True
        
    def on_physics_step_keyboard(self, dt):
        if self._event_flag:
            self._qp_controller.switch_mode()
            self._event_flag = False

        self.move(dt, self._base_command, self._path_follow, auto_start=True)


    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        [Summary]

        Keyboard subscriber callback to when kit is updated.
        
        """
        # reset event
        self._event_flag = False
        # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] += np.array(self._input_keyboard_mapping[event.input.name])
                self._event_flag = True

            # enter, toggle the last command
            if event.input.name == "ENTER" and self._enter_toggled is False:
                self._enter_toggled = True
                if self._base_command[3] == 0:
                    self._base_command[3] = 1
                else:
                    self._base_command[3] = 0
                self._event_flag = True

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] -= np.array(self._input_keyboard_mapping[event.input.name])
                self._event_flag = True
            # enter, toggle the last command
            if event.input.name == "ENTER":
                self._enter_toggled = False
        # since no error, we are fine :)
        return True
    
    ########################
    ## default

    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, _name):
        self._name = _name


    @property
    def prim_path(self):
        return self._prim_path
    
    
    @property
    def usd_path(self):
        assets_root_path = 'omniverse://localhost/DemoAgents'
        usd_path = assets_root_path + "/Aliengo_arm/a1.usda"
        if self.joint_num_bugfix:
            usd_path = os.path.joint(ASSERT_ROOT, "Aliengo_arm/a1.usda")
        else:
            usd_path = os.path.join(ASSERT_ROOT, "Aliengo_arm/aliengo-z1_fixed_base.usda")
        
        return usd_path

    @property
    def robot_arm_descriptor_yamls(self):
        descriptor_yaml_path = os.path.join(ASSERT_ROOT, "Aliengo_arm/z1_descriptor.yaml")
        return descriptor_yaml_path

    @property
    def urdf_path(self):
        assets_root_path = 'omniverse://localhost/DemoAgents'
        urdf_path = assets_root_path + "/Franka/franka.urdf"
        urdf_path = os.path.join(ASSERT_ROOT, "Aliengo_arm/aliengo-z1.urdf")
        
        return urdf_path


