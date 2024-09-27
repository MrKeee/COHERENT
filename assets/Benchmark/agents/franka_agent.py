
import numpy as np
import gym
import os
from copy import deepcopy
from typing import Optional, List
from collections.abc import Iterable
from pxr import Gf, Usd, UsdGeom, UsdShade, UsdPhysics
from omni.isaac.core.utils.rotations import gf_quat_to_np_array
from scipy.spatial.transform import Rotation as R
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import RMPFlowController, PickPlaceController
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage, get_stage_units
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import get_prim_property, set_prim_property, get_prim_parent, get_prim_at_path

import omnigibson as og
from omnigibson.controllers import create_controller
from omnigibson.controllers.controller_base import ControlType
from omnigibson.utils.python_utils import assert_valid_key, merge_nested_dicts

from omnigibson.utils.usd_utils import add_asset_to_stage
from omnigibson.sensors import create_sensor, SENSOR_PRIMS_TO_SENSOR_CLS, ALL_SENSOR_MODALITIES, VisionSensor, ScanSensor

from omnigibson.objects.usd_object import USDObject
from omnigibson.macros import gm

# from omnigibson.robots.robot_base import BaseRobot
# from omnigibson.robots.manipulation_robot import ManipulationRobot
from omnigibson.prims.prim_base import BasePrim
from omnigibson.prims.xform_prim import XFormPrim
from omni.isaac.franka import Franka, KinematicsSolver

from omni.isaac.core.robots.robot import Robot
import omni.isaac.core.prims as isaac_prim
import omni.usd

from constants import *

#############################################################################################
## Franka
#############################################################################################
class FrankaAgent(XFormPrim, Franka):
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
            fixed_base=True,
            prim_type=0,  # PrimType.RIGID

            # Franka
            position: Optional[np.ndarray] = None,
            orientation: Optional[np.ndarray] = None,  # scalar-rear
            end_effector_prim_name: Optional[str] = None,
            gripper_dof_names: Optional[List[str]] = None,
            gripper_open_position: Optional[np.ndarray] = None,
            gripper_closed_position: Optional[np.ndarray] = None,
            deltas: Optional[np.ndarray] = None,

            # Sensor
            obs_modalities="all",
            proprio_obs="default",
            cam_height=256,
            cam_width=256
        ):
        self._active = False

        self.name = name
        self._prim_path = "/World/" + self.name if prim_path is None else prim_path
        self._end_effector_prim_name =  self.eef_link_names if end_effector_prim_name is None else end_effector_prim_name
        self._gripper_dof_names = self.gripper_joint_names if gripper_dof_names is None else gripper_dof_names
        
        # full open is 0.05, cost 5 sim step to fully open a gripper when it is fully closed
        self._gripper_action_deltas = np.array([0.01, 0.01]) / get_stage_units() if deltas is None else deltas
        

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


        # self._usd_path = self.usd_path
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

        if gripper_open_position is None:
            gripper_open_position = np.array([0.05, 0.05]) / get_stage_units()
        if gripper_closed_position is None:
            gripper_closed_position = np.array([0.025, 0.025])
        # s_dc_interface, _articulation_view, _articulation_controller
        Franka.__init__(
            self,
            prim_path=prim_path,
            name=name,
            usd_path=self.usd_path,
            position=self._init_position,
            orientation=self._init_orientation,
            end_effector_prim_name=self._end_effector_prim_name,
            gripper_dof_names=self._gripper_dof_names,
            gripper_open_position=gripper_open_position,
            gripper_closed_position=gripper_closed_position,
            deltas=deltas,
        )

        self.attached_prim_path = None
        self.attached_prim_list = []
        self._agent_attach_prim = get_prim_at_path(self.prim_path+"/"+self.attach_link_names[0])
        # Inverse Kinematics Solver
        # self._ik_solver = RMPFlowController(name="franka_ik_controller", robot_articulation=self)
        # KinematicsSolver给定urdf文件和robot_description.yaml文件可以求解正逆运动学
        # 用的URDF文件和当前用的USD有一些出入，对应的eef_link_names为gripper_right
        self._ik_solver = KinematicsSolver(robot_articulation=self)
        # self.ee_dummy = self.create_ee_dummy()
        self._ik_solver.get_kinematics_solver().set_robot_base_pose(self._init_position, self._init_orientation)
    
    #########################
    @property
    def active(self):
        return self._active
    @active.setter
    def active(self, flag):
        self._active = flag
    
    #########################
    ## load, initialize, remove

    # 添加到Stage并获取prim，BasePrim load(), Franka 无load()
    def _load(self):
        """
        Load the object into pybullet and set it to the correct pose
        """
    
        # prim = add_asset_to_stage(asset_path=self._usd_path, prim_path=self._prim_path)
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
        BasePrim.remove(self)
        self._post_remove()

    def _post_remove(self):
        # Remove all sensors
        for sensor in self._sensors.values():
            sensor.remove()

    def update_physics_sim_view(self, physics_sim_view=None):
        # initialize _handle &　_root_handle

        self._articulation_view._is_initialized = False
        Franka.initialize(self, physics_sim_view)
        self.post_reset()
        # Franka.initialize(self, None)
        # pass

    def post_reset(self):
        super().post_reset()

        # reset
        self._active = False

    
    # invoked by og.sim.play() and cannot invoke og.sim.stop() after initialization
    def initialize(self, physics_sim_view=None):
        # Run super first
        # initialize _handle &　_root_handle
        Franka.initialize(self, physics_sim_view=physics_sim_view)
        self.post_reset()
        # invoke dump_state
        XFormPrim.initialize(self)

        # Initialize all sensors
        for sensor in self._sensors.values():
            sensor.initialize()
        
        # Cache links
        self.cache_links_with_check()
        # Cache joints
        self.cache_joints_with_check()


        # Setup action space
        # self._action_space = self._create_discrete_action_space() if self._action_type == "discrete" \
        #     else self._create_continuous_action_space()


        # self.reset()


        # # Load the observation space for this robot
        # self.load_observation_space()

        # # Validate this robot configuration
        # self._validate_configuration()

    def _dump_state(self):
        # We don't call super, instead, this state is simply the root link state and all joint states
        state = dict()
        link_state = dict()
        joint_state = dict()

        # pos, quat_scalar_first = self._articulation_view.get_world_poses()

        # link_state['pos'] = pos
        # link_state['ori'] = quat_scalar_first

        link_state['pos'] = self.get_position()
        link_state['ori'] = self.get_orientation()

        link_state['lin_vel'] = self.get_linear_velocity()
        link_state['ang_vel'] = self.get_angular_velocity()

        joint_state['pos'] = self.get_joint_positions()
        joint_state['vel'] = self.get_joint_velocities()
        joint_state['eff'] = self.get_joint_efforts()
        
        
        state["root_link"] = link_state
        state["joints"] = joint_state

        return state

    def _load_state(self, state):
        # Load base link state and joint states
        # self._articulation_view.set_world_poses(state["root_link"]['pos'], state["root_link"]['ori'])
        self.set_position(state["root_link"]['pos'])
        self.set_orientation(state["root_link"]['ori'])
        self.set_linear_velocity(state["root_link"]['lin_vel'])
        self.set_angular_velocity(state["root_link"]['lin_vel'])

        self.set_joint_positions(state["joints"]['pos'])
        self.set_joint_velocities(state["joints"]['vel'])
        self.set_joint_efforts(state["joints"]['eff'])


        # Make sure this object is awake
        self._dc_interface.wake_up_articulation(self._handle)

    def _serialize(self, state):
        # We serialize by first flattening the root link state and then iterating over all joints and
        # adding them to the a flattened array
        
        link_state = state["root_link"]
        joint_state = state["joints"]
        state_flat = [v for k, v in {**link_state, **joint_state}.items()]

        return np.concatenate(state_flat).astype(float)

    #######################
    ## 
    def get_ee_position_orientation(self):
        ee_pos, ee_mat = self._ik_solver.compute_end_effector_pose()
        quat_back = R.from_matrix(ee_mat).as_quat()
        ee_quat = np.array([quat_back[3], quat_back[0], quat_back[1], quat_back[2]])
        return ee_pos, ee_quat

    def get_arm_base_position_orientation(self):
        tmp_pose = self._dc_interface.get_rigid_body_pose(self._links[self.arm_link_names[0]]["handle"])
        arm_base_pos = np.asarray(tmp_pose.p, dtype=np.float32)
        quat_back = np.asarray(tmp_pose.r, dtype=np.float32)
        arm_base_ori = quat_back[[3, 0, 1, 2]]
        # scalar-first
        return arm_base_pos, arm_base_ori
    



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
            armattach2world = omni.usd.get_world_transform_matrix(self._agent_attach_prim)
            


            xform = get_prim_at_path(self.attached_prim_list[0])
            # print("111111111111111111111:", armattach2world.ExtractTranslation())
            xform.GetAttribute('xformOp:translate').Set(armattach2world.ExtractTranslation())
            # print("222222222222222222222:", xform.GetAttribute('xformOp:translate').Get())

            if xform.GetAttribute("xformOp:orient").GetTypeName() == "quatf":
                rotq = Gf.Quatf(1, 0, 0, 0)
            else:
                rotq = Gf.Quatd(1, 0, 0, 0)
            xform.GetAttribute("xformOp:orient").Set(rotq)
            
            xform = get_prim_at_path(self.attached_prim_list[1])
            xform.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0, 0, 0))


            if xform.GetAttribute("xformOp:orient").GetTypeName() == "quatf":
                rotq = Gf.Quatf(1, 0, 0, 0)
            else:
                rotq = Gf.Quatd(1, 0, 0, 0)
            xform.GetAttribute("xformOp:orient").Set(rotq)
            # print("Attach!!!!!!!!!!!!!!!!!!")
    ########################
    ## action space
    @property
    def action_space(self):
        """
        Action space for this object.

        Returns:
            gym.space: Action space, either discrete (Discrete) or continuous (Box)
        """
        return deepcopy(self._action_space)
    

    @property
    def discrete_action_list(self):
        # Not supported for this robot
        raise NotImplementedError()

    def _create_discrete_action_space(self):
        # Fetch does not support discrete actions
        raise ValueError("Fetch does not support discrete actions!")
    
    def _create_continuous_action_space(self):
        """
        Create a continuous action space for this object. By default, this loops over all controllers and
        appends their respective input command limits to set the action space.
        Any custom behavior should be implemented by the subclass (e.g.: if a subclass does not
        support this type of action space, it should raise an error).

        Returns:
            gym.space.Box: Object-specific continuous action space
        """
        # Action space is ordered according to the order in _default_controller_config control
        low, high = [], []
        for controller in self._controllers.values():
            limits = controller.command_input_limits
            low.append(np.array([-np.inf] * controller.command_dim) if limits is None else limits[0])
            high.append(np.array([np.inf] * controller.command_dim) if limits is None else limits[1])

        return gym.spaces.Box(shape=(self.action_dim,), low=np.concatenate(low), high=np.concatenate(high), dtype=float)

    
    ########################
    ## Alticulation
    @property
    def links(self):
        return self._links

    @property
    def joints(self):
        return self._joints
    

    def cache_links_without_check(self):
        # _dc_interface not ready

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
                # print(link_name, (self.arm_link_names + self.gripper_link_names)[link_num])
                assert link_name == self.link_names[link_num], \
                    "Inconsistent link order"
                # print("Cache link prim at ", link_prim_path)
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

        self._root_link_name = valid_root_links[0]
        
    def cache_links_with_check(self):
        # _dc_interface is ready
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

        self._root_link_name = valid_root_links[0]
        
        assert self._dc_interface.get_rigid_body_name(self._root_handle) == self._root_link_name, \
            "Inconsistent root prim"


    def cache_joints_with_check(self):
        assert self._handle is not None and self._dc_interface is not None, "Articulation is not initialized yet"


        # Initialize joints dictionary
        self._joints = dict()

        n_dof = self._dc_interface.get_articulation_dof_count(self._handle)
        assert n_dof == len(self.arm_joint_names + self.gripper_joint_names), "Inconsistent joint num"

        # Additionally grab DOF info if we have non-fixed joints
        if n_dof > 0:
            for i in range(n_dof):
                # dof is equivalent to non-fixed joint
                joint_handle = self._dc_interface.get_articulation_dof(self._handle, i)
                joint_name = self._dc_interface.get_dof_name(joint_handle)
                joint_path = self._dc_interface.get_dof_path(joint_handle)
                joint_prim = get_prim_at_path(joint_path)

                assert joint_name == (self.arm_joint_names + self.gripper_joint_names)[i], \
                    "Inconsistent joint order"

                self._joints[joint_name] = {
                    "joint_name": joint_name,
                    "prim": joint_prim,
                    "prim_path": joint_path,
                    "index": i,
                    "handle": self._dc_interface.find_articulation_dof(self._handle, joint_name)
                }
                print("Cache joint prim at ", joint_path)


    
    def set_attach_prim(self, attached_prim_path):
        # self.attached_prim_path = attached_prim_path
        self.attached_prim_list.append(attached_prim_path)
        self.attached_prim_list.append(attached_prim_path+'/base_link')
    

    def detach(self):
        self.attached_prim_path = None
        self.attached_prim_list = []
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
            armattach2world = omni.usd.get_world_transform_matrix(self._agent_attach_prim)

            xform = get_prim_at_path(self.attached_prim_list[0])
            # print("111111111111111111111:", armattach2world.ExtractTranslation())
            xform.GetAttribute('xformOp:translate').Set(armattach2world.ExtractTranslation())
            # print("222222222222222222222:", xform.GetAttribute('xformOp:translate').Get())

            xform = get_prim_at_path(self.attached_prim_list[1])
            xform.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0, 0, 0))
            # print("Attach!!!!!!!!!!!!!!!!!!")
            

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

    ########################
    ## gripper
    def is_gripper_fully_opened(self):

        current_position = self.gripper.get_joint_positions()
        target_position = self.gripper.joint_opened_positions
        if np.linalg.norm(current_position-target_position, ord=1) < 0.03:
            return True
        else:
            return False


    def create_ee_dummy(self):
        _quat_cube2world_back = R.from_euler('XYZ', [180, 0, 180], degrees=True)
        _quat_robot2world_back = R.from_quat([self._init_orientation[1], self._init_orientation[2], self._init_orientation[3], self._init_orientation[0]])
        _quat_cube2robot_back = (_quat_robot2world_back.inv() * _quat_cube2world_back).as_quat()
        _quat_cube2robot_front = np.array([_quat_cube2robot_back[-1], _quat_cube2robot_back[0], _quat_cube2robot_back[1], _quat_cube2robot_back[2]])
        
        # self.ee_dummy = isaac_prim.xform_prim.XFormPrim(
        #         prim_path=self.prim_path + "/ee_dummy",
        #         name="ee_dummy",
        #         position=self._init_position + np.array([0.5, 0, 0.5]),
        #         orientation=_quat_cube2robot_front,
        #         scale=np.array([0.02, 0.02, 0.02]),
        #         visible=False,
        #     )

        # og.sim.add_physics_callback("cube_follow", callback_fn=self.on_physics_step)

        return FixedCuboid(
                        prim_path="/World/target_cube",
                        name="target_cube",
                        position=self._init_position + np.array([0.5, 0, 0.5]),
                        orientation=_quat_cube2robot_front,
                        scale=np.array([0.02, 0.02, 0.02]),
                        color=np.array([0, 0, 1.0]),
                    )


    def on_physics_step(self, dt):
        # target_cube.disable_rigid_body_physics() 
        position, orientation = self.ee_dummy.get_world_pose()
        actions = self._ik_solver.forward(
            target_end_effector_position=position,
            target_end_effector_orientation=orientation,
        )
        self.apply_action(actions)

    def test(self):
        position, orientation = self.target_cube.get_world_pose()
        actions = self._ik_solver.forward(
            target_end_effector_position=position,
            target_end_effector_orientation=orientation,
        )
        self._agent.apply_action(actions)


    
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
        usd_path = assets_root_path + "/Franka/franka.usd"
        usd_path = os.path.join(ASSERT_ROOT, 'Franka/franka.usda')
        # usd_path = "/home/uav/Projs/ov/OmniGibson/omnigibson/data/assets/models/Franka/franka.usd"

        return usd_path

    @property
    def robot_arm_descriptor_yamls(self):
        return {self.default_arm: os.path.join(ASSERT_ROOT, 'Franka/panda_descriptor.yaml')}

    @property
    def urdf_path(self):
        assets_root_path = 'omniverse://localhost/DemoAgents'
        urdf_path = assets_root_path + "/Franka/franka.urdf"
        urdf_path = os.path.join(ASSERT_ROOT, 'Franka/panda_arm_hand.urdf')
        
        return urdf_path

    @property
    def link_names(self):
        return self.arm_link_names + self.gripper_link_names + self.attach_link_names
    
    
    
    @property
    def arm_link_names(self):
        return [
            "panda_link0",
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
            "panda_link7",
            "panda_link8"
        ]

    @property
    def attach_link_names(self):
        return [
            "attach_link"
        ]
    @property
    def arm_joint_names(self):
        return [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]

    @property
    def eef_link_names(self):
        return "panda_rightfinger"  # panda_rightfinger

    @property
    def gripper_link_names(self):
        return ["panda_hand", "panda_leftfinger", "panda_rightfinger"]

    @property
    def gripper_joint_names(self):
        return ["panda_finger_joint1", "panda_finger_joint2"]


    @property
    def map_to_dc_order(self):
        return self.control_idx[self.default_arm]


    @property
    def default_joint_pos(self):
        return np.zeros(self.n_joints)[self.map_to_dc_order]
    