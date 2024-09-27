import sys
sys.path.append("/home/pjlab/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.quadrotor")


import numpy as np
import os
from omni.isaac.quadrotor.crazyflie import Crazyflie
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
import carb
import omni

###
from pxr import Gf
import omnigibson as og
from constants import *
from scipy.spatial.transform import Rotation as R
from agents.crazyflie import Crazyflie
from omnigibson.prims.xform_prim import XFormPrim
from copy import deepcopy
from typing import Optional, List
from collections.abc import Iterable
from omnigibson.sensors import create_sensor, SENSOR_PRIMS_TO_SENSOR_CLS, ALL_SENSOR_MODALITIES, VisionSensor, ScanSensor
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.quadrotor.controller import CFController
from omni.isaac.quadrotor.classutils import CFState, CFMeasurement, CFCommand
from omni.isaac.quadruped.utils.rot_utils import get_xyz_euler_from_quaternion, get_quaternion_from_euler

import omni.isaac.dynamic_control._dynamic_control as omni_dc



#############################################################################################
## Quadrotor
#############################################################################################
class QuadrotorAgent(XFormPrim, Crazyflie):
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

            # Quadrotor
            physics_dt: Optional[float] = 1 / 400.0,
            position: Optional[np.ndarray] = None,
            orientation: Optional[np.ndarray] = None,  # scalar-rear
            keyboard_mode=False,

            # Sensor
            obs_modalities="all",
            proprio_obs="default",
            cam_height=256,
            cam_width=256
        ):
        self._active = False
        self.keyboard_mode = keyboard_mode

        self.name = name
        self._prim_path = "/World/" + self.name if prim_path is None else prim_path
        
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

        Crazyflie.__init__(
            self,
            prim_path=prim_path,
            name=name,
            physics_dt=physics_dt,
            usd_path=self.usd_path,
            position=self._init_position,
            orientation=self._init_orientation,
        )


        self.attached_prim_path = None
        self.attached_prim_list = []
        self._agent_attach_prim = get_prim_at_path(self.prim_path+"/"+self.attach_link_names[0])
        # Controller
        self._controller = CFController(_simulate_dt=physics_dt)
        self._controller.setup()


        self._dof_control_modes: List[int] = list()


    #########################
    @property
    def active(self):
        return self._active
    @active.setter
    def active(self, move=True):
        self._active = move

        if move:
            self._controller.set_target_command(self.move_velocity)
        else:
            self._controller.set_target_command(self.static_velocity)

    

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
        # physics_sim_view = None
        self._articulation_view._is_initialized = False
        Crazyflie.initialize(self, physics_sim_view)

        self.post_reset()
        
    # invoked by og.sim.play() and cannot invoke og.sim.stop() after initialization
    def initialize(self, physics_sim_view=None):
        # Run super first
        # initialize _handle &　_root_handle
        Crazyflie.initialize(self, physics_sim_view=physics_sim_view)
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

    def _dump_state(self):
        # We don't call super, instead, this state is simply the root link state and all joint states
        return self.default_state
        

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
        self._active = False
        
        self._controller.reset()
        self.set_state(self.default_state)
        return
    

    ########################
    ## Alticulation
    @property
    def links(self):
        return self._links

    @property
    def joints(self):
        return self._joints


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
                    "Inconsistent link order, %s!=%s" % (link_name, self.link_names[link_num])
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

        self._root_link_name = "body"
        
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

        self._root_link_name = "body"
        
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

                # assert joint_name == (self.arm_joint_names + self.finger_joint_names)[i], \
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



    #####################
    ## rotor control 
    def wake_deault_rotor_control(self):
        # still 
        self.active = False
        og.sim.add_physics_callback("quadrotor_move", callback_fn=self.on_physics_step)

    def on_physics_step(self, dt):
        
        self.update()
        if dt is None:
            dt = self.physics_dt
        if self.active:
            self._controller.set_target_command(self.move_velocity)
        else:
            self._controller.set_target_command(self.static_velocity)


        self._command.desired_rotor_thrust, self._command.desired_joint_velocity = self._controller.advance(dt, self._measurement)
        self._state.joint_vel = np.asarray(self._command.desired_joint_velocity, dtype=np.float32)

        
        self.set_joint_velocities(velocities=np.asarray(self._command.desired_joint_velocity, dtype=np.float32))
        

        # if self._prev_fly_flag != self._fly_flag:
        #     self._default_hover_state = deepcopy(self._state)
        #     if self._prev_fly_flag in [3, 4, 5, 6]:
        #         self._default_hover_state.base_frame.quat = self.temp_quat
            
        #     self._default_hover_state.base_frame.quat = self.always_z_up(self._default_hover_state.base_frame.quat)
        


    ## pose control
    # (x, y, yaw)
    def set_body_pose(self, pose):# self._dc_interface: dynamic control, 
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
        # self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        # self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, np.zeros(3))
        # self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, np.zeros(3))


        self.set_world_poses(pos, quat_first)

    # (x, y, yaw)
    def get_body_pose(self):
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

        # handle = self._dc_interface.get_articulation(self.prim_path)
        # print(self._prim_path)
        # og.sim.step()
        # handle = self._dc_interface.get_rigid_body(self._prim_path)
        # # self._dc_interface.wake_up_articulation(handle)
        # # orientation: scalar-first
        # quat_back = orientation[[1, 2, 3, 0]]
        # # set base state
        # base_pose = omni_dc.Transform(position, quat_back)
        
        # self._dc_interface.set_rigid_body_pose(handle, base_pose)
        # self._dc_interface.set_rigid_body_linear_velocity(handle, np.zeros(3))
        # self._dc_interface.set_rigid_body_angular_velocity(handle, np.zeros(3))




        # orientation: scalar-first
        # quat_back = orientation[[1, 2, 3, 0]]
        # # set base state
        # base_pose = omni_dc.Transform(position, quat_back)
        # self._dc_interface.set_rigid_body_pose(self._root_handle, base_pose)
        # self._dc_interface.set_rigid_body_linear_velocity(self._root_handle, np.zeros(3))
        # self._dc_interface.set_rigid_body_angular_velocity(self._root_handle, np.zeros(3))


        # print(*list(position))
        # translation = Gf.Vec3d(position)
        
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


        # print("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")





    # 6D
    def get_curr_position_orientation(self):
        # tmp_pose = self._dc_interface.get_rigid_body_pose(self._root_handle)
        # pos = np.asarray(tmp_pose.p, dtype=np.float32)
        # quat_back = np.asarray(tmp_pose.r, dtype=np.float32)
        # quat_first = quat_back[[3, 0, 1, 2]]

        pos = np.array(self._prim.GetAttribute('xformOp:translate').Get())
        Gf_Quatd = self._prim.GetAttribute("xformOp:orient").Get()
        orientaton = np.array([Gf_Quatd.real, Gf_Quatd.imaginary[0], Gf_Quatd.imaginary[1], Gf_Quatd.imaginary[2]])

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


    #####################
    ## control with keyboard
    def control_quadrotor_keyboard(self, way_points=None):
        self._enter_toggled = 0
        self._base_command = [0.0, 0.0, 0.0, 0.0]
        self._event_flag = False

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # hover command
            "SPACE": 0,
            # up command
            "N": 1,
            # down command
            "M": 2,
            # forward command
            "UP": 3,
            # back command
            "DOWN": 4,
            # left command
            "LEFT": 5,
            # right command
            "RIGHT": 6,
            # clockwise command
            "J": 7,
            # counter-clockwise command
            "K": 8
        }

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        
        

        self._default_hover_state = CFState()
        self._default_landing_state = CFState()
        self._default_takeoff_state = CFState()
        self._default_forward_state = CFState()
        self._default_backward_state = CFState()
        self._default_left_state = CFState()
        self._default_right_state = CFState()
        self._default_clockwise_state = CFState()
        self._default_counterclockwise_state = CFState()
        
        og.sim.add_physics_callback("quadrotor_move_keyboard", callback_fn=self.on_physics_step_keyboard)

        if way_points is None:
            self._path_follow = False
        else:
            self._path_follow = True
        
    def on_physics_step_keyboard(self, dt):
        if self._event_flag:
            # self._agent._controller.switch_mode()
            self._event_flag = False

        self.move_keyboard()

        self.robot_info_print()
        # self._agent.initialize()


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
                self._agent.fly_state = self._input_keyboard_mapping[event.input.name]


        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            self._agent.fly_state = self._input_keyboard_mapping["SPACE"]
        # since no error, we are fine :)
        return True

    
    def move_keyboard(self):
        self._dc_interface.wake_up_articulation(self._handle)
        self.update()

        
        if self._fly_flag == 0:
            self.hover()
        elif self._fly_flag == 1:
            self.takeoff()
        elif self._fly_flag == 2:
            self.landing()
        elif self._fly_flag == 3:
            self.forward()
        elif self._fly_flag == 4:
            self.backward()
        elif self._fly_flag == 5:
            self.left()
        elif self._fly_flag == 6:
            self.right()
        elif self._fly_flag == 7:
            self.clockwise()
        elif self._fly_flag == 8:
            self.counterclockwise()
        else:
            assert False, "No Impletementation"



    def _move(self, dt, goal_velocity, path_follow=False, auto_start=True) -> np.ndarray:
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
        if goal_velocity is None:
            goal_velocity = self._goal_velocity
        else:
            self._goal_velocity = goal_velocity

        self._dc_interface.wake_up_articulation(self._handle)
        self.update()
        self._controller.set_target_command(goal_velocity)

        # dim: (self.joint_num_dog, )
        self._command.desired_rotor_thrust, self._command.desired_joint_velocity = self._controller.advance(dt, self._measurement)

        # we convert controller order to DC order for command torque

        # dc_torque_reorder = np.zeros_like(self._command.desired_joint_torque, dtype=np.float32)
        # dc_torque_reorder[self.to_qd_order] = self._command.desired_joint_torque

        dc_thrust_reorder = np.zeros_like(self._command.desired_rotor_thrust, dtype=np.float32)
        dc_thrust_reorder[self.to_qd_order] = self._command.desired_rotor_thrust
        dc_velocity_reorder = np.zeros_like(self._command.desired_joint_velocity, dtype=np.float32)
        dc_velocity_reorder[self.to_qd_order] = self._command.desired_joint_velocity
        

        # dc_torque_reorder = np.array([ 0.12419461, 0.12419461,  0.12419461, 0.12419461], dtype=np.float32)
        # dc_torque_reorder = np.array([ 0.02, 0.02,  0.0, 0.0], dtype=np.float32)
        # dc_velocity_reorder = np.array([ 33.3, -133.3,  233.3, -333.3], dtype=np.float32)
        self.set_joint_velocities(velocities=np.asarray(dc_velocity_reorder, dtype=np.float32))
        
        # self.set_joint_efforts(efforts=np.asarray(dc_torque_reorder, dtype=np.float32))

        # apply actions
        # self.physics_body.apply_forces(-1.0*self._controller.gravity)
        for i in range(4):
            self.physics_rotors[i].apply_forces(dc_thrust_reorder[i])
        
        return self._command

    def simulate_control(self, move=True):
        if move:
            self._controller.set_target_command(self.move_velocity)
        else:
            self._controller.set_target_command(self.static_velocity)

        self._command.desired_rotor_thrust, self._command.desired_joint_velocity = self._controller.advance(self.physics_dt, self._measurement)
        
        state = eval("self._default_%s_state" % self.fly_state_name)
        self.set_body_pose_from_state(state)

        self.set_joint_velocities(velocities=np.asarray(self._command.desired_joint_velocity, dtype=np.float32))

        self._prev_fly_flag = self._fly_flag


    def hover(self):
        if self._prev_fly_flag != self._fly_flag:
            self._default_hover_state = deepcopy(self._state)
            if self._prev_fly_flag in [3, 4, 5, 6]:
                self._default_hover_state.base_frame.quat = self.temp_quat
            
            self._default_hover_state.base_frame.quat = self.always_z_up(self._default_hover_state.base_frame.quat)
        
        
        if(self._default_hover_state.base_frame.pos[2]<=0.05):
            self._default_hover_state.joint_pos = self._default_state.joint_pos
            self._default_hover_state.joint_vel = self._default_state.joint_vel
            self.simulate_control(move=False)
        else:
            self.simulate_control(move=True)

    def always_z_up(self, quat):
        _mat = R.from_quat(quat).as_matrix()
        _mat[:, 2] = np.array([0,0,1])
        _mat[2, 0] = _mat[2, 1] = 0
        _mat[:, 0] = _mat[:, 0] * np.linalg.norm(_mat[:, 0])
        _mat[:, 1] = _mat[:, 1] * np.linalg.norm(_mat[:, 1])
        _quat = R.from_matrix(_mat).as_quat()
        return _quat


    def takeoff(self, delta=0.02):
        if self._prev_fly_flag != self._fly_flag:
            self._default_takeoff_state = deepcopy(self._state)
        self._default_takeoff_state.base_frame.pos[2] += delta


        self.simulate_control(move=True)

    def landing(self, delta=0.02):
        if self._prev_fly_flag != self._fly_flag:
            self._default_landing_state = deepcopy(self._state)
        
        self._default_landing_state.base_frame.pos[2] -= delta
        self._default_landing_state.base_frame.pos[2] = max(self._default_landing_state.base_frame.pos[2], 0.02)
        if(self._default_landing_state.base_frame.pos[2]<=0.02):
            self._default_landing_state.joint_pos = self._default_state.joint_pos
            self._default_landing_state.joint_vel = self._default_state.joint_vel
            self.simulate_control(move=False)
        else:
            self.simulate_control(move=True)

    def forward(self, delta=0.02):
        if self._prev_fly_flag != self._fly_flag:
            self._default_forward_state = deepcopy(self._state)
            self.temp_quat = self._default_forward_state.base_frame.quat
            self._default_forward_state.base_frame.quat = (R.from_quat(self._default_forward_state.base_frame.quat) * R.from_quat(R.from_euler('y', INCLINATION, degrees=True).as_quat())).as_quat()
        
        delta_pos = np.matmul(R.from_quat(self.temp_quat).as_matrix(), np.array([delta, 0, 0]).reshape(3, 1)).flatten()
        self._default_forward_state.base_frame.pos += delta_pos
        self.simulate_control(move=True)

    

    def backward(self, delta=0.02):
        if self._prev_fly_flag != self._fly_flag:
            self._default_backward_state = deepcopy(self._state)
            self.temp_quat = self._default_backward_state.base_frame.quat
            self._default_backward_state.base_frame.quat = (R.from_quat(self._default_backward_state.base_frame.quat) * R.from_quat(R.from_euler('y', -INCLINATION, degrees=True).as_quat())).as_quat()
        
        
        delta_pos = np.matmul(R.from_quat(self.temp_quat).as_matrix(), np.array([-delta, 0, 0]).reshape(3, 1)).flatten()
        self._default_backward_state.base_frame.pos += delta_pos

        self.simulate_control(move=True)



    def left(self, delta=0.02):
        if self._prev_fly_flag != self._fly_flag:
            self._default_left_state = deepcopy(self._state)
            self.temp_quat = self._default_left_state.base_frame.quat
            self._default_left_state.base_frame.quat = (R.from_quat(self._default_left_state.base_frame.quat) * R.from_quat(R.from_euler('x', -INCLINATION, degrees=True).as_quat())).as_quat()

        
        delta_pos = np.matmul(R.from_quat(self.temp_quat).as_matrix(), np.array([0, delta, 0]).reshape(3, 1)).flatten()
        self._default_left_state.base_frame.pos += delta_pos
        self.simulate_control(move=True)



    def right(self, delta=0.02):
        if self._prev_fly_flag != self._fly_flag:
            self._default_right_state = deepcopy(self._state)
            self.temp_quat = self._default_right_state.base_frame.quat
            self._default_right_state.base_frame.quat = (R.from_quat(self._default_right_state.base_frame.quat) * R.from_quat(R.from_euler('x', INCLINATION, degrees=True).as_quat())).as_quat()

        delta_pos = np.matmul(R.from_quat(self.temp_quat).as_matrix(), np.array([0, -delta, 0]).reshape(3, 1)).flatten()
        self._default_right_state.base_frame.pos += delta_pos
        self.simulate_control(move=True)


    def clockwise(self, delta=1):
        if self._prev_fly_flag != self._fly_flag:
            self._default_clockwise_state = deepcopy(self._state)
        
        self._default_clockwise_state.base_frame.quat = (R.from_quat(self._default_clockwise_state.base_frame.quat) * R.from_quat(R.from_euler('z', -delta, degrees=True).as_quat())).as_quat()
        self.simulate_control(move=True)

    def counterclockwise(self, delta=1):
        if self._prev_fly_flag != self._fly_flag:
            self._default_counterclockwise_state = deepcopy(self._state)
        
        self._default_counterclockwise_state.base_frame.quat = (R.from_quat(self._default_counterclockwise_state.base_frame.quat) * R.from_quat(R.from_euler('z', delta, degrees=True).as_quat())).as_quat()
        self.simulate_control(move=True)


    def robot_info_print(self):

        position, orientation = self._agent.get_world_pose()
        print("[" + self.name + "] fly state: " + self._agent.fly_state_name)
        print("[" + self.name + "] position is : " + str(position))
        print("[" + self.name + "] orientation is : " + str(orientation))
        print("[" + self.name + "] Num of degrees of freedom: " + str(self._agent.num_dof)) # prints 2
        
        print("[" + self.name + "] Joint names: " + str(self._agent.dof_names))
        print("[" + self.name + "] Joint Positions: " + str(self._agent.get_joint_positions()/3.14*180))
        print("[" + self.name + "] Joint Velocities: " + str(self._agent.get_joint_velocities()))
        print("[" + self.name + "] Joint Efforts: " + str(self._agent.get_applied_joint_efforts()))
        print("------------------------------------------")

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
        usd_path = assets_root_path + "/Crazyflie/cf2x.usda"
        usd_path = os.path.join(ASSERT_ROOT, 'Crazyflie/cf2x_with_basket.usda')

        return usd_path

    @property
    def robot_arm_descriptor_yamls(self):
        assert False
        return {self.default_arm: "/home/uav/Desktop/1/Franka/panda_descriptor.yaml"}

    @property
    def urdf_path(self):
        assert False
        assets_root_path = 'omniverse://localhost/DemoAgents'
        urdf_path = assets_root_path + "/Franka/franka.urdf"
        urdf_path = os.path.join(ASSERT_ROOT, 'Franka/panda_arm_hand.urdf')
        return urdf_path


