import yaml
import os
import numpy as np
import math
from tkinter import _flatten
# source setup_python_env.sh
import carb
# import omni.kit.ui
# from omni.isaac.urdf import _urdf
import omnigibson as og
from omnigibson.macros import gm



from scipy.spatial.transform import Rotation as R
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.quadruped.utils.rot_utils import get_xyz_euler_from_quaternion, get_quaternion_from_euler

from omni.isaac.core.objects import DynamicCuboid, FixedCuboid

from agents import *

import sys
sys.path.append("/home/uav/Projs/ov/OmniGibson/hademo")
from agents import *


# Don't use GPU dynamics and use flatcache for performance boost
gm.USE_GPU_DYNAMICS = True
gm.ENABLE_FLATCACHE = True


#################################################
# Start with an empty configuration
cfg = dict()
cfg["scene"] = {
    "type": "Scene", 
    "floor_plane_visible": True, 
}
cfg["objects"] = [
    # {
    #     "type": "USDObject",
    #     "name": "door_duymuw_1",
    #     "usd_path": "/home/pjlab/workspace/OmniGibson/Benchmark/asserts/door_single/door_single.usda",
    #     "category": "door",
    #     "model": "duymuw",
    #     "position": [1.7745543718338013, 2.3250746726989746, 1.1510995626449585],
    #     "orientation": [0.0, 0.0, 0.70710678, 0.70710678],
    #     "fixed_base": True,
    #     "bounding_box": [0.1298, 0.6267, 1.9769]
    # }
]
cfg["robots"] = []
cfg["agents"] = [ 
    {
        "type": "AliengoAgent", 
        "name": "aliengo",
        "position": [1.7745543718338013, 3.1, 0.4],
        "rotation": get_quaternion_from_euler(np.array([0,0,-math.pi/2]))[[1,2,3,0]],  # scalar-first
        "scale": 1.0,
        "self_collision": False,
        "action_normalize": True,
        "action_type": "continuous",
        "obs_modalities": ["scan", "rgb", "depth"], 
    }
    # {
    #     "type": "FrankaAgent", 
    #     "name": "franka",
    #     "position": [1.7745543718338013, 2.94643, 0.4],
    #     "rotation": get_quaternion_from_euler(np.array([0,0,-math.pi/2])),  # scalar-first
    #     "scale": 1.0,
    #     "self_collision": False,
    #     "action_normalize": True,
    #     "action_type": "continuous",
    #     "obs_modalities": ["scan", "rgb", "depth"], 
    # }
]


cfg["task"] = {
    "type": "DummyTask", 
    "termination_config": dict(), 
    "reward_config": dict(), 
}

#################################################


def create_cube(
        cube_name, 
        position=np.array([0.5, 0, 0.85]), 
        orientation=np.array([1, 0, 0, 0]), 
        color=np.array([0, 0, 1.0])
    ):

    return FixedCuboid(
            prim_path="/World/" + cube_name,
            name=cube_name,
            position=position,
            orientation=orientation,
            scale=np.array([0.02, 0.02, 0.02]),
            color=color,
        )


#################################################



env = og.Environment(configs=cfg, physics_timestep=1/400., action_timestep=16/400.)
# env = og.Environment(configs=cfg, physics_timestep=1/60., action_timestep=1/60.)

og.sim.enable_viewer_camera_teleoperation()
# Update the simulator's viewer camera's pose so it points towards the robot
og.sim.viewer_camera.set_position_orientation(
    position=np.array([3.74020141, 0, 1.91041777-0.4]) + np.array(cfg["agents"][0]["position"]),
    orientation=(R.from_euler('y', -math.pi/6)*R.from_euler('XYZ', [math.pi/2, math.pi/2, 0])).as_quat(),
    # orientation=np.array([0,0,0,1]),  # (x,y,z,w)
)


aliengo = env.agents[0]



aliengo.wake_deault_gait_control()
aliengo.dog_active = False

iksolver = aliengo._ik_solver

target = create_cube(cube_name="targte", 
                     position=np.array([1.64934, 2.79713, 0.79164]),
                    #  position=np.array([1.61391, 2.42104, 0.853]),  # np.array([1.61391, 2.42104, 0.853]), 
                     orientation=np.array(cfg["agents"][0]["rotation"])[[3,0,1,2]])

# _aliengo_stand_still_state = aliengo.get_dog_pose()
_aliengo_stand_still_pos, _aliengo_stand_still_quat_first = aliengo.get_curr_position_orientation()
    

for i in range(100):
    aliengo.gripper.open()

    # aliengo.set_dog_pose(_aliengo_stand_still_state)

    aliengo.set_curr_position_orientation(_aliengo_stand_still_pos, _aliengo_stand_still_quat_first)
    og.sim.step()


opendoorpose = {
    "door_duymuw_0": [
                        [1.6, 2.8, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        [1.6, 2.75, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        [1.6, 2.7, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        [1.6, 2.65, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        [1.6, 2.6, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        [1.6, 2.55, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        [1.6, 2.5, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        # [1.7, 2.7, 0.83, 0.0, 0.0, -0.71051, 0.70369],
                        # [1.7, 2.6, 0.84, 0.0, 0.0, -0.71051, 0.70369],
                        # [1.7, 2.5, 0.85, 0.0, 0.0, -0.71051, 0.70369],
                        # [1.7, 2.4, 0.85, 0.0, 0.0, -0.71051, 0.70369],
                        # [1.7, 2.3, 0.85, 0.0, 0.0, -0.71051, 0.70369],
                        # [1.7, 2.2, 0.85, 0.0, 0.0, -0.71051, 0.70369]
                    ]
}



while og.app.is_running():

    arm_base_pos, arm_base_ori = aliengo.get_arm_base_position_orientation()
    iksolver.get_kinematics_solver().set_robot_base_pose(arm_base_pos, arm_base_ori)
    

    pos, ori = target.get_world_pose()
    # pos = arm_base_pos + np.array([0.2232989, 0.4447203, 0.77334772])
    # target.set_world_pose(pos, ori)
    
    # _poses = opendoorpose["door_duymuw_0"]
    # if len(_poses):
    #     pos = np.array(_poses[0][:3])
    #     ori = np.array(_poses[0][3:])[[3, 0,1,2]]
    actions, succ = iksolver.compute_inverse_kinematics(
        target_position=pos,
        target_orientation=ori,
    )
    # print(np.array(aliengo.get_joint_positions()))
    ee_pos, ee_ori = iksolver._kinematics.compute_forward_kinematics(iksolver.end_effector_frame_name, aliengo.get_arm_joint_positions())
    # print("Distance:", np.linalg.norm(ee_pos-pos))
    # if np.linalg.norm(ee_pos-pos) < 0.002:
    #     if len(opendoorpose["door_duymuw_0"]):
    #         opendoorpose["door_duymuw_0"].pop(0)
        # _poses[0][1] -= 0.1
        # print("Advance")
        # print(_poses[0])
    

    # print(actions.joint_positions)
    if succ:
        aliengo.apply_action(actions)
    else:
        carb.log_warn("IK did not converge to a solution.  No action is being taken.")

    # aliengo.set_dog_pose(_aliengo_stand_still_state)
    aliengo.set_curr_position_orientation(_aliengo_stand_still_pos, _aliengo_stand_still_quat_first)
    og.sim.step()





#########################################################
#initialize the controller

my_controller = KinematicsSolver(my_denso)

articulation_controller = my_denso.get_articulation_controller()

while simulation_app.is_running():

    my_world.step(render=True)

    if my_world.is_playing():

        if my_world.current_time_step_index == 0:

            my_world.reset()

        observations = my_world.get_observations()

        actions, succ = my_controller.compute_inverse_kinematics(

            target_position=observations[target_name]["position"],

            target_orientation=observations[target_name]["orientation"],

        )

        if succ:

            articulation_controller.apply_action(actions)

        else:

            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

simulation_app.close()