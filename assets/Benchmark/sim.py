import numpy as np
import math
import argparse
from tkinter import _flatten
import omnigibson as og
from omnigibson.macros import gm
import rospy

from omni.isaac.core.utils.prims import get_prim_at_path
from scipy.spatial.transform import Rotation as R
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.quadruped.utils.rot_utils import get_xyz_euler_from_quaternion, get_quaternion_from_euler

from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import get_current_stage



from haskillset import SkillSet, LogSet
import worldmodel
from constants import *
from tasks import TASKSET, TASK_OCCUPANCY_MAP_CONFIG
from agents import *
import time
from tqdm import tqdm
duration = 5
# Don't use GPU dynamics and use flatcache for performance boost
gm.USE_GPU_DYNAMICS = True
# gm.ENABLE_FLATCACHE = False
gm.ENABLE_HQ_RENDERING = True


from ros_hademo_ws.src.hademo.src.action_subscriber import ResultPublishandActionSubscribe


from multiprocessing import Process  
import omni
from omni.isaac.occupancy_map import _occupancy_map


# def get_occupancy_map(task_name):
#     physx = omni.physx.acquire_physx_interface()
#     stage_id = omni.usd.get_context().get_stage_id()

#     generator = _occupancy_map.Generator(physx, stage_id)
#     # 0.05m cell size, output buffer will have 4 for occupied cells, 5 for unoccupied, and 6 for cells that cannot be seen
#     # this assumes your usd stage units are in m, and not cm
#     generator.update_settings(1, 0, 255, 255)
#     print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#     # Set location to map from and the min and max bounds to map to
#     if "Merom_1_int" in task_name:
#         generator.set_transform((0, 0, 0), (-2, -2, 1), (5, 9, 2))
#         '''
#             Top Left: (4.925000286102295, -1.975)		 Top Right: (-1.975, -1.975)
#             Bottom Left: (4.925000286102295, 8.924999809265136)		 Bottom Right: (-1.975, 8.924999809265136)
#             Coordinates of top left of image (pixel 0,0) as origin, + X down, + Y right:
#             (1.975, 4.925000286102295)
#             Image size in pixels: 139, 219
#         '''
#     generator.generate2d()
#     # Get locations of the occupied cells in the stage
#     points = generator.get_occupied_positions()
#     # Get dimensions for 2d buffer
#     dims = generator.get_dimensions()
#     print(dims)
#     # Get computed 2d occupancy buffer
#     buffer = generator.get_buffer()
#     print(np.array(buffer).shape)
#     print("***********************************************************")




# def create_cube(agent, cube_name, color=np.array([0, 0, 1.0])):
#     # _quat_cube2world_back = R.from_euler('XYZ', [-180, 0, 180], degrees=True)
    
    
#     if agent.name == "franka":
#         base_pos = agent._init_position
#         _quat_cube2world_front = euler_angles_to_quat(np.array([-np.pi, 0, np.pi]))
#     if agent.name == "aliengo":
#         base_pos, base_ori = agent.get_arm_base_position_orientation()
#         _quat_cube2world_front = base_ori

#     return FixedCuboid(
#             prim_path="/World/" + cube_name,
#             name=cube_name,
#             position=base_pos + np.array([0.5, 0.3, 0.5]),
#             orientation=_quat_cube2world_front,
#             scale=np.array([0.02, 0.02, 0.02]),
#             color=color,
#         )



class HASimulationSystem:
    def __init__(self, task_name, env):
        self.task_name = task_name
        self.env = env
        

        if len(env.agents) == 0:
            while True:
                og.sim.step()
        
        self.world_entity_name = self.getWorldEntityName(env.config)
        # print(self.world_entity_name)
        # print("+++++++++++++++++++++++++++++++++++++++++++++")
        for _ in range(50):
            # step simulation
            og.sim.step()

        with tqdm(total=duration, desc="Wait 5 sec before the sim start.", unit="s") as pbar:
            start_time = time.time()
            while True:

                og.sim.step()
                self.update_worldmodel()
                elapsed_time = time.time() - start_time
                pbar.update(elapsed_time - pbar.n)
                
                # Check if 5 seconds have passed
                if elapsed_time > duration:
                    print("5 seconds have passed, exiting loop.")
                    break
        self.update_worldmodel()

        self.skillset = SkillSet(self.task_name, env.agents)
        print(env.agents)
        # input('stop')
        self.logger = LogSet()


        rospy.init_node('SimNode', anonymous=True)
        self.simnode = ResultPublishandActionSubscribe(task_name)


        tmp = 0
        _tmp = 0
        flag = False
        while og.app.is_running():

            next_step_action = self.simnode._next_step_action
            if next_step_action:
                print(next_step_action)
                print("----------------------------------------------------------------")
                feedback_result = self.running_next_step_action(next_step_action)
                self.logger.action_round_plus1()

                self.update_worldmodel()
                self.simnode.publish_feedback_result(feedback_result)
                
            else:
                for aliengo_name in self.skillset.aliengo_name_list:
                    self.skillset.aliengo_dog_stand_still(aliengo_name)

                for quadrotor_name in self.skillset.quadrotor_name_list:
                    self.skillset.quadrotor_hover(quadrotor_name)
            if flag:
                ocm_p = Process(target=get_occupancy_map, args=(task_name, ))
                ocm_p.start()
            
            self.skillset.attach()

            # step simulation
            og.sim.step()
            

        rospy.spin()

    def getWorldEntityName(self, env_config):
        robot_config = env_config['robots']
        agent_config = env_config['agents']
        object_config = env_config['objects']


        world_entity_name = []

        for i in range(len(robot_config)):
            entity = robot_config[i]
            world_entity_name.append(entity['name'])
            
        for i in range(len(agent_config)):
            entity = agent_config[i]
            world_entity_name.append(entity['name'])
            
        for i in range(len(object_config)):
            entity = object_config[i]
            world_entity_name.append(entity['name'])
            
        return world_entity_name

    def update_worldmodel(self):
        
        state = dict()

        # print('******Current World Model***********')
        for entity_name in self.world_entity_name:
            prim_path = '/World/' + entity_name
            xform = get_current_stage().GetPrimAtPath(prim_path)
            translate = xform.GetAttribute('xformOp:translate').Get()
            orient = xform.GetAttribute('xformOp:orient').Get()  # scalar-first
            quat_imaginery = np.array(orient.GetImaginary())
            # print(orient.GetReal())
            # print(orient.GetImaginary())
            # print(np.array(orient))
            # print(np.array(orient)[0])
            # import time
            # time.sleep(3)
            state[entity_name] = [np.array(translate)[0], np.array(translate)[1], np.array(translate)[2], 
                                  quat_imaginery[0], quat_imaginery[1], quat_imaginery[2], orient.GetReal()]
            
            # print(xform.GetAuthoredAttributes())
            # print('translate:', xform.GetAttribute('xformOp:translate').Get())
            # print('orient:', xform.GetAttribute('xformOp:orient').Get())
            
            # print('entity name', entity_name)
            # print('translate:', translate)
            # print('orient (scalar-first):', orient)
            # print(state[entity_name])

            if "aliengo" in entity_name:
                arm_base_pos, arm_base_ori = self.env.agents[1].get_arm_base_position_orientation()
                arm_base_pos = arm_base_pos.tolist()
                arm_base_ori = arm_base_ori.tolist()
                state[entity_name].extend([arm_base_pos[0], arm_base_pos[1], arm_base_pos[2], 
                                           arm_base_ori[1], arm_base_ori[2], arm_base_ori[3], arm_base_ori[0]])

              
        worldmodel.set_state(state, self.task_name)

    

    def running_next_step_action(self, next_step_action):
        action_num = 0
        finish_flag = {}
        feedback_result = {}
        # print(next_step_action)
        for agent_name in self.skillset.agents_dict.keys():
            if len(next_step_action[agent_name]):
                action_num += 1
            finish_flag[agent_name] = False if len(next_step_action[agent_name]) else True
            feedback_result[agent_name] = {}
                
        assert action_num, "current recerived next_step_action is empty"
        # if action_num == 0:
            
        #     # Run simulation step
        #     og.sim.step()
        ########################################
        ## 
        print("============== Action Round %d ==============" % self.logger.action_round)
        print(finish_flag)
        while action_num:
            

            for agent_name, agent_finish_flag in finish_flag.items():
                if not agent_finish_flag:
                    func_name, func_args = next_step_action[agent_name]
                    print('next_step_action:', next_step_action)
                    done, success, info = eval(f"self.skillset.{func_name}")(**func_args)
                    print('skillset:', func_name, 'done:', done, 'success:', success, 'info:', info,'\n')

                    if "waypoint_ind" in func_args.keys():
                        func_args["waypoint_ind"] = info["waypoint_ind"]
                    if done:
                        finish_flag[agent_name] = True
                        feedback_result[agent_name] = {"has_result": True, "success": success, "info": self.logger.get_single_agent_result_info(success, info)}
                        print("%s: %s%s" % (func_name, "success" if success else "failed, ", "" if success else self.logger.get_single_agent_result_info(success, info)))
                        
                        if "franka" in agent_name:
                            self.skillset.reset_franka_waypoint_ind(agent_name)
                        if "aliengo" in agent_name:
                            self.skillset.reset_aliengo_waypoint_ind(agent_name)
                        if "quadrotor" in agent_name:
                            self.skillset.reset_quadrotor_waypoint_ind(agent_name)
                            
                        action_num -= 1
                else:
                    # If there is no action for the aliengo/quadrotor, then remain stationary.
                    if "aliengo" in agent_name:
                        self.skillset.aliengo_dog_stand_still(agent_name)
                    if "quadrotor" in agent_name:
                        self.skillset.quadrotor_hover(agent_name)


            # Run simulation step
            og.sim.step()

        return feedback_result

    # def test_with_defined_next_step_action(self):
    #     ########################################
    #     ## Franka
    #     if self.skillset.franka:
    #         franka_target_cube = create_cube(self.skillset.franka, cube_name="franka_target_cube", color=np.array([0, 0, 1.0]))
    #         franka_cube_position, franka_cube_orientation = franka_target_cube.get_world_pose()

    #         franka_waypoint_pos = np.array([
    #             franka_cube_position
    #         ])
    #         franka_waypoint_ori = np.array([
    #             franka_cube_orientation
    #         ])

    #     ########################################
    #     ## Aliengo
    #     if self.skillset.aliengo:
    #         aliengo_dog_waypoint_pos = np.array([
    #             self.skillset.aliengo._init_position + np.array([0.5, 0, 0]),
    #             self.skillset.aliengo._init_position + np.array([1, 0.5, 0])
    #         ])

    #         aliengo_dog_waypoint_ori = np.array([
    #             get_quaternion_from_euler(np.array([0.0, 0.0, 0.0])),
    #             get_quaternion_from_euler(np.array([0.0, 0.0, 45.0/180*math.pi])),
    #         ])

    #         aliengo_target_cube = create_cube(self.skillset.aliengo, cube_name="aliengo_target_cube", color=np.array([0, 1.0, 0.0]))
    #         aliengo_cube_position, aliengo_cube_orientation = aliengo_target_cube.get_world_pose()

    #         aliengo_arm_waypoint_pos = np.array([
    #             aliengo_cube_position
    #         ])
    #         aliengo_arm_waypoint_ori = np.array([
    #             aliengo_cube_orientation
    #         ])


    #     ########################################
    #     ## Quadrotor
    #     if self.skillset.quadrotor:
    #         quadrotor_waypoint_pos = np.array([
    #             self.skillset.quadrotor._init_position+np.array([0.5, 0, self.skillset.quadrotor.default_hover_height]),
    #             self.skillset.quadrotor._init_position+np.array([1.0, 0.5, self.skillset.quadrotor.default_hover_height]),
    #         ])

    #         quadrotor_waypoint_ori = np.array([
    #             self.skillset.quadrotor._init_orientation,
    #             (R.from_quat(R.from_euler('z', 45, degrees=True).as_quat()) * R.from_quat(self.skillset.quadrotor._init_orientation[[1, 2, 3, 0]])).as_quat()[[3, 0, 1, 2]]
    #         ])



    #     ########################################
    #     ## Define action_list
    #     next_step_action = {
    #         # "franka": [],
    #         # ["franka_open_gripper", {}],
    #         # ["franka_close_gripper", {}],
    #         "franka": ["franka_pick", {"waypoint_pos": franka_waypoint_pos, "waypoint_ori": franka_waypoint_ori, "waypoint_ind": 0}],
    #         # ["franka_place", {"waypoint_pos": franka_waypoint_pos, "waypoint_ori": franka_waypoint_ori, "waypoint_ind": 0}],
            
    #         "aliengo": ["aliengo_dog_move_with_waypoints", {"waypoint_pos": aliengo_dog_waypoint_pos, "waypoint_ori": aliengo_dog_waypoint_ori, "waypoint_ind": 0}],
    #         # ["aliengo_arm_open_gripper", {}],
    #         # ["aliengo_arm_close_gripper", {}],
    #         # ["aliengo_arm_pick", {"waypoint_pos": aliengo_arm_waypoint_pos, "waypoint_ori": aliengo_arm_waypoint_ori, "waypoint_ind": 0}],
    #         # ["aliengo_arm_place", {"waypoint_pos": aliengo_arm_waypoint_pos, "waypoint_ori": aliengo_arm_waypoint_ori, "waypoint_ind": 0}],
            
    #         # "quadrotor": []
    #         # ["quadrotor_takeoff", {}]
    #         # ["quadrotor_land", {}]
    #         "quadrotor": ["quadrotor_move_with_waypoints", {"waypoint_pos": quadrotor_waypoint_pos, "waypoint_ori": quadrotor_waypoint_ori, "waypoint_ind": 0}]
    #     }

    #     finish_flag = {
    #         "franka": False if len(next_step_action["franka"]) else True,
    #         "aliengo": False if len(next_step_action["aliengo"]) else True,
    #         "quadrotor": False if len(next_step_action["quadrotor"]) else True
    #     }

    #     # has_result: bool, success: bool, errorInfo: string
    #     feedback_result = {
    #         "franka": {},
    #         "aliengo": {},
    #         "quadrotor": {}
    #     }

    #     ########################################
    #     ## 
    #     print("Simulation Begin!")
    #     # Step!


    #     for i in range(100000):
            
    #         # 没有针对 aliengo / quadrotor 的动作，则保持不动
    #         if finish_flag["aliengo"]:
    #             self.skillset.aliengo_dog_stand_still()
    #         if finish_flag["quadrotor"]:
    #             self.skillset.quadrotor_hover()

    #         for agent_name, agent_finish_flag in finish_flag.items():
    #             if not agent_finish_flag:
    #                 func_name, func_args = next_step_action[agent_name]
    #                 done, success, info = eval(f"self.skillset.{func_name}")(**func_args)
    #                 if "waypoint_ind" in func_args.keys():
    #                     func_args["waypoint_ind"] = info["waypoint_ind"]
    #                 if done:
    #                     finish_flag[agent_name] = True
    #                     feedback_result[agent_name] = {"has_result": True, "success": success, "info": self.skillset.get_single_agent_result_info(success, info)}
    #                     print("%s: %s%s" % (agent_name, "success" if success else "failed, ", "" if success else self.skillset.get_single_agent_result_info(success, info)))
                    
    #                     self.skillset.reset_all_waypoint_ind()
    #                     eval("self.skillset.reset_%s_waypoint_ind" % agent_name)()
    #                     eval("self.skillset.reset_%s_cache" % agent_name)()
                    
                    

    #         # Run simulation step
    #         og.sim.step()



def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', type=str, default='Merom_1_int_Task1')


    return parser.parse_args()


def main(args):

    cfg = TASKSET[args.task_name]

    env = og.Environment(configs=cfg, physics_timestep=1/400., action_timestep=16/400.)

    og.sim.enable_viewer_camera_teleoperation()
    # Update the simulator's viewer camera's pose so it points towards the robot
    if 'grocery_store_cafe' in args.task_name:
        # original grocery
        og.sim.viewer_camera.set_position_orientation(
            position=np.array([4.9, 7.72065345, 0.91041777]),
            orientation=np.array([0.6366,-0.2268,-0.274,0.694]),
        )
        # updated grocery
        # og.sim.viewer_camera.set_position_orientation(
        #     position=np.array([7.99, 10.57, 0.71]),
        #     orientation=np.array([0.50783615, 0.44324224, 0.4857279 , 0.55651341]),
        # )

    if 'Merom_1_int' in args.task_name:
        # original Merom_1_int
        # og.sim.viewer_camera.set_position_orientation(
        #     position=np.array([4.86, 7.85, 1.05]),
        #     orientation=np.array([0.4576,0.46,0.54,0.536]),  # (x,y,z,w)

        # )

        # original Merom_1_int (franka back)
        og.sim.viewer_camera.set_position_orientation(
            position=np.array([-2.10719494,  6.81360779,  1.55880719]),
            orientation=np.array([0.47629345, -0.38766603, -0.49748654,  0.61267181]),  # (x,y,z,w)

        )

    if 'restaurant_brunch' in args.task_name:
        # original restaurant
        og.sim.viewer_camera.set_position_orientation(
            position=np.array([0.89, -3.33, 1.846]),
            orientation=np.array([0.56, 0.275, 0.337, 0.7]),  # (x,y,z,w)

        )

    if 'Pomaria_1_int' in args.task_name:
        # original Pomaria_1_int
        og.sim.viewer_camera.set_position_orientation(
            position=np.array([0.97, -3, 1.7]),
            orientation=np.array([0.57,0.272,0.33,0.7]),  # (x,y,z,w)

        )
    
    if 'house_double_floor_lower' in args.task_name:
        # original house_double_floor (garden)
        # og.sim.viewer_camera.set_position_orientation(
        #     position=np.array([20.53, 14.6, 3.1]),
        #     orientation=np.array([0.31,0.537,0.68,0.4]),  # (x,y,z,w)

        # )

        # original house_double_floor (far from fridge)
        og.sim.viewer_camera.set_position_orientation(
            position=np.array([8.25, 3.35, 1.05]),
            orientation=np.array([0.19,0.6,0.737,0.23]),  # (x,y,z,w)
    
        )

        # original house_double_floor (in front of the fridge)
        # og.sim.viewer_camera.set_position_orientation(
        #     position=np.array([5, 2.235, 1.46]),
        #     orientation=np.array([0.058,0.535,0.84,0.075]),  # (x,y,z,w)
          
        # )


        # original house_double_floor (outside)
        og.sim.viewer_camera.set_position_orientation(
            position=np.array([13.43361232, 10.73460434,  1.65867802]),
            orientation=np.array([0.57089634, -0.34936513, -0.39396453, 0.62993121]),  # (x,y,z,w)
        #    
        )
    ss = HASimulationSystem(task_name=args.task_name, env=env)





if __name__ == '__main__':
    args = parse_args()
    main(args)


