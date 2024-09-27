

# ## path planning

# import omni
# from omni.isaac.occupancy_map import _occupancy_map


## msg
import rospy
from std_msgs.msg import Float64MultiArray
from hademo.msg import Action, Func_and_Args, Args
from hademo.msg import Result


import math
import yaml
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R
import os
import sys
print(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))))
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))))
# print(sys.path)
from tasks import *
import worldmodel
from constants import *







def _get_quat_from_euler(euler):
    quat_back = R.from_euler("XYZ", euler).as_quat()

    quat_first = quat_back[[3, 0, 1, 2]]

    return quat_first

def _get_quat_with_z_up(start_pos, end_pos):
    diff_pos = np.array(end_pos) - np.array(start_pos)
    norm_dir_2D = diff_pos / np.linalg.norm(diff_pos)

    x_axis = np.array([norm_dir_2D[0], norm_dir_2D[1], 0], dtype=np.float32)
    z_axis = np.array([0, 0, 1], dtype=np.float32)
    y_axis = np.cross(z_axis, x_axis)

    rot_mat = np.zeros((3, 3))
    rot_mat[:, 0] = x_axis
    rot_mat[:, 1] = y_axis
    rot_mat[:, 2] = z_axis

    euler = R.from_matrix(rot_mat).as_euler('XYZ')

    quat_first = _get_quat_from_euler(euler)

    return quat_first


Merom_1_int_Task1_TextPlanList = [
    "Task beginning",

    ["aliengo_0", "movetowards", "apple_agveuv_0"],
    ["aliengo_0", "grab", "apple_agveuv_0"],
    ["aliengo_0", "movetowards", "quadrotor_0"],
    ["aliengo_0", "putintobasket", ["apple_agveuv_0", "quadrotor_0"]],

    ["quadrotor_0", "takeoff_from", ""],
    ["quadrotor_0", "movetowards", "breakfast_table_skczfi_0"],
    ["quadrotor_0", "land_on", "breakfast_table_skczfi_0"],
    ["franka_0", "grabfrombasket", ["apple_agveuv_0", "quadrotor_0"]],
    ["franka_0", "puton", "breakfast_table_wicker_basket_dgkhyn_0-p1"],
    
]    
house_double_floor_lower_Task1_TextPlanList = [
    "Task beginning",

    ["aliengo_0", "movetowards", "fridge_xyejdx_0_open_door-p1"],
    ["aliengo_0", "movetowards", "fridge_xyejdx_0_open_door-p2"],
    ["aliengo_0", "open", "fridge_xyejdx_0-j_link_1"],
    ["aliengo_0", "movetowards", "kebab_cewhbv_0"],
    ["aliengo_0", "grab", "kebab_cewhbv_0"],
    ["aliengo_0", "movetowards", "quadrotor_0"],
    ["aliengo_0", "putintobasket", ["kebab_cewhbv_0", "quadrotor_0"]],
    ["quadrotor_0", "takeoff_from", ""],
    ["quadrotor_0", "movetowards", "flying_waypoint_out_livingroom_p1"],
    ["quadrotor_0", "movetowards", "flying_waypoint_over_pedestal_table_dmghrm_0"],
    ["quadrotor_0", "land_on", "pedestal_table_dmghrm_0"],
    ["franka_0", "grabfrombasket", ["kebab_cewhbv_0", "quadrotor_0"]],
    ["franka_0", "puton", "grill_sxfjac_0"],

]  


class Translator:
    def __init__(self, task_name):
        self.task_name = task_name
        self.task = eval("defaults_%s" % self.task_name)

    def translate(self, text_action):
        agent_name = text_action[0]
        operation = text_action[1]
        other = text_action[2]
        if '-' in other:
            parts = other.split("-")
            other1 = parts[0]
            other2 = parts[1]
        self.worldstate = worldmodel.get_state(self.task_name)
        

        # e.g. aliengo [movetowards] <apple> (17)
        if operation == "movetowards":
            sim_action_list = self.movetowards2Simaction(agent_name, operation, other)        
        if operation == "grab":
            sim_action_list = self.grab2Simaction(agent_name, operation, other)         
        if operation == "grabfrombasket":
            sim_action_list = self.grabfrombasket2Simaction(agent_name, operation, other)       
        if operation == "puton":
            sim_action_list = self.puton2Simaction(agent_name, operation, other)  
        if operation == "putintobasket":
            sim_action_list = self.putintobasket2Simaction(agent_name, operation, other)    
        
        if operation == "open":
            sim_action_list = self.opendoor2Simaction(agent_name, operation, other)   
        if operation == "takeoff_from":
            sim_action_list = self.takeoff2Simaction(agent_name, operation, other)      
        if operation == "land_on":
            sim_action_list = self.land2Simaction(agent_name, operation, other)      
        
        return sim_action_list

    def getactionTemplate(self):
        t = dict()
        for name in self.task.agent_name_list:
            t[name] = []
        return t
        
    def movetowards2Simaction(self, agent_name, operation, other):
        sim_action_list = []

        if "aliengo" in agent_name:
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_dog_stand_begin_to_move", {}]
            sim_action_list.append(simactionTemplate)

        
        currentpose = self.worldstate[agent_name]
        
        if 'putpoint-' in other:

            putname = other[len('putpoint-'):]
            putpose = self.task.putpose[other[len('putpoint-'):]]
            if putname in self.task.nearbypose.keys():
                _goalposes = self.task.nearbypose[putname]
                if len(_goalposes) == 1:
                    goalpose = _goalposes[0]
                else:
                    closeInd = np.argmin(np.linalg.norm(np.array(_goalposes)[:, :2] - np.array(putpose)[:2], axis=-1))
                    goalpose = _goalposes[closeInd]
            else:
                _goalposes = np.concatenate(list(self.task.nearbypose.values()), axis=0)
                if len(_goalposes) == 1:
                    goalpose = _goalposes[0]
                else:
                    closeInd = np.argmin(np.linalg.norm(np.array(_goalposes)[:, :2] - np.array(putpose)[:2], axis=-1))
                    goalpose = list(_goalposes[closeInd])
            
        elif other in self.task.room_name_list:

            _goalposes = self.task.roompose[other]
            if len(_goalposes) == 1:
                goalpose = _goalposes[0]
            else:
                closeInd = np.argmin(np.linalg.norm(np.array(_goalposes)[:, :2] - np.array(currentpose)[:2], axis=-1))
                goalpose = _goalposes[closeInd]
        elif other in self.task.nearbypose.keys():
            _goalposes = self.task.nearbypose[other]
            if len(_goalposes) == 1:
                goalpose = _goalposes[0]
            else:
                closeInd = np.argmin(np.linalg.norm(np.array(_goalposes)[:, :2] - np.array(currentpose)[:2], axis=-1))
                goalpose = _goalposes[closeInd]
        elif "quadrotor" in other:
            landpointName = self.get_landpoint_name(other)
            _goalposes = self.task.nearbypose[landpointName]
            if len(_goalposes) == 1:
                goalpose = _goalposes[0]
            else:
                closeInd = np.argmin(np.linalg.norm(np.array(_goalposes)[:, :2] - np.array(currentpose)[:2], axis=-1))
                goalpose = list(_goalposes[closeInd])
        else:
            # movetowards to object
            objectpose =  self.worldstate[other]
            _goalposes = np.concatenate(list(self.task.nearbypose.values()), axis=0)
            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", other)
            # print(_goalposes)
            if len(_goalposes) == 1:
                goalpose = _goalposes[0]
            else:
                closeInd = np.argmin(np.linalg.norm(np.array(_goalposes)[:, :2] - np.array(objectpose)[:2], axis=-1))
                goalpose = list(_goalposes[closeInd])
            # print("(", closeInd, ")")
            # print(goalpose)
            # input('sss')
        waypoints = self.pathplanning(currentpose, goalpose)

        simactionTemplate = self.getactionTemplate()
        if "aliengo" in agent_name:
            simactionTemplate[agent_name] = ["aliengo_dog_move_with_waypoints", waypoints]
        elif "franka" in agent_name:
            simactionTemplate[agent_name] = ["franka_move_with_waypoints", waypoints]
        else:
            simactionTemplate[agent_name] = ["quadrotor_move_with_waypoints", waypoints]
        
        sim_action_list.append(simactionTemplate)

        print("))))))))))))))))))))))))))))))))))))))")
        print(sim_action_list)
        return sim_action_list

    
    def grab2Simaction(self, agent_name, operation, other):


        sim_action_list = []

        attached_prim_path = '/World/' + other  #  + '/base_link'
        objectpose =  self.worldstate[other]
        agentpose = self.worldstate[agent_name]
        print('objectpose', objectpose)
        print('agentpose', agentpose)
             
        grabpos = [
            objectpose[0],
            objectpose[1],
            objectpose[2],        
        ]

        print('grabpos', grabpos)
        
        if "aliengo" in agent_name:
            # print("###########################################")
            # print(self.worldstate[agent_name])
            if self.task_name == "Merom_1_int_Task1":
                #grab apple
                grabori = self.get_oripose2world(gripper2base=aliengo_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][10:])
            elif self.task_name == "house_double_floor_lower_Task1":
                #grab kebab
                grabori = self.get_oripose2world(gripper2base=aliengo_defaults["frontreargrasp"], base2world=self.worldstate[agent_name][10:])
            else:   
                grabori = self.get_oripose2world(gripper2base=aliengo_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][10:])

            print('grabori', grabori)
            # input('stop')
            # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            # print(grabori)
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=aliengo_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name][7:])
            
            # print(init_pos)
            # print(init_quat)
            # print("*********************************************!!!!!!!!!!!!!!!!!!!!!")
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_dog_stand_begin_to_manipulate", {}]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_open_gripper", {}]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_pick", {
                'attached_prim_path': attached_prim_path,
                'waypoint_pos': [
                                    grabpos,
                                ],
                'waypoint_ori': [
                                    grabori,
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)


            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                ],
                'waypoint_ori': [
                                    init_quat
                                ],
                'waypoint_ind': 1
            }]
            # TODO
            sim_action_list.append(simactionTemplate)
        
        if "franka" in agent_name:
            grabori = self.get_oripose2world(gripper2base=franka_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][3:])
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=franka_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name])
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_open_gripper", {}]
            sim_action_list.append(simactionTemplate)
            
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_pick", {
                'attached_prim_path': attached_prim_path,
                'waypoint_pos': [objectpose[:3]],
                'waypoint_ori': [grabori],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)


            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                ],
                'waypoint_ori': [
                                    init_quat
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)


        return sim_action_list
    
    def grabfrombasket2Simaction(self, agent_name, operation, other):


        attached_prim_path = '/World/' + other[0]  #  + '/base_link'
        sim_action_list = []


        objectpose =  self.worldstate[other[0]]
        agentpose = self.worldstate[agent_name]
        signflag = (np.array(agentpose) - np.array(objectpose)) / np.linalg.norm(np.array(agentpose) - np.array(objectpose), 2)
        
        grabpos = [
            objectpose[0],
            objectpose[1],
            objectpose[2],
        ]

        

        if "aliengo" in agent_name:

            grabori = self.get_oripose2world(gripper2base=aliengo_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][3:])

            # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            # print(grabori)
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=aliengo_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name][7:])
            
            # print(init_pos)
            # print(init_quat)
            # print("*********************************************!!!!!!!!!!!!!!!!!!!!!")
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_dog_stand_begin_to_manipulate", {}]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_open_gripper", {}]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate["quadrotor"] = ["quadrotor_detach_prim_path", {}]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_pick", {
                'attached_prim_path': attached_prim_path,
                'waypoint_pos': [
                                    grabpos,
                                ],
                'waypoint_ori': [
                                    grabori,
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)


            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                ],
                'waypoint_ori': [
                                    init_quat
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)
        
        if "franka" in agent_name:
            grabori = self.get_oripose2world(gripper2base=franka_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][3:])
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=franka_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name])
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_open_gripper", {}]
            sim_action_list.append(simactionTemplate)
            
            # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", objectpose[:3])
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_pick", {
                'agent_name': agent_name,
                'attached_prim_path': attached_prim_path,
                # 'waypoint_pos': [objectpose[:3]],
                "waypoint_pos": [grabpos],
                'waypoint_ori': [grabori],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[other[1]] = ["quadrotor_detach_prim_path", {}]
            sim_action_list.append(simactionTemplate)


            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                ],
                'waypoint_ori': [
                                    init_quat
                                ],
                'waypoint_ind': 1
            }]
            print('init_pos', init_pos, 'init_quat', init_quat)
            sim_action_list.append(simactionTemplate)


        return sim_action_list
    
    

    def putintobasket2Simaction(self, agent_name, operation, other):


        attached_prim_path = '/World/' + other[0]  #  + '/base_link'
        sim_action_list = []

        quadrotorpose =  self.worldstate[other[1]]
        putpos = [
                    quadrotorpose[0], 
                    quadrotorpose[1], 
                    quadrotorpose[2]+0.3
                ]
        
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # print(putpos)

        if "aliengo" in agent_name:
            putori = self.get_oripose2world(gripper2base=aliengo_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][10:])
            # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            # print(putpos)
            # print(putori)
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=aliengo_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name][7:])
            
            # print(init_pos)
            # print(init_quat)
            # print("*********************************************!!!!!!!!!!!!!!!!!!!!!")
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_dog_stand_begin_to_manipulate", {}]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_place", {
                'waypoint_pos': [
                                    putpos,
                                ],
                'waypoint_ori': [
                                    putori,
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)


            simactionTemplate = self.getactionTemplate()
            simactionTemplate[other[1]] = ["quadrotor_set_attached_prim_path", {
                                                "agent_name": other[1],
                                                "attached_prim_path": attached_prim_path
                                            }]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                ],
                'waypoint_ori': [
                                    init_quat
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)
        
        if "franka" in agent_name:
            putori = self.get_oripose2world(gripper2base=franka_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][3:])
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=franka_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name])
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_place", {
                'waypoint_pos': [
                                    putpos
                                ],
                'waypoint_ori': [
                                    putori
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)
            simactionTemplate = self.getactionTemplate()
            simactionTemplate["quadrotor"] = ["quadrotor_set_attached_prim_path", {
                                                "attached_prim_path": attached_prim_path
                                            }]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                ],
                'waypoint_ori': [
                                    init_quat
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)


        print("(((((((((((((())))))))))))))")
        print(sim_action_list)
        return sim_action_list


    def puton2Simaction(self, agent_name, operation, other):
        sim_action_list = []


        putpos = [
            self.task.putpose[other][0],
            self.task.putpose[other][1],
            self.task.putpose[other][2]+0.1,
        ]

        if "aliengo" in agent_name:

            putori = self.get_oripose2world(gripper2base=aliengo_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][10:])
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=aliengo_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name][7:])
            
            # print(init_pos)
            # print(init_quat)
            # print("*********************************************!!!!!!!!!!!!!!!!!!!!!")
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_dog_stand_begin_to_manipulate", {}]
            sim_action_list.append(simactionTemplate)

            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_place", {
                'waypoint_pos': [
                                    putpos,
                                ],
                'waypoint_ori': [
                                    putori,
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)



            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["aliengo_arm_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                ],
                'waypoint_ori': [
                                    init_quat
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)

        if "franka" in agent_name:
            putori = self.get_oripose2world(gripper2base=franka_defaults["topdowngrasp"], base2world=self.worldstate[agent_name][3:])
            init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=franka_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name])
            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_place", {
                'waypoint_pos': [
                                    putpos
                                ],
                'waypoint_ori': [
                                    putori
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)


            simactionTemplate = self.getactionTemplate()
            simactionTemplate[agent_name] = ["franka_move_with_waypoints", {
                'waypoint_pos': [
                                    init_pos
                                
                                ],
                'waypoint_ori': [
                                    init_quat
                                    
                                ],
                'waypoint_ind': 1
            }]
            sim_action_list.append(simactionTemplate)

        return sim_action_list

    def opendoor2Simaction(self, agent_name, operation, other):
        sim_action_list = []


        openpos_list, openori_list = self.opendoorTrajectory(self.task.opendoorpose[other])
        
        grabori = self.get_oripose2world(gripper2base=aliengo_defaults["frontreargrasp"], base2world=self.worldstate[agent_name][10:])
        init_pos, init_quat = self.get_init_EE_pose2world(initEEpose2base_list=aliengo_defaults["initEEpose2base"], base2world_list=self.worldstate[agent_name][7:])
        
        # print(init_pos)
        # print(init_quat)
        # print("*********************************************!!!!!!!!!!!!!!!!!!!!!")
        simactionTemplate = self.getactionTemplate()
        simactionTemplate[agent_name] = ["aliengo_dog_stand_begin_to_manipulate", {}]
        sim_action_list.append(simactionTemplate)

        simactionTemplate = self.getactionTemplate()
        simactionTemplate[agent_name] = ["aliengo_arm_open_gripper", {}]
        sim_action_list.append(simactionTemplate)
       
        simactionTemplate = self.getactionTemplate()
        simactionTemplate[agent_name] = ["aliengo_arm_move_with_waypoints", {
            'waypoint_pos': openpos_list,
            'waypoint_ori': grabori,
            'waypoint_ind': len(openpos_list)
        }]
        sim_action_list.append(simactionTemplate)

        simactionTemplate = self.getactionTemplate()
        simactionTemplate[agent_name] = ["aliengo_arm_move_with_waypoints", {
            'waypoint_pos': [
                                init_pos
                            ],
            'waypoint_ori': [
                                init_quat
                            ],
            'waypoint_ind': 1
        }]
        sim_action_list.append(simactionTemplate)

        ############################################
        
        simactionTemplate = self.getactionTemplate()
        new_other = other.replace('-', '/')
        simactionTemplate[agent_name] = ["aliengo_arm_door_open", {
            'articulation_path': '/World/' + new_other,
            # 'link_name': other2,
        }]
        sim_action_list.append(simactionTemplate)
        ##########################################################
        return sim_action_list
    
    
    def takeoff2Simaction(self, agent_name, operation, other):
        sim_action_list = []

        simactionTemplate = self.getactionTemplate()
        simactionTemplate[agent_name] = ["quadrotor_takeoff", {}]
        sim_action_list.append(simactionTemplate)

        return sim_action_list

    def land2Simaction(self, agent_name, operation, other):
        self.landmark = other
        sim_action_list = []

        # landpose = self.task.landpose[other]
        landpos = self.task.landpose[other][:3]
        landori = self.quatrear2quatfirst(self.task.landpose[other][3:])
        print("landpos:", landpos)
        print("landori:", landori)
        
        simactionTemplate = self.getactionTemplate()
        

        simactionTemplate = self.getactionTemplate()
        simactionTemplate[agent_name] = ["quadrotor_land", {
                'waypoint_pos': [
                                    landpos,
                                ],
                'waypoint_ori': [
                                    landori,
                                ],
                'waypoint_ind': 1
            }]
        sim_action_list.append(simactionTemplate)

        return sim_action_list


    def opendoorTrajectory(self, waypoints, deltastep=0.05):
        
        openpos_list = []
        openori_list = []
        cnt = len(waypoints)
        if cnt == 1:
            openpos_list.append(waypoints[0][:3])
            openori_list.append(waypoints[0][3:])
            return openpos_list, openori_list
        
        for i in range(cnt-1):
            posA = np.array(waypoints[i][:3])
            posB = np.array(waypoints[i+1][:3])
            
            deltapos = posB - posA
            deltasteps = np.ceil(np.abs(posB-posA) / deltastep).astype(int)
            max_step = np.max(deltasteps)
            for k in range(max_step):
                _pos = []
                for dim in range(3):
                    x = posA[dim] + np.sign(deltapos[dim]) * deltastep * k
                    if np.sign(deltapos[dim]) > 0 and x > posB[dim]:
                        x = posB[dim]
                    if np.sign(deltapos[dim]) < 0 and x < posB[dim]:
                        x = posB[dim]
                    _pos.append(x)

                openpos_list.append(_pos)
                openori_list.append(self.quatrear2quatfirst(waypoints[i+1][3:]))
        return openpos_list, openori_list

    def pathplanning(self, currentpose, goalpose):
        return  {
                    "waypoint_pos": [
                        goalpose[:3]
                    ],
                    "waypoint_ori": [
                        [goalpose[6], goalpose[3], goalpose[4], goalpose[5]]
                    ],
                    "waypoint_ind": 1
                }
    
    def quatrear2quatfirst(self, quatrear):
        return [quatrear[3], quatrear[0], quatrear[1], quatrear[2]]

    def quatfirst2quatrear(self, quatfirst):
        return [quatfirst[1], quatfirst[2], quatfirst[3], quatfirst[0]]

    def get_oripose2world(self, gripper2base, base2world):
        # print("gripper2base:", gripper2base)
        # print("base2world:", base2world)
        gripper2world = R.from_quat(base2world) * R.from_quat(gripper2base)
        quat_rear = gripper2world.as_quat()
        return self.quatrear2quatfirst(quat_rear)

    def get_init_EE_pose2world(self, initEEpose2base_list, base2world_list):
        
        # print("EE2basePos: ", initEEpose2base_list)
        EE2base = np.eye(4)
        EE2base[:3, 3] = np.array(initEEpose2base_list[:3])
        EE2base[:3, :3] = R.from_quat(np.array(initEEpose2base_list[3:])).as_matrix()



        # print("base2worldPos: ", base2world_list)
        base2world = np.eye(4)
        base2world[:3, 3] = np.array(base2world_list[:3])
        base2world[:3, :3] = R.from_quat(np.array(base2world_list[3:])).as_matrix()
        EE2world = np.matmul(base2world, EE2base)
        init_pos = EE2world[:3, 3]
        init_quat = R.from_matrix(EE2world[:3, :3]).as_quat()[[3, 0, 1, 2]]

        return init_pos, init_quat

    def get_landpoint_name(self, agent_name):
        alllandname = list(self.task.landpose.keys())
        alllandpos = np.array(list(self.task.landpose.values()))[:, :2]
        quadrotorpos = self.worldstate[agent_name][:2]

        closeInd = np.argmin(np.linalg.norm(alllandpos - quadrotorpos, axis=-1))
    
        return "landpoint-" + alllandname[closeInd]

class ActionPublishandResultSubscribe:
    def __init__(self, task_name):
        self.task_name = task_name

        self.task = eval("defaults_%s" % self.task_name)
        self.TextPlanList = eval(self.task_name+"_TextPlanList")
        # print("## Task Name: ", self.task_name)
        # for i in range(len(self.TextPlanList)):
        #     print("Step %d: [agent] %s [op] %s %s" % (i+1, self.TextPlanList[i][0], self.TextPlanList[i][1], self.TextPlanList[i][2]))
        # print("#############################")

        self.translator = Translator(task_name)
        self.sim_action_list = []
        self.text_action_cnt = 0

        self._sub = rospy.Subscriber("resultTopic", Result, self.callback)
        # Define the publisher's name, message type, and the number of queues.
        self._pub = rospy.Publisher('actionTopic', Action, queue_size=10, latch=True)

        self.warmup = True
        self.pub_flag = False
    


    # action_list, pub action
    def callback(self, result_msg):
        assert isinstance(result_msg, Result)
        if self.warmup:
            self.warmup = False
        else:
            self.sim_action_list.pop(0)
            
            print("*******************************")
            print("Receive result:")
            print(result_msg)
            print("*******************************")
            
            # print(worldmodel.get_state(self.task_name))
            # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        # check sub_data
        # if not self.check_success_from_result_msg(result_msg):
        #     # LLM needs to re-plan
        #     return
        
        if len(self.sim_action_list)==0:
            self.TextPlanList.pop(0)
            if len(self.TextPlanList):
                print("************************************************************************************")
                next_text_action = self.TextPlanList[0]
                self.text_action_cnt += 1
                print('### Text-based Action %d: [agent] %s [op] %s %s' % (self.text_action_cnt, next_text_action[0], next_text_action[1], next_text_action[2]))
                self.sim_action_list = self.translator.translate(next_text_action)
            print(self.sim_action_list)
            # input("Press Enter to continue...")
        if len(self.sim_action_list):
            next_sim_action = self.sim_action_list[0]
            print("next_sim_action: ", next_sim_action)
            action_msg = self.encode_to_action_msg(next_sim_action)
            # print(action_msg)
            # input("Press Enter to continue...")
            # rospy.loginfo("Publish the next action") # Output log information to the screen and write it to the rosout node.
            # print(action_msg)
            self._pub.publish(action_msg) # Publish messages to the topic.
            self.sub_flag = False
            self.pub_flag = True
        


    


    def encode_to_action_msg(self, next_step_action):

        def _parse(next_step_action):
            func_args_msg = Func_and_Args()
            if len(next_step_action):
                func_args_msg.has_func = True
                func_args_msg.func_name = next_step_action[0]
                
                args_msg = Args()
                # no args
                if next_step_action[1] == dict():
                    args_msg.has_args = False
                else:
                    args_msg.has_args = True
                    
                    if 'attached_prim' in next_step_action[0]:
                        args_msg.attached_prim_path = next_step_action[1]["attached_prim_path"]
                    elif 'door_open' in next_step_action[0]:
                        args_msg.attached_prim_path = next_step_action[1]["articulation_path"]

                    else:
                        waypoint_pos_msg = Float64MultiArray()
                        waypoint_ori_msg = Float64MultiArray()
                        # print(list(np.array(next_step_action[1]["waypoint_pos"], dtype=np.float64).flatten()))
                        waypoint_pos_msg.data = list(np.array(next_step_action[1]["waypoint_pos"], dtype=np.float64).flatten())
                        waypoint_ori_msg.data = list(np.array(next_step_action[1]["waypoint_ori"], dtype=np.float64).flatten())

                        args_msg.waypoint_pos = waypoint_pos_msg  # list(np.array(next_step_action[1]["waypoint_pos"]).flatten())
                        args_msg.waypoint_ori = waypoint_ori_msg  # list(np.array(next_step_action[1]["waypoint_ori"]).flatten())
                        args_msg.waypoint_ind = 0
                        if 'pick' in next_step_action[0]:
                            args_msg.attached_prim_path = next_step_action[1]["attached_prim_path"]
                    # print(args_msg)
                    # print("####################################")

                func_args_msg.args = args_msg
            else:
                func_args_msg.has_func = False
            return func_args_msg

        action_msg = Action()
        # print(next_step_action)
        # potential_agent_name_list = ['franka_0', 'franka_1', 'franka_2', 'aliengo_0', 'aliengo_1', 'aliengo_2']

        for agent in self.task.agent_name_list:
            exec("action_msg.%s = _parse(next_step_action[agent])" % agent)
                
        # print(action_msg)

        # action_msg.franka = franka_func_args_msg
        # action_msg.aliengo = aliengo_func_args_msg
        # action_msg.quadrotor = quadrotor_func_args_msg

        return action_msg

    def check_success_from_result_msg(self, result_msg):

        return True


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', type=str, default='Merom_1_int_Task1')


    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node('LLM', anonymous=True)
    wake = ActionPublishandResultSubscribe(task_name=args.task_name)
    rospy.spin()


if __name__ == '__main__':
    main()


