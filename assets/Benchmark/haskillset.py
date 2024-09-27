

import numpy as np
import math
import time
from constants import *

import omnigibson as og
from scipy.spatial.transform import Rotation as R
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.quadruped.utils.rot_utils import get_xyz_euler_from_quaternion, get_quaternion_from_euler
from omni.isaac.dynamic_control import _dynamic_control


from tasks import *

class DistanceSet:
    def __init__(self):
        self.delta_pose = 0.01
        self.threshold_delta_position = 0.03
        self.threshold_delta_quat = 0.2

    def reach_position(self, my_pos, goal_pos):
        delta_pos, succ_pos = self._judge_pos_close(my_pos, goal_pos)
        return delta_pos, succ_pos
        
    def reach_pose(self, my_pos, my_ori, goal_pos, goal_ori):
        delta_pos, succ_pos = self._judge_pos_close(my_pos, goal_pos)
        delta_ori, succ_ori = self._judge_qua_close(my_ori, goal_ori)
        print('delta_pos', delta_pos, 'delta_ori', delta_ori)
        print('reach_pose', 'succ_pos', succ_pos, 'succ_ori', succ_ori)
        return delta_pos, delta_ori, (succ_pos and succ_ori)

    
    def _judge_pos_close(self, p0, p1):
        delta_pos = np.linalg.norm(p0-p1)
        # print('delta_pos', delta_pos)
        if delta_pos < self.threshold_delta_position:
            return delta_pos, True
        else:
            return delta_pos, False


    def _judge_qua_close(self, q0_scalar_first,q1_scalar_first):
        # input (w, x, y, z)
        q0 = np.array([q0_scalar_first[1], q0_scalar_first[2], q0_scalar_first[3], q0_scalar_first[0]])
        q1 = np.array([q1_scalar_first[1], q1_scalar_first[2], q1_scalar_first[3], q1_scalar_first[0]])

        #input   q0,q1  (x,y,z,w)
        q11 = np.array([-q1[0],-q1[1],-q1[2],q1[3]])   
        mul = self._qua_multi(q11,q0,0)    
        logq = self._qua_log(mul)
        dis_qua = np.linalg.norm(logq) *2.0
        # print('dis_qua', dis_qua)
        if dis_qua < self.threshold_delta_quat:
            return dis_qua, True
        else:
            return dis_qua, False
    
    def _quat_first_from_XYZ_euler(self, euler):
        quat_first = get_quaternion_from_euler(euler)
        return quat_first
    
    def _XYZ_euler_from_quat_first(self, quat_first):
        euler = get_xyz_euler_from_quaternion(quat_first)
        return euler
    
    def diff_XYZ_euler_from_quat(self, curr_quat, goal_quat):
        euler_0 = self._XYZ_euler_from_quat_first(curr_quat)
        euler_1 = self._XYZ_euler_from_quat_first(goal_quat)

        minimal_diff_euler = self._minimal_diff_XYZ_euler(euler_1, euler_0)

        return minimal_diff_euler
    
    def _minimal_diff_XYZ_euler(self, euler_0, euler_1):
        diff_euler = euler_0 - euler_1
        minimal_diff_euler = np.zeros(3)
        for i in range(3):
            if diff_euler[i] > math.pi:
                minimal_diff_euler = diff_euler[i] - 2*math.pi
            if diff_euler[i] < -math.pi:
                minimal_diff_euler = diff_euler[i] + 2*math.pi
        return minimal_diff_euler

    
    def _qua_multi(self, qa,qb,m):
        #Both qa and qb are quaternions, [x, y, z, w], with the real part at the end
        if(m==0):        
            #Left-multiply qb by qa
            qa_m=[[qa[3],-qa[2],qa[1],qa[0]],
            [qa[2],qa[3],-qa[0],qa[1]],
            [-qa[1],qa[0],qa[3],qa[2]],
            [-qa[0],-qa[1],-qa[2],qa[3]]]   
            qc=np.matmul(qa_m,qb)

        if(m==1):      
            #Right-multiply qb by qa
            qa_m=[[qa[3],qa[2],-qa[1],qa[0]],
            [-qa[2],qa[3],qa[0],qa[1]],
            [qa[1],-qa[0],qa[3],qa[2]],
            [-qa[0],-qa[1],-qa[2],qa[3]]]
            qc=np.matmul(qa_m,qb)
        return qc

    def _qua_log(self, q0):
        # Define the logarithm of a quaternion:
        # The logarithm of a quaternion is defined as the projection of q onto the tangent plane 
        # of the identity quaternion [0,0,0,1] (no rotation).
        # input: q (4,)  Quaternion in the form x, y, z, w
        # output: log(q) (3,)

        qv = np.array([q0[0],q0[1],q0[2]])

        logq  =np.zeros((3,))
        if (q0[0]**2+q0[1]**2+q0[2]**2) <1e-5:
            logq = np.zeros((3,))
        else:
            if q0[-1]<0 and q0[-1]>=-1:
                logq = (math.acos(q0[-1])-math.pi)*qv/np.linalg.norm(qv)
            if q0[-1]<=1 and q0[-1]>=0:
                logq = math.acos(q0[-1])*qv/np.linalg.norm(qv)
        return logq

class LogSet:
    def __init__(self) -> None:
        
        self._action_round = 0

        self.errorFlags = {
            0: "IK failed",
            1: "Not reach",
            2: "Gripper is not opened",
            3: "Gripper is not closed",
            4: "Illegal dimenstions",
            5: "Not ready to move",  # if aliengo need to move
            6: "Not ready to manipulate",  # if aliengo need to manipulate
            7: "Not takeoff"  # if quadrotor need to hover / move
        }

    @property
    def action_round(self):
        return self._action_round

    @action_round.setter
    def action_round(self, round):
        self._action_round = round
    def action_round_plus1(self):
        self._action_round += 1

    def get_single_agent_result_info(self, success, info):
        result_info = ""
        # if not success:
        #     result_info = self.errorFlags[info["errorFlag"]]
        #     if info["errorFlag"] in [0, 1]:
        #         result_info += " at waypoint %d" % info["waypoint_ind"]
        #     if info["errorFlag"] == 4:
        #         if info["waypoint_ind"] == 0:
        #             result_info += ", perhaps len(waypoints_pos) != len(waypoints_ori) or len(waypoints_pos[0]) != 3 or len(waypoints_ori[0]) != 4"
        #         else:
        #             result_info += ", len(waypoints_pos[%d]) != 3 or len(waypoints_ori[%d])!= 4" % (info["waypoint_ind"], info["waypoint_ind"])
        return result_info
    
    
    def log_all_result(self, result_list, result_info):
        print("============= Action Round %d =============" % self._action_round)
        for action_ind, (success, info) in enumerate(zip(result_list, result_info)):

            errorInfo = ""
            if not success:
                errorInfo = self.errorFlags[info["errorFlag"]]
                if info["errorFlag"] in [0, 1]:
                    errorInfo += " at waypoint %d" % info["waypoint_ind"]
                if info["errorFlag"] == 4:
                    if info["waypoint_ind"] == 0:
                        errorInfo += ", perhaps len(waypoints_pos) != len(waypoints_ori) or len(waypoints_pos[0]) != 3 or len(waypoints_ori[0]) != 4"
                    else:
                        errorInfo += ", len(waypoints_pos[%d]) != 3 or len(waypoints_ori[%d])!= 4" % (info["waypoint_ind"], info["waypoint_ind"])
            print("action %d: %s - %s" % (action_ind, 
                                            "successful" if success else "failed",
                                            errorInfo))




class SkillSet(DistanceSet):
    def __init__(self, task_name, agents):
        super().__init__()
        self.task_name = task_name

        self.task = eval("defaults_%s" % self.task_name)
        self.agents_dict = dict(zip(self.task.agent_name_list, agents))
        self.agents_memory_dict = dict(zip(self.task.agent_name_list, [{}] * len(self.task.agent_name_list)))
        self.agents_waypoint_dict = dict(zip(self.task.agent_name_list, [{}] * len(self.task.agent_name_list)))
        self.franka_name_list = []
        self.aliengo_name_list = []
        self.quadrotor_name_list = []

        for cnt, agent_name in enumerate(self.task.agent_name_list):
            if "franka" in agent_name:
                self.franka_name_list.append(agent_name)
                self.agents_memory_dict[agent_name] = {
                    'prev_gripper_position': None
                }
                self.agents_waypoint_dict[agent_name] = {
                    "cnt": 0
                }
            if "aliengo" in agent_name:
                self.aliengo_name_list.append(agent_name)
                agents[cnt].wake_deault_gait_control()
                
                self.agents_memory_dict[agent_name] = {
                    'stand_still_pos': None,
                    'stand_still_quat_first': None,
                    'prev_gripper_position': None,
                    'prev_ee_pos': None,
                    'prev_ee_quat_first': None,
                    'arm_base_pos': None,
                    'arm_base_quat_first': None
                }
                self.agents_waypoint_dict[agent_name] = {
                    "arm_cnt": 0,
                    "dog_cnt": 0
                }
            if "quadrotor" in agent_name:
                self.quadrotor_name_list.append(agent_name)
                agents[cnt].wake_deault_rotor_control()
                self.agents_memory_dict[agent_name] = {
                    'hover_state': None,
                }
                self.agents_waypoint_dict[agent_name] = {
                    "cnt": 0
                }


        self._default_aliengo_delta_pos = 0.04
        self._default_aliengo_delta_turn = 6. / 180. * math.pi
        self._default_aliengo_delta_quat, _ = self._judge_qua_close(q0_scalar_first=self._quat_first_from_XYZ_euler(np.array([0, 0, self._default_aliengo_delta_turn])),
                                                                      q1_scalar_first=np.array([1, 0, 0, 0]))

        self._default_quadrotor_delta_pos = 0.02
        self._default_quadrotor_delta_turn = 6. / 180. * math.pi  # euler-format
        

        self.reset()
        # not done & not success: is ok
        # done & success: action is executed successfully
        # done & not success: check the errorFlag
        # not done & success: my codes have bugs

    @property
    def has_franka(self):
        return len(self.franka_name_list)
    

    @property
    def has_aliengo(self):
        return len(self.aliengo_name_list)
    

    @property
    def has_quadrotor(self):
        return len(self.quadrotor_name_list)
    

    def reset(self):
        self.disactive_agents()
        self.reset_all_agents_default_states()
        
    def reset_all_agents_default_states(self):
        print('\n-----------------------------------------------')
        self.reset_franka_default_states()
        self.reset_aliengo_default_states()
        self.reset_quadrotor_default_states()
        print('-----------------------------------------------\n')



    def reset_franka_default_states(self):
        # franka
        
        default_gripper_opened = False
        ####################################################

        for agent_name in self.agents_dict.keys():
            if "franka" not in agent_name:
                continue
            if default_gripper_opened:
                franka_gripper_opened = False
                while not franka_gripper_opened:
                    _, franka_gripper_opened, _ = self.franka_open_gripper(agent_name)
                    
                    og.sim.step()
            else:
                franka_gripper_closed = False
                while not franka_gripper_closed:
                    _, franka_gripper_closed, _ = self.franka_close_gripper(agent_name)
                    
                    og.sim.step()

            print("***************** Franka *****************")
            print("%s Gripper: %s" % (agent_name, "opened" if self.agents_dict[agent_name].is_gripper_fully_opened() else "closed"))
    
            self.reset_franka_cache(agent_name)


    def reset_aliengo_default_states(self):
        # aliengo

        default_dog_active = False
        default_gripper_opened = False
        ####################################################

        for agent_name in self.agents_dict.keys():
            if "aliengo" not in agent_name:
                continue
            begin_to_mani = False
            while not begin_to_mani:
                _, begin_to_mani, _ = self.aliengo_dog_stand_begin_to_manipulate(agent_name)
                og.sim.step()
            
            if default_gripper_opened:
                aliengo_gripper_opened = False
                while not aliengo_gripper_opened:
                    _, aliengo_gripper_opened, _ = self.aliengo_arm_open_gripper(agent_name)
                    
                    og.sim.step()
            else:
                aliengo_gripper_closed = False
                while not aliengo_gripper_closed:
                    _, aliengo_gripper_closed, _ = self.aliengo_arm_close_gripper(agent_name)
                    
                    og.sim.step()

            if default_dog_active:
                begin_to_move = False
                while not begin_to_move:
                    _, begin_to_move, _ = self.aliengo_dog_stand_begin_to_move(agent_name)
                    og.sim.step()

                # for i in range(300):
                #     og.sim.step()
            else:
                begin_to_mani = False
                while not begin_to_mani:
                    _, begin_to_mani, _ = self.aliengo_dog_stand_begin_to_manipulate(agent_name)
                    og.sim.step()

                    
            print("***************** Aliengo *****************")
            print("%s Dog is active: %s" % (agent_name, "True" if self.agents_dict[agent_name].dog_active else "False"))
            print("%s Gripper: %s" % (agent_name, "opened" if self.agents_dict[agent_name].is_gripper_fully_opened() else "closed"))


            self.reset_aliengo_cache(agent_name)

    def reset_quadrotor_default_states(self):
        # aliengo

        default_quadrotor_active = False
        if not default_quadrotor_active:
            default_quadrotor_state = "land"
        else:
            default_quadrotor_state = "takeoff"
        ####################################################



        for agent_name in self.agents_dict.keys():
            if "quadrotor" not in agent_name:
                continue
            self.agents_dict[agent_name].active = default_quadrotor_active

            if default_quadrotor_state == "takeoff":
                reach_default_state = False
                while not reach_default_state:
                    _, reach_default_state, _ = self.quadrotor_takeoff(agent_name)

                og.sim.step()

            print("***************** Quadrotor *****************")
            print("%s is active: %s" % (agent_name, "True" if self.agents_dict[agent_name].active else "False"))
            print("%s  state: %s" % (agent_name, default_quadrotor_state))

    
            self.reset_quadrotor_cache(agent_name)
    


    def reset_franka_cache(self, agent_name):
        self.agents_memory_dict[agent_name] = {
            'prev_gripper_position': None
        }

    def reset_franka_waypoint_ind(self, agent_name):
        assert "franka" in agent_name
        self.agents_waypoint_dict[agent_name] = {
                    "cnt": 0,
                }


        
    def reset_aliengo_cache(self, agent_name):
        
        self.agents_memory_dict[agent_name] = {
            'stand_still_pos': None,
            'stand_still_quat_first': None,
            'prev_gripper_position': None,
            'prev_ee_pos': None,
            'prev_ee_quat_first': None,
            'arm_base_pos': None,
            'arm_base_quat_first': None
        }
        
    def reset_aliengo_waypoint_ind(self, agent_name):
        assert "aliengo" in agent_name
        self.agents_waypoint_dict[agent_name] = {
            "arm_cnt": 0,
            "dog_cnt": 0
        }

    
    def reset_quadrotor_cache(self, agent_name):
        self.agents_memory_dict[agent_name] = {
            'hover_state': None,
        }

    def reset_quadrotor_waypoint_ind(self, agent_name):
        assert "quadrotor" in agent_name
        self.agents_waypoint_dict[agent_name] = {
            "cnt": 0,
        }

    def disactive_agents(self):
        # if self.franka:
        #     self.franka.active = False
        
        for agent_name, agent in self.agents_dict.items():
            if "aliengo" in agent_name:
                agent.dog_active = False
            if "quadrotor" in agent_name:
                agent.active = False
    

    def attach(self):
        for agent_name in self.agents_dict.keys():
            self.agents_dict[agent_name].attach()

    ##############################################################
    ## Franka
    
    def franka_open_gripper(self, agent_name):
        done = False
        success = False
        info = {}

        self.agents_dict[agent_name].gripper.open()

        done = success = self.agents_dict[agent_name].is_gripper_fully_opened()

        return done, success, info
    
    def franka_close_gripper(self, agent_name):
        done = False
        success = False
        info = {}
        
        self.agents_dict[agent_name].gripper.close()

        prev_gripper_position = self.agents_memory_dict[agent_name]['prev_gripper_position']
        if prev_gripper_position is None:
            self.agents_memory_dict[agent_name]['prev_gripper_position'] = self.agents_dict[agent_name].gripper.get_joint_positions()
        else:
            curr_position = self.agents_dict[agent_name].gripper.get_joint_positions()
            dist = np.linalg.norm(prev_gripper_position-curr_position, ord=1)
            done = success = dist < 0.01
            self.agents_memory_dict[agent_name]['prev_gripper_position'] = curr_position

        return done, success, info

    def franka_pick(self, agent_name, attached_prim_path, waypoint_pos, waypoint_ori, waypoint_ind):
        

        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}

        if not self.agents_dict[agent_name].is_gripper_fully_opened():
            # "`Pick` can only be done when the gripper is fully opened"
            done, success, info = self.franka_open_gripper(agent_name)
            return done, success, info
        
        
        move_done, move_success, move_info = self.franka_move_with_waypoints(agent_name, waypoint_pos, waypoint_ori, waypoint_ind)
        info.update(move_info)
        if move_done and move_success:
            if attached_prim_path is not None:
                self.agents_dict[agent_name].set_attach_prim(attached_prim_path)
                done, success, _ = self.franka_close_gripper(agent_name)
            else:
                done = success = True

        if move_done and not move_success:
            done = True
            return done, success, info
        
        return done, success, info

    def franka_place(self, agent_name, waypoint_pos, waypoint_ori, waypoint_ind):


        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}

        
        move_done, move_success, move_info = self.franka_move_with_waypoints(agent_name, waypoint_pos, waypoint_ori, waypoint_ind)
        print('franka_move_with_waypoints---',"move_done:", move_done, "move_success:", move_success, "move_info:", move_info)
        info.update(move_info)
        if move_done and move_success:
            done, success, _ = self.franka_open_gripper(agent_name)
            print('franka_open_gripper---',"done:", done, "success:", success)
            self.agents_dict[agent_name].detach()
        if move_done and not move_success:
            done = True
            return done, success, info
        
        return done, success, info

    def franka_move_with_waypoints(self, agent_name, waypoint_pos, waypoint_ori, waypoint_ind):
        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}
        
        
        if len(waypoint_pos) != len(waypoint_ori):
            done = True
            info["errorFlag"] = 4  # "Illegal dimenstions"
            return done, success, info
        

        if waypoint_ind >= len(waypoint_pos):
            done = success = True
            return done, success, info

        sub_done, sub_success, sub_info = self._franka_move_to_pose(agent_name, waypoint_pos[waypoint_ind], waypoint_ori[waypoint_ind])
        print('_franka_move_to_pose---',"move_done:", sub_done, "move_success:", sub_success, "move_info:", sub_info)
        if sub_done and sub_success:
            waypoint_ind += 1
            if waypoint_ind == len(waypoint_pos):
                done = success = True
                info["waypoint_ind"] = len(waypoint_pos)  # Number of completed waypoints
                return done, success, info
            else:
                info["waypoint_ind"] = waypoint_ind
                return done, success, info
        if sub_done and not sub_success:
            # failed in waypoints[ind]

            done = True
            info.update(sub_info)
            
            return done, success, info


        return done, success, info
    
    def _franka_move_to_pose(self, agent_name, position, orientation):
        # quat scalar-first
        done = False
        success = False
        info = {}
        if len(position) != 3 or len(orientation) != 4:
            done = True
            info["errorFlag"] = 4  # "Illegal dimenstions"
            return done, success, info
        
        more_check_num = 3
        
        for i in range(more_check_num):
            actions, succ = self.agents_dict[agent_name]._ik_solver.compute_inverse_kinematics(
                target_position=position,
                target_orientation=orientation,
            )
            if succ:
                break
        print('compute_inverse_kinematics---',"actions:", actions, "succ:", succ)

        if succ:
            self.agents_dict[agent_name].apply_action(actions)
            self.agents_dict[agent_name].attach()
            
            ##################
            ee_pos, ee_ori = self.agents_dict[agent_name].get_ee_position_orientation()
            # biased_goal_ori = R.from_quat(np.array([ 0.,  1.,  0., -0.])) * R.from_quat(np.array([orientation[1], orientation[2], orientation[3], orientation[0]]))
            # biased_goal_ori = biased_goal_ori.as_quat()
            # biased_goal_ori = np.array([biased_goal_ori[3], biased_goal_ori[0], biased_goal_ori[1], biased_goal_ori[2]])
            delta_pos, delta_ori, success = self.reach_pose(my_pos=ee_pos, my_ori=ee_ori, goal_pos=position, goal_ori=orientation)
            print('reach_pose---',"delta_pos:", delta_pos, "delta_ori:", delta_ori, "success:", success)
            if success:
                done = True
            else:
                joint_velocities = np.abs(self.agents_dict[agent_name].get_joint_velocities()).mean()
            
                if joint_velocities < 0.01:
                    done = True
                    info["errorFlag"] = 1  # "Not reach"
                
            return done, success, info

        else:
            done = True
            info["errorFlag"] = 0  # "IK failed"
            return done, success, info
        
            # carb.log_warn("IK did not converge to a solution.  No action is being taken.")
        
    
    ##############################################################
    ## Aliengo

    def aliengo_dog_stand_still(self, agent_name):
        # print(self.agents_dict[agent_name])
        self.agents_dict[agent_name].dog_active = False
        
        if self.agents_memory_dict[agent_name]['stand_still_pos'] is None:
            _aliengo_stand_still_pos, _aliengo_stand_still_quat_first = self.agents_dict[agent_name].get_curr_position_orientation()
            self.agents_memory_dict[agent_name]['stand_still_pos'] = _aliengo_stand_still_pos
            self.agents_memory_dict[agent_name]['stand_still_quat_first'] = _aliengo_stand_still_quat_first
            

        # cast joint state to numpy float32
        self.agents_dict[agent_name].set_curr_position_orientation(
            self.agents_memory_dict[agent_name]['stand_still_pos'], 
            self.agents_memory_dict[agent_name]['stand_still_quat_first']
        )
        done = success = True
        return done, success, {}
    
    # stopping gait
    def aliengo_dog_stand_begin_to_manipulate(self, agent_name):
        
        self.agents_dict[agent_name].dog_stand_still()

        done = success = True

        return done, success, {}


    # starting gait
    def aliengo_dog_stand_begin_to_move(self, agent_name):
        
        self.agents_dict[agent_name].dog_stand_begin_to_move()
        self.reset_aliengo_cache(agent_name)
        
        done = success = True

        return done, success, {}
    
    def aliengo_dog_move_with_waypoints(self, agent_name, waypoint_pos, waypoint_ori, waypoint_ind):
        
        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}

        self.agents_dict[agent_name].dog_active = True
        
        
        if len(waypoint_pos) != len(waypoint_ori):
            done = True
            info["errorFlag"] = 4  # "Illegal dimenstions"
            return done, success, info
        

        if waypoint_ind >= len(waypoint_pos):
            done = success = True
            return done, success, info

        waypoint_pos[:, 2] = self.agents_dict[agent_name].default_begin_to_move_height
        

        if waypoint_ind != self.agents_waypoint_dict[agent_name]['dog_cnt']:
            self.agents_waypoint_dict[agent_name]['dog_cnt'] = waypoint_ind
            self.reset_aliengo_cache(agent_name)
            

            

        sub_done, sub_success, sub_info = self._aliengo_dog_move_to_pose(agent_name, waypoint_pos[waypoint_ind], waypoint_ori[waypoint_ind])
        
        if sub_done and sub_success:
            waypoint_ind += 1
            if waypoint_ind == len(waypoint_pos):
                done = success = True
                info["waypoint_ind"] = len(waypoint_pos)  # Number of completed waypoints
                return done, success, info
            else:
                info["waypoint_ind"] = waypoint_ind
                return done, success, info
        if sub_done and not sub_success:
            # failed in waypoints[ind]

            done = True
            info.update(sub_info)
            
            return done, success, info


        return done, success, info


    def _aliengo_dog_move_to_pose(self, agent_name, position, orientation):
        
        done = False
        success = False
        info = {}

        
        if len(position) != 3 or len(orientation) != 4:
            done = True
            info["errorFlag"] = 4  # "Illegal dimenstions"
            return done, success, info
        

        self.agents_dict[agent_name].dog_active = True
        
        

        # scalar-first
        curr_pos, curr_ori = self.agents_dict[agent_name].get_curr_position_orientation()
        curr_yaw = get_xyz_euler_from_quaternion(curr_ori)[2]
        goal_yaw = get_xyz_euler_from_quaternion(orientation)[2]

        diff_pos = position - curr_pos
        diff_yaw = goal_yaw - curr_yaw

        if np.linalg.norm(diff_pos) <= self._default_aliengo_delta_pos and abs(diff_yaw) <= self._default_aliengo_delta_turn:
            self.agents_dict[agent_name].set_curr_position_orientation(position, orientation)
            self.agents_dict[agent_name].attach()

            done = success = True
            return done, success, info
        

        temp_goal_pos = curr_pos.copy()
        temp_goal_yaw = curr_yaw
        if np.linalg.norm(diff_pos) <= self._default_aliengo_delta_pos:
            temp_goal_pos += diff_pos
        else:
            temp_goal_pos += diff_pos / np.linalg.norm(diff_pos) * self._default_aliengo_delta_pos
        if abs(diff_yaw) <= self._default_aliengo_delta_turn:
            temp_goal_yaw += diff_yaw
        else:
            # temp_goal_yaw += diff_yaw / abs(diff_yaw) * self._default_aliengo_delta_turn
            temp_goal_yaw += diff_yaw / diff_yaw * self._default_aliengo_delta_turn

        temp_goal_quat = get_quaternion_from_euler(np.array([0, 0, temp_goal_yaw]))
        # work when sim.step()
        self.agents_dict[agent_name].set_curr_position_orientation(temp_goal_pos, temp_goal_quat)
        self.agents_dict[agent_name].attach()

        return done, success, info


    def aliengo_arm_pick(self, agent_name, attached_prim_path, waypoint_pos, waypoint_ori, waypoint_ind):
        # self._aliengo_stand_still_state = None
        self.aliengo_dog_stand_still(agent_name)
        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}

        
        
        move_done, move_success, move_info = self.aliengo_arm_move_with_waypoints(agent_name, waypoint_pos, waypoint_ori, waypoint_ind)
        print('aliengo_arm_move_with_waypoints---',"move_done:", move_done, "move_success:", move_success, "move_info:", move_info)
        info.update(move_info)
        # import ipdb
        # ipdb.set_trace()
        if move_done and move_success:
            # pass
            # time.sleep(1)
            if attached_prim_path is not None:
                self.agents_dict[agent_name].set_attach_prim(attached_prim_path)
                done, success, _ = self.aliengo_arm_close_gripper(agent_name)
            else:
                done = success = True

        if move_done and not move_success:
            done = True
            return done, success, info
        
        return done, success, info

    def aliengo_arm_place(self, agent_name, waypoint_pos, waypoint_ori, waypoint_ind):
        # self._aliengo_stand_still_state = None
        self.aliengo_dog_stand_still(agent_name)
        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}


        move_done, move_success, move_info = self.aliengo_arm_move_with_waypoints(agent_name, waypoint_pos, waypoint_ori, waypoint_ind)
        
        info.update(move_info)
        if move_done and move_success:
            done, success, _ = self.aliengo_arm_open_gripper(agent_name)
            self.agents_dict[agent_name].detach()
        if move_done and not move_success:
            done = True
            return done, success, info
        
        return done, success, info


    def aliengo_arm_move_with_waypoints(self, agent_name, waypoint_pos, waypoint_ori, waypoint_ind):
        # self._aliengo_stand_still_state = None
        self.aliengo_dog_stand_still(agent_name)
        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}

        
        
        
        if len(waypoint_pos) != len(waypoint_ori):
            done = True
            info["errorFlag"] = 4  # "Illegal dimenstions"
            return done, success, info
        
        if waypoint_ind >= len(waypoint_pos):
            done = success = True
            return done, success, info

        if self.agents_memory_dict[agent_name]['arm_base_pos'] is None:
            self.agents_memory_dict[agent_name]['arm_base_pos'], self.agents_memory_dict[agent_name]['arm_base_quat_first'] = self.agents_dict[agent_name].get_arm_base_position_orientation()
            self.agents_dict[agent_name]._ik_solver.get_kinematics_solver().set_robot_base_pose(self.agents_memory_dict[agent_name]['arm_base_pos'], self.agents_memory_dict[agent_name]['arm_base_quat_first'])
            # print("I am here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        sub_done, sub_success, sub_info = self._aliengo_arm_move_to_pose(agent_name, waypoint_pos[waypoint_ind], waypoint_ori[waypoint_ind])
        print('_aliengo_arm_move_to_pose---',"move_done:", sub_done, "move_success:", sub_success, "move_info:", sub_info)
        if sub_done and sub_success:
            waypoint_ind += 1
            if waypoint_ind == len(waypoint_pos):
                done = success = True
                info["waypoint_ind"] = len(waypoint_pos) # Number of completed waypoints
                return done, success, info
            else:
                info["waypoint_ind"] = waypoint_ind
                return done, success, info
        if sub_done and not sub_success:
            # failed in waypoints[ind]

            done = True
            info.update(sub_info)
            
            return done, success, info


        return done, success, info
        

    def _aliengo_arm_move_to_pose(self, agent_name, position, orientation):
        # self._aliengo_stand_still_state = None
        self.aliengo_dog_stand_still(agent_name)
        # quat scalar-first
        done = False
        success = False
        info = {}

        if len(position) != 3 or len(orientation) != 4:
            done = True
            info["errorFlag"] = 4  # "Illegal dimenstions"
            return done, success, info


        
        more_check_num = 3
        
        for i in range(more_check_num):
            actions, succ = self.agents_dict[agent_name]._ik_solver.compute_inverse_kinematics(
                target_position=position,
                target_orientation=orientation,
            )
            # print("Ik solved ", succ)
            if succ:
                break
        print('compute_inverse_kinematics---',"actions:", actions, "succ:", succ)
        # self.aliengo.apply_action(actions)
        if succ:
            self.agents_dict[agent_name].apply_action(actions)

            self.agents_dict[agent_name].attach()
            
            ee_pos, ee_quat_first = self.agents_dict[agent_name].get_ee_position_orientation()
            delta_pos, delta_ori, success = self.reach_pose(my_pos=ee_pos, my_ori=ee_quat_first, goal_pos=position, goal_ori=orientation)
            print('reach_pose---',"delta_pos:", delta_pos, "delta_ori:", delta_ori, "success:", success)
            # print(self.agents_memory_dict[agent_name]['prev_ee_pos'])
            # print("-------------------------------------------------------")
            if self.agents_memory_dict[agent_name]['prev_ee_pos'] is not None:
                _, _, success = self.reach_pose(my_pos=self.agents_memory_dict[agent_name]['prev_ee_pos'], 
                                                my_ori=self.agents_memory_dict[agent_name]['prev_ee_quat_first'], 
                                                goal_pos=ee_pos, 
                                                goal_ori=ee_quat_first)  
                
                self.agents_memory_dict[agent_name]['prev_ee_pos'] = ee_pos
                self.agents_memory_dict[agent_name]['prev_ee_quat_first'] = ee_quat_first
        
                if success:
                    done = True
                    info["errorFlag"] = 1  # "Not reach"
            else:
                self.agents_memory_dict[agent_name]['prev_ee_pos'] = ee_pos
                self.agents_memory_dict[agent_name]['prev_ee_quat_first'] = ee_quat_first
                
                
                

        else:
            done = True
            info["errorFlag"] = 0  # "IK failed"

        return done, success, info
        
        
    def aliengo_arm_open_gripper(self, agent_name):
        # self._aliengo_stand_still_state = None
        self.aliengo_dog_stand_still(agent_name)
        done = False
        success = False
        info = {}

        self.agents_dict[agent_name].gripper.open()

        done = success = self.agents_dict[agent_name].is_gripper_fully_opened()
        

        return done, success, info
    
    def aliengo_arm_close_gripper(self, agent_name):
        # self._aliengo_stand_still_state = None
        self.aliengo_dog_stand_still(agent_name)
        done = False
        success = False
        info = {}

        self.agents_dict[agent_name].gripper.close()

        curr_position = self.agents_dict[agent_name].gripper.get_joint_positions()
        if self.agents_memory_dict[agent_name]['prev_gripper_position'] is not None:
            dist = np.linalg.norm(self.agents_memory_dict[agent_name]['prev_gripper_position']-curr_position, ord=1)
            done = success = dist < 0.01
        self.agents_memory_dict[agent_name]['prev_gripper_position'] = curr_position


        return done, success, info
    
    # aliengo_arm_open_door
    def aliengo_arm_door_open(self, agent_name, attached_prim_path):
        self.aliengo_dog_stand_still(agent_name)
        done = True
        success = True
        info = {}
        parts = attached_prim_path.split('/')
        articulation_path = '/' + parts[1] + '/' + parts[2]
        link_name = parts[-1]
        print('articulation_path:', articulation_path, "link_name:", link_name)
        # input("Press Enter to continue...")
        dc = _dynamic_control.acquire_dynamic_control_interface()

        # # Get the joint object using its path
        
        articulation = dc.get_articulation(articulation_path)
        # # Get information about the structure of the articulation
        num_joints = dc.get_articulation_joint_count(articulation)
        num_dofs = dc.get_articulation_dof_count(articulation)
        num_bodies = dc.get_articulation_body_count(articulation)
        # print(f"Articulation {articulation_path} has {num_joints} joints, {num_dofs} degrees of freedom, and {num_bodies} bodies.")
        # # Get a specific degree of freedom on an articulation
        for i in range(num_dofs):
            dof_handle = dc.get_articulation_dof(articulation, i)
            print (dof_handle)
            dof_name = dc.get_dof_name(dof_handle)
            print(f"Degree of freedom {i} is named {dof_name}.")
        dof_ptr = dc.find_articulation_dof(articulation, link_name)
        # # dof_state = dc.get_dof_state(articulation, 1)
        # # print(dof_state.pos)
        # print(dof_ptr)
        dc.wake_up_articulation(articulation)
        dc.set_dof_position(dof_ptr, 1.4)

        return done, success, info

    ##############################################################
    ## Quadrotor

    def quadrotor_set_attached_prim_path(self, agent_name, attached_prim_path):
        self.agents_dict[agent_name].set_attach_prim(attached_prim_path)
        done = success = True
        return done, success, {}
    

    def quadrotor_detach_prim_path(self, agent_name):
        self.agents_dict[agent_name].detach()

        done = success = True
        return done, success, {}

    def quadrotor_hover(self, agent_name):
        # if not self.agents_dict[agent_name].active:
        #     return True, True, {}
        
        if self.agents_memory_dict[agent_name]['hover_state'] is None:
            self.agents_memory_dict[agent_name]['hover_state'] = self.agents_dict[agent_name].get_state()
            self.agents_memory_dict[agent_name]['hover_state'].base_frame.quat = self.agents_dict[agent_name].always_z_up(self.agents_memory_dict[agent_name]['hover_state'].base_frame.quat)
        
        done = success = True

        # cast joint state to numpy float32
        self.agents_dict[agent_name].set_body_pose_from_state(self.agents_memory_dict[agent_name]['hover_state'])
        return done, success, {}
    

    def quadrotor_takeoff(self, agent_name):
        # start_time = time.time()
        # while time.time() - start_time < 5:
            
        #     og.sim.step()
        self.agents_memory_dict[agent_name]['hover_state'] = None
        self.agents_dict[agent_name].active = True
        pos, quat_first = self.agents_dict[agent_name].get_curr_position_orientation()
        print("takeoff:", pos)
        print("takeoff:", quat_first)
        pos[2] = quadrotor_defaults["hover_height"]
        done, success, info = self._quadrotor_move_to_pose(agent_name, position=pos, orientation=quat_first)

        return done, success, info
        
    def quadrotor_land(self, agent_name, waypoint_pos, waypoint_ori, waypoint_ind):
        self.reset_quadrotor_cache(agent_name)
        
        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}
        
        if waypoint_ind < len(waypoint_pos):
            move_done, move_success, move_info = self.quadrotor_move_with_waypoints(agent_name, waypoint_pos, waypoint_ori, waypoint_ind)
            info.update(move_info)
        else:
            land_pos = waypoint_pos[-1]
            land_ori = waypoint_ori[-1]
            
            done, success, _info = self._quadrotor_move_to_pose(agent_name, position=land_pos, orientation=land_ori)
            info.update(_info)

        if done and success:
            self.agents_dict[agent_name].active = False
        # print(done, success)
        return done, success, info
    


    
    def quadrotor_move_with_waypoints(self, agent_name, waypoint_pos, waypoint_ori, waypoint_ind):
        self.reset_quadrotor_cache(agent_name)
        
        done = False
        success = False
        info = {"waypoint_ind": waypoint_ind}

        if len(waypoint_pos) != len(waypoint_ori):
            done = True
            info["errorFlag"] = 4  # "Illegal dimenstions"
            return done, success, info
        if not self.agents_dict[agent_name].active:
            info["errorFlag"] = 7  # "Not takeoff"
            done = True
            return done, success, info
        if waypoint_ind != self.agents_waypoint_dict[agent_name]['cnt']:
            self.agents_waypoint_dict[agent_name]['cnt'] = waypoint_ind
            
        _waypoint_pos = waypoint_pos.copy()
        _waypoint_ori = waypoint_ori.copy()
        _waypoint_pos[:, 2] = quadrotor_defaults["hover_height"]

        sub_done, sub_success, sub_info = self._quadrotor_move_to_pose(agent_name, _waypoint_pos[waypoint_ind], _waypoint_ori[waypoint_ind])
        
        if sub_done and sub_success:
            waypoint_ind += 1
            if waypoint_ind == len(waypoint_pos):
                done = success = True
                info["waypoint_ind"] = len(waypoint_pos)  # Number of completed waypoints
                return done, success, info
            else:
                info["waypoint_ind"] = waypoint_ind
                return done, success, info
        if sub_done and not sub_success:
            # failed in waypoints[ind]

            done = True
            info.update(sub_info)
            
            return done, success, info


        return done, success, info
    
    
    def _quadrotor_move_to_pose(self, agent_name, position, orientation):
        self.reset_quadrotor_cache(agent_name)
        
        done = False
        success = False
        info = {}

        
        # scalar-first
        curr_pos, curr_ori = self.agents_dict[agent_name].get_curr_position_orientation()
        curr_yaw = get_xyz_euler_from_quaternion(curr_ori)[2]
        goal_yaw = get_xyz_euler_from_quaternion(orientation)[2]

        # print("goal:", position)
        # print("curr_pos:", curr_pos)
        diff_pos = position - curr_pos
        diff_yaw = goal_yaw - curr_yaw
        # print("diff_pos:", diff_pos)
        if np.linalg.norm(diff_pos) <= self._default_quadrotor_delta_pos and abs(diff_yaw) <= self._default_quadrotor_delta_turn:
            self.agents_dict[agent_name].attach()
            self.agents_dict[agent_name].set_curr_position_orientation(position, orientation)

            done = success = True
            return done, success, info
        


        temp_goal_pos = curr_pos.copy()
        temp_goal_yaw = curr_yaw
        if np.linalg.norm(diff_pos) <= self._default_quadrotor_delta_pos:
            temp_goal_pos += diff_pos
        else:
            temp_goal_pos += diff_pos / np.linalg.norm(diff_pos) * self._default_quadrotor_delta_pos
            
        if abs(diff_yaw) <= self._default_quadrotor_delta_turn:
            temp_goal_yaw += diff_yaw
        else:
            temp_goal_yaw += diff_yaw / abs(diff_yaw) * self._default_quadrotor_delta_turn
            

        temp_goal_quat = get_quaternion_from_euler(np.array([0, 0, temp_goal_yaw]))
        # work when sim.step()
        self.agents_dict[agent_name].attach()
        self.agents_dict[agent_name].set_curr_position_orientation(temp_goal_pos, temp_goal_quat)

        return done, success, info

