
from omni.isaac.kit import SimulationApp

app = SimulationApp({"headless": False})

import math
import numpy as np
from omni.isaac.franka.tasks import FollowTarget
from omni.isaac.franka.controllers import RMPFlowController
from omni.isaac.core import World
from omni.isaac.franka import KinematicsSolver, Franka
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from scipy.spatial.transform import Rotation as R
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import carb

class DistanceSet:
    def __init__(self):
        self.delta_pose = 0.01
        self.delta_position = 0.01

    def reach_position(self, my_pos, goal_pos):
        delta_pos = self._judge_pos_close(my_pos, goal_pos)
        if delta_pos < self.delta_position:
            return True
        else: 
            return False
        
    def reach_pose(self, my_pos, my_ori, goal_pos, goal_ori):
        delta_pos = self._judge_pos_close(my_pos, goal_pos)
        delta_ori = self._judge_qua_close(my_ori, goal_ori)
        return delta_pos, delta_ori

    
    def _judge_pos_close(self, p0, p1):
        delta_pos = np.linalg.norm(p0-p1)
        print('delta_pos', delta_pos)
        return delta_pos


    def _judge_qua_close(self, q0_scalar_first,q1_scalar_first):
        # input (w, x, y, z)
        q0 = np.array([q0_scalar_first[1], q0_scalar_first[2], q0_scalar_first[3], q0_scalar_first[0]])
        q1 = np.array([q1_scalar_first[1], q1_scalar_first[2], q1_scalar_first[3], q1_scalar_first[0]])

        #input   q0,q1  (x,y,z,w)
        q11 = np.array([-q1[0],-q1[1],-q1[2],q1[3]])   #q1的逆
        mul = self._qua_multi(q11,q0,0)    #  q0 * q1逆
        logq = self._qua_log(mul)
        dis_qua = np.linalg.norm(logq) *2.0
        print('dis_qua', dis_qua)
        return dis_qua
    
    def _qua_multi(self, qa,qb,m):
        #qa,qb均为四元数,[x,y,z,w],实轴在最后
        if(m==0):        
            #qb左乘qa
            qa_m=[[qa[3],-qa[2],qa[1],qa[0]],
            [qa[2],qa[3],-qa[0],qa[1]],
            [-qa[1],qa[0],qa[3],qa[2]],
            [-qa[0],-qa[1],-qa[2],qa[3]]]   
            qc=np.matmul(qa_m,qb)

        if(m==1):      
            #qb右乘qa
            qa_m=[[qa[3],qa[2],-qa[1],qa[0]],
            [-qa[2],qa[3],qa[0],qa[1]],
            [qa[1],-qa[0],qa[3],qa[2]],
            [-qa[0],-qa[1],-qa[2],qa[3]]]
            qc=np.matmul(qa_m,qb)
        return qc

    def _qua_log(self, q0):
        #定义四元数的对数:
            # q在无旋转四元数[0,0,0,1]的切平面的投影定义为四元数的对数
            # input q(4,)  四元数 x y z w
            # output log(q) (3,)
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

dis = DistanceSet()

my_world = World(stage_units_in_meters=1.0)
my_franka = Franka(prim_path='/World/franka', name='franka')

def create_cube(agent):
    # _quat_cube2world_back = R.from_euler('XYZ', [-180, 0, 180], degrees=True)
    
    _quat_cube2world_front = euler_angles_to_quat(np.array([-np.pi, 0, np.pi]))
 
    return FixedCuboid(
            prim_path="/World/target_cube",
            name="target_cube",
            position=np.array([0, 0.1, 0.7]),  # agent._init_position + np.array([0.5, 0, 0.5]),
            orientation=_quat_cube2world_front,
            scale=np.array([0.02, 0.02, 0.02]),
            color=np.array([0, 0, 1.0]),
        )


target_cube = create_cube(agent=my_franka)

my_world.reset()
my_franka.initialize()

my_controller = KinematicsSolver(my_franka)
articulation_controller = my_franka.get_articulation_controller()
while app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        observations = my_world.get_observations()

        target_position, target_orientation = target_cube.get_world_pose()
        actions, succ = my_controller.compute_inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation,
        )
        if succ:
            articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

        ee_pos, mat = my_controller.compute_end_effector_pose()
        quat = R.from_matrix(mat).as_quat()
        quat_scalar_first = np.array([quat[1], quat[2], quat[3], quat[0]])
        if dis.reach_pose(my_pos=ee_pos, my_ori=quat_scalar_first, goal_pos=target_position, goal_ori=target_orientation):
            print("reach!!!")
        else:
            print("approach!!!")
app.close()