import rospy
import math
import numpy as np

from scipy.spatial.transform import Rotation as R


## msg
from hademo.msg import Action
from hademo.msg import Result, ResultInfo


from tasks import *


class ResultPublishandActionSubscribe:
    def __init__(self, task_name):
        self.task_name = task_name
        
        self.task = eval("defaults_%s" % self.task_name)
        # Define the publisher name, message type, and queue size.
        self._pub = rospy.Publisher('resultTopic', Result, queue_size=10, latch=True)
        self._sub = rospy.Subscriber('actionTopic', Action, self.callback)
        
        self.initial_pub()

        self._next_step_action = None

        self.pub_flag = False
    
    def initial_pub(self):
        print("Sim side waits for communication")
        flag = True
        while flag:
            published_topics_types = rospy.get_published_topics()

            for topic, type in published_topics_types:
                # LLM side is ready to begin
                if "/actionTopic" == topic:
                    flag = False
        # Sim side invoke communication
        print("Sim side invoke communication")
        self._pub.publish(Result())

    def publish_feedback_result(self, feedback_result):
        self._next_step_action = None
        result_msg = self.encode_feedback_result_to_result_msg(feedback_result)
        self._pub.publish(result_msg)

    def encode_feedback_result_to_result_msg(self, feedback_result):
        '''
            {
                "franka": {"has_result": bool, "success": bool, "info": string},
                "aliengo": {},
                "quadrotor": {}
            }
        '''
        result_msg = Result()


        return result_msg

    def getactionTemplate(self):
        t = dict()
        for name in self.task.agent_name_list:
            t[name] = []
        return t

    def decode_action_msg_to_next_step_action(self, action_msg):
        assert isinstance(action_msg, Action)
        # print(action_msg)
        # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&7777")
        next_step_action = self.getactionTemplate()
        for agent_name in self.task.agent_name_list:
            if eval("action_msg.%s.has_func" % agent_name):
                next_step_action[agent_name].append(eval("action_msg.%s.func_name" % agent_name))
                if eval("action_msg.%s.args.has_args" % agent_name):
                    # print(action_msg.aliengo)
                    if 'pick' in eval("action_msg.%s.func_name" % agent_name):
                        next_step_action[agent_name].append({
                            "agent_name": agent_name,
                            "attached_prim_path": eval("action_msg.%s.args.attached_prim_path" % agent_name),
                            "waypoint_pos": np.array(eval("action_msg.%s.args.waypoint_pos.data" % agent_name)).reshape(-1, 3),
                            "waypoint_ori": np.array(eval("action_msg.%s.args.waypoint_ori.data" % agent_name)).reshape(-1, 4),
                            "waypoint_ind": eval("action_msg.%s.args.waypoint_ind" % agent_name),
                        })
                    elif 'attached_prim' in eval("action_msg.%s.func_name" % agent_name):
                        next_step_action[agent_name].append({
                            "agent_name": agent_name,
                            "attached_prim_path": eval("action_msg.%s.args.attached_prim_path" % agent_name)
                        })
                    elif 'door_open' in eval("action_msg.%s.func_name" % agent_name):
                        next_step_action[agent_name].append({
                            "agent_name": agent_name,
                            "attached_prim_path": eval("action_msg.%s.args.attached_prim_path" % agent_name)
                        })
                    else:
                        next_step_action[agent_name].append({
                            "agent_name": agent_name,
                            "waypoint_pos": np.array(eval("action_msg.%s.args.waypoint_pos.data" % agent_name)).reshape(-1, 3),
                            "waypoint_ori": np.array(eval("action_msg.%s.args.waypoint_ori.data" % agent_name)).reshape(-1, 4),
                            "waypoint_ind": eval("action_msg.%s.args.waypoint_ind" % agent_name),
                        })
                else:
                    next_step_action[agent_name].append({"agent_name": agent_name})
        
        
        return next_step_action

    def callback(self, action_msg):
        
        # Simulation will know
        self._next_step_action = self.decode_action_msg_to_next_step_action(action_msg)
        
        



def main():
    rospy.init_node('Sim', anonymous=True)
    wake = ResultPublishandActionSubscribe()

    rospy.spin()


if __name__ == '__main__':
    main()