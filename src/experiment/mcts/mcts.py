# adapted from https://github.com/jys5609/MC-LAVE-RL.git

import numpy as np
from tqdm import tqdm
from llm_policy import LLMPolicy
import utils as utils
from collections import defaultdict
import copy
import re
import random


DISCOUNT_FACTOR = 0.95

class StateNode:
    def __init__(self, reward=0, done=False):
        self.ob = None
        self.look = None
        self.inv = None
        self.state = None
        self.prev_action = None
        self.id = None
        self.valid_actions = None
        self.history = []
        self.parent = None
        self.parent_action_id = None
        self.best_action_node = None
        self.ob_text = None 

        self.N = 0
        self.children = []
        self.children_probs = []
        self.reward = reward/(1-DISCOUNT_FACTOR)
        self.score = 0
        self.done = done
        self.predicted_reward = 0
        self.use_llm = False


class ActionNode:
    def __init__(self, action):
        self.action = action
        self.N = 0
        self.Q = 0
        self.Q_hat = 0
        self.Rs = []
        self.children = None
        self.children_id = None


class MCTSAgent:
    def __init__(self, args, env, policy=None, name='MCTS', 
                uct_type='PUCT', dict_list=None, valid_action_dict=None, actions_info=None,
                  log_dir=None, visited_transitions=None, replay_file=None,
                  use_llm=True):
        self.args = args
        self.env = env
        self.init_env = copy.deepcopy(env)
        self.name = name
        # self.num_actions = env.action_num
        self.best_action_node = None
        self.uct_type = uct_type
        self.init_graph = self.env.graph
        self.task_goal = self.env.task_goal
        self.init_id2node = {x['id']: x for x in self.env.graph['nodes']}
        self.satisfied = []
        self.unsatisfied = {}
        self.history = []
        self.dict_list = dict_list
        self.id_name_dict = self.env.id_name_dict

        # self.seed = args.seed
        self.round = args.round
        self.root = None


        self.exploration_constant = args.exploration_constant
        self.bonus_constant = args.bonus_constant
        self.max_depth = args.max_depth
        self.simulation_per_act = args.simulation_per_act
        self.discount_factor = args.discount_factor
        self.visited_transitions = visited_transitions

        self.action_selection_temp = 0.1 / (self.round + 1)

        self.policy = policy
        self.actions = [] if actions_info is None else actions_info[0]
        self.actions_e = [] if actions_info is None else actions_info[1]

        self.action_values = defaultdict(set)   # Ex: {north: [3.01, 2.00, 5.01]}

        self.maxlen_obs = 150
        self.maxlen_look = 150
        self.maxlen_inv = 50
        self.maxlen_action = 12
        self.simulation_num = args.simulation_num
        self.use_llm = use_llm
        if use_llm:
            self.llm_policy = LLMPolicy(args=args, device="cuda:0", model=args.model) 
        self.q_network = None

        self.state_dict = {}
        self.action_embedding = {}
        self.replay_file = replay_file

    def search(self, ob, history, cur_depth, valid_actions, valid_actions2agents, done):
        '''
        Search the best action with probs
        :return: best action
        '''
        init_history = history.copy()

        self.root = self.build_state(ob, history, valid_actions, done, use_llm=True)

        for i in tqdm(range(self.simulation_num)):

            self.history = init_history.copy()
            self.reset_env()
            print('simulation_num: ', i)
            _, root = self.simulate(self.root, 0, valid_actions2agents)
            self.root = root
        # select best action by Q-value
        best_action_node_idx = self.greedy_action_node(self.args, self.root, 0, 0, if_print=True)
        # select best action by Count
        # best_action_node = self.max_visit_action_node(self.root)
        best_action_node = self.root.children[best_action_node_idx]
        self.root.best_action_node = best_action_node
        return self.root.best_action_node.action

    @staticmethod
    def state_id(history: list):
        return ' '.join(history)

    def rebuild_state(self, state, ob, history, valid_actions, done, reward=0, prev_action='<s>', use_llm=False):
        state.id = self.state_id(history)
        # state.id = ob + info['look'] + info['inv'] + str(reward) + str(info['score']) + prev_action
        state.valid_actions = valid_actions
        state.use_llm = use_llm

        if not use_llm:
            state.children_probs = np.ones((len(state.valid_actions),)) / len(state.valid_actions)

        # elif state.id in self.state_dict.keys():
        #     state.children_probs = self.state_dict[state.id].children_probs
        self.state_dict[state.id] = state
        for valid_action in state.valid_actions:
            if isinstance(state.valid_actions, dict):
                state.children.append(ActionNode(state.valid_actions[valid_action]))
            else:
                state.children.append(ActionNode(valid_action))

        return state

    def build_state(self, ob, history, valid_actions, done, reward=0, prev_action='<s>', use_llm=False):
        state = StateNode()
        state.ob = ob

        state.state = ob
        state.done = done

        state.reward = reward

        state.prev_action = prev_action
        state.history = history
        state.id = self.state_id(history)

        state.valid_actions = valid_actions
        state.use_llm = use_llm
        agent_num = len(ob)
        obs2text = ''
        for i in range(len(ob)):
            obs2text += utils.agent_obs2text(ob, i, self.id_name_dict) + '\n'
        state.ob_text = obs2text

        if not use_llm:
            state.children_probs = np.ones((len(state.valid_actions),)) / len(state.valid_actions)

            
        else:
            state.children_probs = self.llm_policy._calculate_emperical_prob(
                history, obs2text, valid_actions, self.env.goal_instruction, agent_num, 10, 0, 0.95)
            
        self.state_dict[state.id] = state
        for valid_action in state.valid_actions:
            if isinstance(state.valid_actions, dict):
                state.children.append(ActionNode(state.valid_actions[valid_action]))
            else:
                state.children.append(ActionNode(valid_action))
        return state

        
    def simulate(self, state_node, depth, valid_actions2agents):

        if state_node.done or depth == self.max_depth:
            return 0, state_node

        best_action_node_idx = self.greedy_action_node(self.args, state_node, self.exploration_constant, self.bonus_constant)
        best_action_node = state_node.children[best_action_node_idx]
        rollout_next = False
        agent_action = best_action_node.action
        self.history.append(agent_action)
        print('simulate-depth', depth, 'simulate-action: ', agent_action)
        class_name = valid_actions2agents[agent_action]['class_name']
        real_id = valid_actions2agents[agent_action]['id']
        agent_action = agent_action.split(': ', 1)[1]
        # ob, reward, done, history, valid_actions = self.env.step(best_action_node.action)

        done, task_results,satisfied, unsatisfied,steps= self.env.step(class_name, real_id, agent_action, self.task_goal)
        ob = self.env.get_observations()
        env_graph = self.env.graph
        _, _, reward = self.get_reward(env_graph, self.task_goal)
        
        valid_actions, actions2agent = self.get_actions(ob, len(self.dict_list), self.dict_list)

        next_state_id = self.state_id(self.history)
        # path_of_nodes.append((state_node, best_action_node))
        if next_state_id == best_action_node.children_id:
            next_state_node = best_action_node.children
            '''
            if next_state_node.use_llm == False:
                next_state_node = self.build_state(ob, self.history, valid_actions, done, reward, prev_action=best_action_node.action, use_llm=self.use_llm)
                # best_action_node.children[index] = next_state_node
                next_state_node.parent = state_node
                rollout_next = True
            '''
        else: 
            next_state_node = self.build_state(ob, self.history, valid_actions, done, reward, prev_action=best_action_node.action, use_llm=True)
            next_state_node.parent = state_node
            best_action_node.children = next_state_node
            best_action_node.children_id = next_state_node.id
            rollout_next = True


        if rollout_next:
            if self.use_llm:
                rollout_r = []
                for _ in range(1):
                    random_r = reward + self.discount_factor * self.rollout(next_state_node, depth+1, actions2agent)
                    rollout_r.append(random_r)  
                R = sum(rollout_r)/len(rollout_r)
            else:
                rollout_r = []
                for _ in range(1):
                    random_r = reward + self.discount_factor * self.rollout(next_state_node, depth+1, actions2agent)
                    rollout_r.append(random_r)  
                R = sum(rollout_r)/len(rollout_r)
        else:
            r, next_state_node = self.simulate(next_state_node, depth+1, actions2agent)
            R = reward + self.discount_factor * r

        state_node.N += 1
        best_action_node.N += 1
        best_action_node.children = next_state_node
        best_action_node.Rs.append(R)
        best_action_node.Q = np.sum(np.array(best_action_node.Rs) * utils.softmax(best_action_node.Rs, T=10))
        state_node.best_action_node = best_action_node       
        return R, state_node
    '''
    def max_visit_action_node(self, state_node):
        children_count = []

        for i in range(len(state_node.children)):
            child = state_node.children[i]
            children_count.append(child.N)

        children_count = children_count / np.max(children_count)
        count_based_probs = children_count ** (1/self.action_selection_temp) / (np.sum(children_count ** (1/self.action_selection_temp)))
        return np.random.choice(state_node.children, p=count_based_probs)
    '''
    def greedy_action_node(self, args, state_node, exploration_constant, bonus_constant, if_print=False):
        best_value = -np.inf
        best_children = []
        best_children_prob = []
        for i in range(len(state_node.children)):
            child = state_node.children[i]
            assert len(state_node.children_probs) == len(state_node.children), print(state_node.children_probs)
            child_prob = state_node.children_probs[i]
            # if if_print:
            #     print('action:',child, 'probility:',child_prob)
            #     with open(args.log_path, 'a') as f:
            #         f.write('action:'+str(child)+' probility:'+str(child_prob)+'\n')
            
            if exploration_constant == 0:
                ucb_value = child.Q
            elif self.uct_type == 'UCT':
                ucb_value = child.Q + exploration_constant * np.sqrt(np.log(state_node.N + 1) / (child.N + 1))
                # print(child.Q, exploration_constant * np.sqrt(np.log(state_node.N + 1) / (child.N + 1)))
            elif self.uct_type == 'PUCT':
                # print(child_prob)
                ucb_value = child.Q + exploration_constant * child_prob * np.sqrt(state_node.N) / (child.N + 1)
            elif self.uct_type == 'MC-LAVE':
                if child.action in self.action_embedding.keys():
                    action_e = self.action_embedding[child.action]
                else:
                    action_e = utils.vectorize(child.action)
                    self.action_embedding[child.action] = action_e

                actions = list(self.action_values.keys())
                if child.action in actions:
                    actions.pop(actions.index(child.action))

                actions_e = []
                for a in actions:
                    actions_e.append(self.action_embedding[a])

                near_act, near_idx = utils.find_near_actions(action_e, actions, np.array(actions_e), threshold=0.8)
                if len(near_idx) == 0:
                    child.Q_hat = 0
                else:
                    near_Qs = set()
                    for a in near_act:
                        near_Qs.add(np.mean(list(self.action_values[a])))
                    near_Qs = list(near_Qs)
                    child.Q_hat = utils.softmax_value(near_Qs)

                ucb_value = child.Q \
                            + exploration_constant * np.sqrt(state_node.N + 1) / (child.N + 1) * child_prob \
                            + bonus_constant * np.sqrt(state_node.N + 1) / (child.N + 1) * child.Q_hat

            else:
                raise NotImplementedError

            if ucb_value == best_value:
                best_children.append(i)
                best_children_prob.append(child_prob)
            elif ucb_value > best_value:
                best_value = ucb_value
                best_children = [i]
                best_children_prob = [child_prob]
        if if_print:
            child_index = 0
            for c in state_node.children:
                # if c.N > 0:
                print('List of executable actions:',c.action, ' Q value of action:',c.Q, ' Number of visits to the action:',c.N, ' Number of visits to the action:'+str(state_node.children_probs[child_index])+'\n')
                with open(args.log_path, 'a') as f:
                    f.write('List of executable actions:'+str(c.action)+' Q value of action:'+str(c.Q)+' Number of visits to the action:'+str(c.N)+' Number of visits to the action:'+str(state_node.children_probs[child_index])+'\n')
                child_index += 1
        best_children_prob = np.array(best_children_prob) / np.sum(best_children_prob)
        output_action_index = np.argmax(best_children_prob)
        return best_children[output_action_index]

    def rollout(self, state_node, depth, valid_actions2agents):
        if state_node.done or depth == self.max_depth:
            return 0
        action_node = np.random.choice(state_node.children, 1)[0]
        action = action_node.action
        self.history.append(action)
        print('rollout-depth: ', depth, 'rollout-action: ', action)
        # print('rollout-action: ', action)

        class_name = valid_actions2agents[action]['class_name']
        real_id = valid_actions2agents[action]['id']
        action = action.split(': ', 1)[1]


        done, task_results,satisfied, unsatisfied,steps= self.env.step(class_name, real_id, action, self.task_goal)
        ob = self.env.get_observations()
        env_graph = self.env.graph
        _, _, reward = self.get_reward(env_graph, self.task_goal)

        valid_actions, actions2agent = self.get_actions(ob, len(self.dict_list), self.dict_list)

        if done:
            print("Done!")
        next_state_id = self.state_id(self.history)


        if next_state_id == action_node.children_id:
            next_state_node = action_node.children
        else:
            next_state_node = self.build_state(ob, self.history, valid_actions, done, reward, prev_action=action)
            next_state_node.parent = state_node
            action_node.children = next_state_node
            action_node.children_id = next_state_node.id
        r = reward + self.discount_factor * self.rollout(next_state_node, depth+1, actions2agent)
        return r

    def get_actions(self, obs, agent_num, dict_list):
        
        actions = []
        action_num = 0
        action2agent = {}
        for i in range(agent_num):
            action = self.get_action(obs[i], dict_list[i]['agent_node'], self.env.task_goal)
            class_name = dict_list[i]['agent_node']['class_name']
            agent_id = dict_list[i]['agent_node']['id']
            new_action = [] 
            for a in action:
                new_a = f'<{class_name}>({agent_id}): ' + a
                new_action.append(new_a)
            for num in range(len(action)):
                action2agent[new_action[num]] = {'class_name': class_name, 'id': agent_id}
            actions = actions + new_action
            random.shuffle(actions)
        return actions, action2agent


    def get_action(self, observation, agent_node, goal):
		
        satisfied, unsatisfied = self.check_progress(observation, goal) 
        # print(f"satisfied: {satisfied}")
        if len(satisfied) > 0:
            self.unsatisfied = unsatisfied
            self.satisfied = satisfied
        
        self.agent_node = agent_node
        obs = observation
        self.grabbed_objects = None
        self.reachable_objects = []
        self.landable_surfaces = None
        self.on_surfaces = None
        self.all_landable_surfaces = []
        self.all_landable_surfaces = [x for x in obs['nodes'] if 'LANDABLE' in x['properties']]
        self.on_same_surfaces = []
        self.on_same_surfaces_ids = []
        # self.chat_agent_info = chat_agent_info

        self.id2node = {x['id']: x for x in obs['nodes']}

        for e in obs['edges']:
            x, r, y = e['from_id'], e['relation_type'], e['to_id']
            
            if x == self.agent_node['id']:

                if r == 'INSIDE':
                    self.current_room = self.id2node[y]
                if r == 'ON' :
                    self.on_surfaces = self.id2node[y]
                    if self.agent_node['class_name'] == 'robot arm' or self.agent_node['class_name'] == 'robot_arm':
                        for i in range(3):
                            for edge in obs['edges']:
                                if (edge['from_id'] != x and edge['to_id'] == y and edge['relation_type'] == 'ON') or (edge['from_id'] != x and edge['to_id'] in self.on_same_surfaces_ids and edge['relation_type'] == 'ON') or (edge['from_id'] != x and edge['to_id'] in self.on_same_surfaces_ids and edge['relation_type'] == 'INSIDE') :
                                    self.on_same_surfaces_ids.append(edge['from_id'])
                                    #self.on_same_surfaces.append(self.id2node[edge['from_id']])  # Find any contain or surface on the table
                                    if 'SURFACES' in self.id2node[edge['from_id']]['properties'] or 'CONTAINERS' in self.id2node[edge['from_id']]['properties']:
                                        for ee in obs['edges']:
                                            if ee['to_id'] == edge['from_id'] and (ee['relation_type'] == 'INSIDE' or ee['relation_type'] == 'ON'):
                                                self.on_same_surfaces_ids.append(ee['from_id'])
                                                #self.on_same_surfaces.append(self.id2node[ee['from_id']]) # The goal here is to find objects that are not directly on the surface
                                self.on_same_surfaces_ids = list(set(self.on_same_surfaces_ids))	
                        for id in self.on_same_surfaces_ids:
                            self.on_same_surfaces.append(self.id2node[id])						
                
                if r == 'HOLD':
                    # self.grabbed_objects.append(y)
                    self.grabbed_objects = self.id2node[y]
                if r == 'CLOSE':
                    self.reachable_objects.append(self.id2node[y])
                if r == 'ABOVE' and 'LANDABLE' in self.id2node[y]['properties']:
                    self.landable_surfaces = self.id2node[y]

        self.unreached_objects = copy.deepcopy(obs['nodes'])
        for node in obs['nodes']:
            if node == self.grabbed_objects or node in self.reachable_objects:
                self.unreached_objects.remove(node)
            elif node['category'] == 'Rooms' or node['category'] == 'Agents' or node['category'] == 'Floor' or "HIGH_HEIGHT" in node['properties'] or 'ON_HIGH_SURFACE' in node['properties']:
                self.unreached_objects.remove(node)  #The idea here is to find the places that the robotic dog has not reached, remove what it already has in its hand, remove what it is close to, remove the room, the floor, the agent itself, the high surface and what is on the high surface

        self.doors = []
        self.next_rooms = []
        self.doors = [x for x in obs['nodes'] if x['class_name'] == 'door']
        for door in self.doors:
            for edge in self.init_graph['edges']:
                if edge['relation_type'] == "LEADING TO" and edge['from_id'] == door['id'] and edge['to_id'] != self.current_room["id"]:
                        self.next_rooms.append([self.init_id2node[edge['to_id']], door])

        info = {'graph': obs,
                "obs": {	
                        "agent_class": self.agent_node["class_name"],
                        "agent_id":self.agent_node["id"],
                        "grabbed_objects": self.grabbed_objects,
                        "reachable_objects": self.reachable_objects,
                        "on_surfaces": self.on_surfaces,
                        "landable_surfaces": self.landable_surfaces,
                        "doors": self.doors,
                        "next_rooms": self.next_rooms,
                        "objects_on_the_same_surfaces": self.on_same_surfaces,
                        "satisfied": self.satisfied,
                        "current_room": self.current_room['class_name'],
                        },
                }
        available_plans, num, available_plans_list = self.get_available_plans(self.agent_node, self.next_rooms, self.all_landable_surfaces, self.landable_surfaces, self.on_surfaces, self.grabbed_objects, self.reachable_objects, self.unreached_objects, self.on_same_surfaces
																		 )
        # message, a_info = self.LLM_plan()
        # if a_info['plan'] is None: 
        #     print("No more things to do!")
        # plan = a_info['plan']
        # a_info.update({"steps": self.steps})
        # info.update({"LLM": a_info})

        return available_plans_list

    def get_available_plans(self, agent_node, next_rooms, all_landable_surfaces, landable_surfaces, on_surfaces, 
						 grabbed_objects, reached_objects, unreached_objecs, on_same_surface_objects
						 ):
        """
        'quadrotor':
        [land_on] <surface>
        [movetowards] <surface>/<next_room>
        [takeoff_from] <surface>

        'robot dog':
        [open] <container>/<door>
        [close] <container>/<door>
        [grab] <object>
        [putinto] <object> into <container>
        [puton] <object> on <surface>
        [movetowards] <object>

        'robot arm':
        [open] <container>
        [close] <container>
        [grab] <object>
        [putinto] <object> into <container>
        [puton] <object> on <surface>

        """
        available_plans = []
        if agent_node["class_name"] == "quadrotor":
            other_landable_surfaces = []
            if "FLYING" in agent_node["states"]:
                if landable_surfaces is not None:
                    available_plans.append(f"[land_on] <{landable_surfaces['class_name']}>({landable_surfaces['id']})")
                    all_landable_surfaces.remove(landable_surfaces)
                    other_landable_surfaces = copy.deepcopy(all_landable_surfaces)
                if len(other_landable_surfaces) != 0:
                    for surface in other_landable_surfaces :
                        available_plans.append(f"[movetowards] <{surface['class_name']}>({surface['id']})")
                for next_room in next_rooms:
                    if 'OPEN' in next_room[1]['states'] or "OPEN_FOREVER" in next_room[1]['states']:
                        available_plans.append(f"[movetowards] <{next_room[0]['class_name']}>({next_room[0]['id']})")

            if "LAND" in agent_node["states"]:
                if on_surfaces is not None:
                    available_plans.append(f"[takeoff_from] <{on_surfaces['class_name']}>({on_surfaces['id']})")

        if agent_node["class_name"] == "robot dog" or agent_node["class_name"] == "robot_dog":
            # if grabbed_objects is not None:
            # 	available_plans.append(f"[puton] <{grabbed_objects['class_name']}>({grabbed_objects['id']}) on <{on_surfaces['class_name']}>({on_surfaces['id']})")
            # The robotic dog is not allowed to put things on the floor. If it needs to open the door and has something in its hand, it needs to find a low surface to put things first
            if len(reached_objects) != 0:
                for reached_object in reached_objects:
                    if grabbed_objects is None:
                        if 'CONTAINERS' in reached_object['properties'] and 'CLOSED' in reached_object['states'] or \
                            reached_object['class_name'] == 'door' and 'CLOSED' in reached_object['states']:
                            available_plans.append(f"[open] <{reached_object['class_name']}>({reached_object['id']})")
                        if 'CONTAINERS' in reached_object['properties'] and 'OPEN' in reached_object['states'] or \
                            reached_object['class_name'] == 'door' and 'OPEN' in reached_object['states']:
                            available_plans.append(f"[close] <{reached_object['class_name']}>({reached_object['id']})")
                        if 'GRABABLE' in reached_object['properties']:
                            available_plans.append(f"[grab] <{reached_object['class_name']}>({reached_object['id']})")
                    if grabbed_objects is not None:
                        if 'CONTAINERS' in reached_object['properties'] and ('OPEN' in reached_object['states'] or "OPEN_FOREVER" in reached_object['states']):
                            available_plans.append(f"[putinto] <{grabbed_objects['class_name']}>({grabbed_objects['id']}) into <{reached_object['class_name']}>({reached_object['id']})")
                        if 'SURFACES' in reached_object['properties']:
                            available_plans.append(f"[puton] <{grabbed_objects['class_name']}>({grabbed_objects['id']}) on <{reached_object['class_name']}>({reached_object['id']})")
            
            if len(unreached_objecs) != 0:
                for unreached_object in unreached_objecs:
                    available_plans.append(f"[movetowards] <{unreached_object['class_name']}>({unreached_object['id']})")
            for next_room in next_rooms:
                    if 'OPEN' in next_room[1]['states'] or "OPEN_FOREVER" in next_room[1]['states']:
                        available_plans.append(f"[movetowards] <{next_room[0]['class_name']}>({next_room[0]['id']})")


        if agent_node['class_name'] == 'robot arm' or agent_node['class_name'] == 'robot_arm':
            if grabbed_objects is not None:
                available_plans.append(f"[puton] <{grabbed_objects['class_name']}>({grabbed_objects['id']}) on <{on_surfaces['class_name']}>({on_surfaces['id']})")
            for on_same_surface_object in on_same_surface_objects:
                if grabbed_objects is None:
                    if 'CONTAINERS' in on_same_surface_object['properties'] and 'OPEN' in on_same_surface_object['states']:
                        available_plans.append(f"[close] <{on_same_surface_object['class_name']}>({on_same_surface_object['id']})")
                    if 'CONTAINERS' in on_same_surface_object['properties'] and 'CLOSED' in on_same_surface_object['states']:
                        available_plans.append(f"[open] <{on_same_surface_object['class_name']}>({on_same_surface_object['id']})")
                    if 'GRABABLE' in on_same_surface_object['properties']:
                        available_plans.append(f"[grab] <{on_same_surface_object['class_name']}>({on_same_surface_object['id']})")

                if grabbed_objects is not None:
                    
                    if 'CONTAINERS' in on_same_surface_object['properties'] and ('OPEN' in on_same_surface_object['states'] or "OPEN_FOREVER" in on_same_surface_object['states']):
                        available_plans.append(f"[putinto] <{grabbed_objects['class_name']}>({grabbed_objects['id']}) into <{on_same_surface_object['class_name']}>({on_same_surface_object['id']})")
                    if 'SURFACES' in on_same_surface_object['properties']:
                        available_plans.append(f"[puton] <{grabbed_objects['class_name']}>({grabbed_objects['id']}) on <{on_same_surface_object['class_name']}>({on_same_surface_object['id']})")

        plans = ""
        for i, plan in enumerate(available_plans):
            plans += f"{chr(ord('A') + i)}. {plan}\n"
        # print(agent_node["class_name"],agent_node['id'])
        # print(available_plans)
        return plans, len(available_plans), available_plans

    def check_progress(self, state, goal_spec):
        unsatisfied = {}
        satisfied = []
        id2node = {node['id']: node for node in state['nodes']}

        for key, value in goal_spec.items():
            elements = key.split('_')
            goal_location_id = int((re.findall(r'\((.*?)\)', elements[-1]))[0])
            target_object_id = int((re.findall(r'\((.*?)\)', elements[1]))[0])
            cnt = value[0]
            for edge in state['edges']:
                if cnt == 0:
                    break
                if edge['relation_type'].lower() == elements[0] and edge['to_id'] == goal_location_id and edge['from_id'] == target_object_id:
                    satisfied.append(id2node[edge['from_id']])  # A list of nodes that meet the goal
                    cnt -= 1
                    # if self.debug:
                    # 	print(satisfied)
            if cnt > 0:
                unsatisfied[key] = value  
        return satisfied, unsatisfied 
     
    def get_reward(self, state, task_goal):
        # reward = 0
        unsatisfied = {}
        satisfied = {}
        all_state_nodes = state['nodes']
        all_state_edges = state['edges']

        for key, value in task_goal.items():
            elements = key.split('_')
            goal_location_id = int((re.findall(r'\((.*?)\)', elements[-1]))[0])
            target_object_id = int((re.findall(r'\((.*?)\)', elements[1]))[0])
            target_object_class = re.findall(r'<(.*?)>', elements[1])[0]
                    
            cnt = value[0]

            for edge in all_state_edges:
                if edge['relation_type'].lower() == elements[0] and edge['to_id'] == goal_location_id and edge['from_id'] == target_object_id:
                    cnt -= 1

            if cnt > 0:
                unsatisfied[key] = [target_object_id, goal_location_id, cnt] 

        if len(unsatisfied) == 0:
            reward = 10
        else:
            reward = 0    
        return satisfied, unsatisfied, reward
    
    def reset_env(self):
        self.env = copy.deepcopy(self.init_env)
        self.init_graph = self.env.graph
        self.task_goal = self.env.task_goal
        self.init_id2node = {x['id']: x for x in self.env.graph['nodes']}
