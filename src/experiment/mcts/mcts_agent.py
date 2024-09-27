import sys
# sys.path.append('/home/lkh/Project/COHERENT/src/exp')
from mcts import MCTSAgent

import pickle
import argparse
import time
import copy
import json
import random
from datetime import datetime
from get_env_info import Get_env_info

class mcts_vh_env:
    def __init__(self, graph, goal_spec, task_goal):
        super().__init__()
        self.goal_spec = goal_spec
        self.task_goal = task_goal
        # self.vh_pyenv.pomdp = True
        self.model = None
        self.history = []
        self.init_history = []
        self.cur_state_graph = graph
        # self.cur_state = self.vh_pyenv.get_vh_state(graph)
        self.init_state = copy.deepcopy(self.cur_state)
        self.init_graph = self.init_state.to_dict()
        self.belief = None
    
    def filtering_graph(self, graph):
        new_edges = []
        edge_dict = {}
        for edge in graph['edges']:
            key = (edge['from_id'], edge['to_id'])
            if key not in edge_dict:
                edge_dict[key] = [edge['relation_type']]
                new_edges.append(edge)
            else:
                if edge['relation_type'] not in edge_dict[key]:
                    edge_dict[key] += [edge['relation_type']]
                    new_edges.append(edge)

        graph['edges'] = new_edges
        return graph

    def sample_belief(self):
        new_graph = self.belief.sample_from_belief()
        new_graph = self.filtering_graph(new_graph)
        return new_graph

    def update_and_sample_belief(self, obs_graph):
        new_graph = self.belief.update_graph_from_gt_graph(obs_graph)
        new_graph = self.filtering_graph(new_graph)
        return new_graph
    

    def check_progress(self, state, goal_spec):
        """TODO: add more predicate checkers; currently only ON"""
        count = 0
        for key, value in goal_spec.items():
            if key.startswith('off'):
                count += value
        id2node = {node['id']: node for node in state['nodes']}
        for key, value in goal_spec.items():
            elements = key.split('_')
            for edge in state['edges']:
                if elements[0] in ['on', 'inside']:
                    if edge['relation_type'].lower() == elements[0] and edge['to_id'] == int(elements[2]) and (id2node[edge['from_id']]['class_name'] == elements[1] or str(edge['from_id']) == elements[1]):
                        count += 1
                elif elements[0] == 'offOn':
                    if edge['relation_type'].lower() == 'on' and edge['to_id'] == int(elements[2]) and (id2node[edge['from_id']]['class_name'] == elements[1] or str(edge['from_id']) == elements[1]):
                        count -= 1
                elif elements[1] == 'offInside':
                    if edge['relation_type'].lower() == 'inside' and edge['to_id'] == int(elements[2]) and (id2node[edge['from_id']]['class_name'] == elements[1] or str(edge['from_id']) == elements[1]):
                        count -= 1
                elif elements[0] == 'holds':
                    if edge['relation_type'].lower().startswith('holds') and id2node[edge['to_id']]['class_name'] == elements[1] and edge['from_id'] == int(elements[2]):
                        count += 1
                elif elements[0] == 'sit':
                    if edge['relation_type'].lower().startswith('on') and edge['to_id'] == int(elements[2]) and edge['from_id'] == int(elements[1]):
                        count += 1
            if elements[0] == 'turnOn':
                if 'ON' in id2node[int(elements[1])]['states']:
                    count += 1
        goals = sum([value[0] for key, value in goal_spec.items()])
        if count < goals:
            reward = 0
        else:
            reward = 10
        return reward

    def graph_to_text(self, obs_graph):
        list_obs_item = [node['class_name'] for node in obs_graph['nodes']]
        list_obs_item = list(set(list_obs_item))
        observation = 'Visible objects are: ' + ', '.join(list_obs_item)
        return observation

    def get_observations_text(self, graph_env=None, char_index=0):
        par_obs = self.vh_pyenv.get_observations()
        par_obs_text = self.graph_to_text(par_obs)
        return par_obs_text
    
    # def reset(self, obs=None, graph=None, goal_spec=None, task_goal=None):
    #     if graph is None:
    #         graph = self.init_graph
    #     else:
    #         self.belief = Belief(graph, agent_id=1)
    #         graph = self.sample_belief()
    #         self.init_graph = graph
    #     obs = self.vh_pyenv.reset(graph)

    #     self.cur_state_graph = graph
    #     self.cur_state = self.vh_pyenv.get_vh_state(graph)
    #     self.init_state = copy.deepcopy(self.cur_state)
    #     self.init_graph = copy.deepcopy(graph)
    #     self.goal_spec = goal_spec if goal_spec is not None else self.goal_spec
    #     self.task_goal = task_goal if task_goal is not None else self.task_goal

    #     self.history = []
    #     self.init_history = []
    #     valid_action_space = self.get_valid_action(obs)
    #     return obs, valid_action_space

    # def get_valid_action(self, obs, agent_id=0):
    #     valid_action_space = []
    #     valid_action_space_dict = get_valid_actions(obs, agent_id)
    #     for action in valid_action_space_dict:
    #         interact_item_idxs = valid_action_space_dict[action]
    #         action = action.replace('walktowards', 'walk')
    #         if 'put' in action:
                
    #             valid_action_space += [
    #                 f'[{action}] <{grab_name}> ({grab_id}) <{item_name}> ({item_id})'
    #                     for grab_id, grab_name, item_id, item_name in interact_item_idxs]
    #         else:
    #             valid_action_space += [
    #                 f'[{action}] <{item_name}> ({item_id})'
    #                     for item_id, item_name in interact_item_idxs if item_name not in ['wall', 'floor', 'ceiling', 'curtain', 'window']]
        
    #     return valid_action_space
    
    def action_to_text(self, action):
        return None
    
    def get_action_from_text(self, text_action):
        return None

    def copy_env(self):
        self.reset(self.init_graph, self.goal_spec, self.task_goal)
        return self

    # def get_goal(self):
    #     goal = self.task_goal[0]
    #     task_goal_languages = get_goal_language(goal, self.init_graph)
    #     task_goal = 'Goal: ' + ', '.join(task_goal_languages) + '.'
    #     return task_goal

    # @staticmethod
    # def get_goal_(formal_goal, init_graph):
    #     task_goal_languages = get_goal_language(formal_goal, init_graph)
    #     task_goal = 'Goal: ' + ', '.join(task_goal_languages) + '.'
    #     return task_goal

    def update(self, action, obs):
        self.vh_pyenv.step({0: action}) 
        self.cur_state = self.vh_pyenv.vh_state
        self.cur_state_graph = self.vh_pyenv.state
        obs = self.vh_pyenv._mask_state(self.cur_state_graph, 0)
        text_obs = self.graph_to_text(obs)
        if action is not None:
            self.history.append(action)
        valid_actions = self.get_valid_action([obs])
        reward = self.check_progress(self.cur_state_graph, self.goal_spec)
        if reward <= 0:
            done = False 
        else:
            done = True
        self.init_graph = copy.deepcopy(self.cur_state_graph)
        self.init_state = copy.deepcopy(self.cur_state)
        return text_obs, reward, done, self.history, valid_actions   

    def update_(self, action, obs):
        if action is not None:
            self.vh_pyenv.step({0: action}) 
        self.cur_state_graph = self.update_and_sample_belief(obs)
        self.cur_state = self.vh_pyenv.get_vh_state(self.cur_state_graph)
        text_obs = self.graph_to_text(obs)
        if action is not None:
            self.history.append(action)
        valid_actions = self.get_valid_action([obs])
        reward = self.check_progress(self.cur_state_graph, self.goal_spec)
        if reward <= 0:
            done = False 
        else:
            done = True
        self.init_graph = copy.deepcopy(self.cur_state_graph)
        self.init_state = copy.deepcopy(self.cur_state)
        return text_obs, reward, done, self.history, valid_actions   
    
    def step(self, action):
        obs = self.vh_pyenv._mask_state(self.cur_state_graph, 0)
        valid_actions = self.get_valid_action([obs])
        try:
            self.cur_state, succeed = self.vh_pyenv.transition(self.cur_state, {0: action}) 
        except:
            print(action)
        self.cur_state_graph = self.cur_state.to_dict()
        obs = self.vh_pyenv._mask_state(self.cur_state_graph, 0)
        plate_ids = []
        text_obs = self.graph_to_text(obs)
        self.history.append(action)

        valid_actions = self.get_valid_action([obs])
        reward = self.check_progress(self.cur_state_graph, self.goal_spec)
        if reward <= 0:
            done = False 
        else:
            done = True
        return text_obs, reward, done, self.history, valid_actions



def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exploration_constant', default=24, type=int)
    parser.add_argument('--bonus_constant', default=1, type=int)
    parser.add_argument('--max_episode_len', default=50, type=int)
    parser.add_argument('--max_depth', default=20, type=int)
    parser.add_argument('--round', default=0, type=int)
    parser.add_argument('--simulation_per_act', default=2, type=int)
    parser.add_argument('--simulation_num', default=50, type=int)
    parser.add_argument('--discount_factor', default=0.95, type=float)
    parser.add_argument('--uct_type', default='PUCT', type=str)
    parser.add_argument('--mode', default='simple', type=str)
    parser.add_argument('--save_cache', action='store_true', default=False)
    parser.add_argument('--load_cache', action='store_true', default=False)
    parser.add_argument('--seen_item', action='store_true', default=True)
    parser.add_argument('--seen_apartment', action='store_true', default=True)
    parser.add_argument('--seen_comp', action='store_true', default=True)
    parser.add_argument('--load_path', default=None, type=str)
    parser.add_argument('--batch_size', default=128, type=int)
    parser.add_argument('--evaluate', default=True)
    parser.add_argument('--model', default="gpt-4o-mini", type=str)
    parser.add_argument('--use_llm', default=True, type=bool)
    parser.add_argument('--env', type=str, required=True, choices=['env0', 'env1', 'env2', 'env3', 'env4'],
                    help='Select a simulation environment')
    parser.add_argument('--task', type=int, required=True, nargs='+', 
                    help='Specifies the ID of the task to run. Enter at least one parameter.')
    parser.add_argument('--api_key', default='',
                    help='please enter your openai api_key')
    parser.add_argument('--organization', default='', 
                    help='please enter your openai organization')

    return parser.parse_args()

# def find_test_data_file_path(args):
#     file_path = f'./vh/dataset/env_task_set_50_{args.mode}_'
#     if not args.seen_item:
#         file_path += 'unseen_item.pik'
#     elif not args.seen_apartment:
#         file_path += 'unseen_apartment.pik'
#     elif not args.seen_comp:
#         file_path += 'unseen_composition.pik'
#     else:
#         file_path += 'seen.pik'
#     return file_path


def get_actions(obs, agent_num, agents, env, dict_list):
   
    actions = []
    action_num = 0
    action2agent = {}
    for i in range(agent_num):
        action = agents.get_action(obs[i], dict_list[i]['agent_node'], env.task_goal)
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

def main():
    args = parse_args()

    if args.use_llm == True:
        args.log_path = './select_action_llm_mcts.txt'
    else:
        args.log_path = './select_action_no_llm_mcts.txt'
    
    with open(f'./env/{args.env}.json') as file:
        data = json.load(file)
    tasklist = args.task
    
    '''
    for i in range(5):
        if i == 0:
            with open('./env/env0.json') as file:
                data = json.load(file)
            tasklist = [2, 4, 9, 10, 11, 15, 16, 20]
        if i == 1:
            with open('./env/env1.json') as file:
                data = json.load(file)
            tasklist = [1, 3, 7, 8, 11, 10, 16, 20]
        if i == 2:
            with open('./env/env2.json') as file:
                data = json.load(file)
            tasklist = [3, 5, 6, 7, 10, 11, 16, 17]
        if i == 3:
            with open('./env/env3.json') as file:
                data = json.load(file)
            tasklist = [2, 4, 6, 7, 10, 16, 17, 19]
        if i == 4:
            with open('./env/env4.json') as file:
                data = json.load(file)
            tasklist = [0, 1, 7, 10, 12, 17, 18, 19]
        '''
    for task_id in tasklist:
        env_id = data[task_id]["env_id"]
        task_name = data[task_id]["task_name"]
        init_graph = data[task_id]["init_graph"]
        task_goal = data[task_id]["task_goal"]
        goal_instruction = data[task_id]["goal_instruction"]
        ground_truth_step_num = data[task_id]["ground_truth_step_num"]
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(args.log_path, 'a') as f:
            f.write(f'The start time of this experiment: {current_time}\n')
            f.write("Env_id: " + str(env_id) + "\n")
            f.write("Task_id: " + str(task_id) + "\n")
            f.write("Task_goal: " + str(task_goal) + "\n")
            f.write("Goal_instruction: " + goal_instruction[0] + "\n")
            f.write("Ground_truth_step_num: " + str(ground_truth_step_num) + "\n")

        test_results = {}
        agent = []
        for node in data[task_id]["init_graph"]["nodes"]:
            if node["category"] == "Agents":
                agent.append([node["class_name"],node["id"]])
    
        agent_nodes = []
        for a in agent:
            agent_node = [node for node in init_graph['nodes'] if node['id'] == a[1]]
            agent_nodes += agent_node

        dict_list = []
        for i, agent_name in enumerate(agent):
            dict_item = {
                'agent_id': i,
                'args': args,
                'agent_node': agent_nodes[i],
                'init_graph': init_graph,
            }
            dict_list.append(dict_item)

        def env_fn(update_graph):
            return Get_env_info(
                                task_id = task_id,
                                env_id = env_id,
                                task_name = task_name,
                                graph = update_graph,
                                task_goal = task_goal,
                                goal_instruction = goal_instruction,
                                ground_truth_step_num = ground_truth_step_num,
                                agent = agent,
                                num_agent = len(agent)
                                )
        sim_env = env_fn(init_graph)
        real_env = env_fn(init_graph)

        def MCTS_agent_fn(env):
            return MCTSAgent(args, env, uct_type=args.uct_type, dict_list=dict_list, use_llm=args.use_llm)

        
        history = []
        done = False
        succ = 0
        total = 0
        
        done = False
        max_step = ground_truth_step_num[0] * 2 + 1
        sim_agents = MCTS_agent_fn(sim_env)
        real_agents = MCTS_agent_fn(real_env)
        for i in range(max_step):
            print(" ---------------------- Step: ", i+1, " ---------------------- ")
            with open(args.log_path, 'a') as f:
                f.write(" ---------------------- Step: " + str(i+1) + " ----------------------\n")
        


            obs = real_env.get_observations()
            valid_actions, valid_actions2agents = get_actions(obs, len(agent), real_agents, real_env, dict_list)
            with open(args.log_path, 'a') as f:
                f.write("Historical action: " + str(history) + "\n")
        
            action = sim_agents.search(obs, history, i, valid_actions, valid_actions2agents, done)
            history.append(action)
            print("Actual execution action: ", action)
            with open(args.log_path, 'a') as f:
                f.write("Actual execution action: " + action + "\n")
            # input('Press Enter to continue...')
            class_name = valid_actions2agents[action]['class_name']
            real_id = valid_actions2agents[action]['id']

            action = action.split(': ', 1)[1]
            done, task_results,satisfied, unsatisfied,steps= real_env.step(class_name, real_id, action, task_goal)

            update_graph = real_env.graph
            update_sim_env = env_fn(update_graph)
            sim_agents = MCTS_agent_fn(update_sim_env)


            if done:
                succ += 1
                with open(args.log_path, 'a') as f:
                    f.write("Task Success\n")
                    f.write('Step number: ' + str(i+1) + '\n')
                break
        if not done:
            with open(args.log_path, 'a') as f:
                f.write("Task Failed\n")

        total += 1
        sim_agents.root = None
        sim_agents.state_dict = {}
        time.sleep(5)
        print("succ rate: ", succ / total)

if __name__ == "__main__" :
    main()