import random
import copy
import json
from openai import OpenAIError,OpenAI
import openai
import backoff
import re


# @ray.remote
class ArenaMP(object):
    def __init__(self, environment_fn, agent_fn, args , run_predefined_actions=False):
        # run_predefined_actions is a parameter that you can use predefined_actions.json to strictly set the agents' actions instead of using algorithm to calculate the action.
        

        self.env_fn = environment_fn
        self.agents = agent_fn
        self.args = args
        self.num_agents = len(agent_fn)
        self.task_goal = None
        self.record_dir = f'./log/{args.env}.txt'
        self.debug = args.debug
        print("Init Env")
        self.env = environment_fn()

        self.run_predefined_actions = run_predefined_actions

        self.oracle_prompt_path = args.oracle_prompt_path


        self.dialogue_history = ""
        self.total_dialogue_history = []
        self.chat = True
        self.source = args.source
        self.lm_id = args.lm_id
        # self.lm_id = 'gpt-3.5-turbo-1106'
        self.device = None
        self.sampling_parameters = None
        self.total_cost = 0

        self.last_done = False
        self.last_task_results = None
        self.last_satisfied = None
        self.last_unsatisfied = None
        self.costdict = {}
        self.action_history_list = []
        self.action_history = ''
        

        if self.source == 'openai':    
            api_key = args.api_key # your openai api key
            organization=args.organization # your openai organization

            client = OpenAI(api_key = api_key, organization=organization)
            if self.chat:
                self.sampling_params = {
                    "max_tokens": args.max_tokens,
                    "temperature": args.t,
                    # "top_p": args.top_p,
                    "n": args.n
                }

        def lm_engine( source, lm_id, device):

            @backoff.on_exception(backoff.expo, OpenAIError)
            def _generate(prompt, sampling_params):
                usage = 0
                if source == 'openai':
                    try:
                        if self.chat:
                            prompt.insert(0,{"role":"system", "content":"You are a helper assistant."})
                            response = client.chat.completions.create(
                                model=lm_id, messages=prompt, **sampling_params
                            )
                            if self.debug:
                                with open(f"./chat_raw.json", 'a') as f:
                                    f.write(json.dumps(response, indent=4))
                                    f.write('\n')
                            generated_samples = [response.choices[i].message.content for i in
                                                    range(sampling_params['n'])]
                            if 'gpt-4-0125-preview' in self.lm_id:
                                usage = response.usage.prompt_tokens * 0.01 / 1000 + response.usage.completion_tokens * 0.03 / 1000
                            elif 'gpt-3.5-turbo-1106' in self.lm_id:
                                usage = response.usage.prompt_tokens * 0.0015 / 1000 + response.usage.completion_tokens * 0.002 / 1000
                        # mean_log_probs = [np.mean(response['choices'][i]['logprobs']['token_logprobs']) for i in
                        # 				  range(sampling_params['n'])]
                        else:
                            raise ValueError(f"{lm_id} not available!")
                    except OpenAIError as e:
                        print(e)
                        raise e
                else:
                    raise ValueError("invalid source")
                return generated_samples, usage 
            return _generate

        self.generator = lm_engine(self.source, self.lm_id, self.device)


    def get_actions(self, obs, chat_agent_info):

        for id, agent in enumerate(self.agents):
            if agent.agent_node["id"] == chat_agent_info["id"]:

                action, message, info = agent.get_action(obs[id], chat_agent_info, self.env.task_goal)

        return action, message, info

    def agent_obs2text(self, observation, agent_id):
        text = ""
        observation = observation[agent_id]
       
        id2node = {node['id']: node for node in observation["nodes"]}
        agent_class = id2node[int(self.env.id_name_dict[agent_id][1])]["class_name"]
        with_quadrotor_id = None
        for node in observation["nodes"]:
            if node["category"] == "Agents" and self.env.id_name_dict[agent_id][1] == node["id"]:
                # agent_node = node
                text += "I am <" + node["class_name"] +">(" + str(node["id"]) + "). "   
                if len(node['states']) != 0:
                    states = ', '.join(node['states'])
                    text += "Now my state is: " + states + ". "
                for edge in observation["edges"]:
                    if edge["from_id"] == node["id"]:
                        text += "I am " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). "
                    if edge["relation_type"] == "WITH":
                        with_quadrotor_id = edge["to_id"]
                text += '\n'


        for node in observation["nodes"]:
            if node["category"] == "Rooms" and node["id"] == observation["agent_in_room_id"]:
                text += "Now I am in the <"+ node["class_name"] +">(" + str(node["id"]) + "). In this room, I can see : \n"
        for node in observation["nodes"]:
            if node["id"] != self.env.id_name_dict[agent_id][1] and node["category"] != "Rooms":
                text += "<" + node["class_name"] +">(" + str(node["id"]) + "). "
                if len(node['properties']) != 0:
                    properties = ', '.join(node['properties'])
                    text += "Its properties are: " + properties + ". "
                if len(node['states']) !=0 :
                    states = ', '.join(node['states'])
                    text += "Now its state is: " + states + ". \n"
                else:
                    text += '\n'
        text += "These objects have a certain position relationship with each other: \n"
        for node in observation["nodes"]:
            if node["id"] != self.env.id_name_dict[agent_id][1] and node["category"] != "Rooms":
                for edge in observation["edges"]:
                    if edge["from_id"] == node["id"]: 
                    # if edge["from_id"] == node["id"] and edge["relation_type"] != "WITH" : #WITH is exclusive to quadrotor and basket
                        if edge["from_id"] == with_quadrotor_id and agent_class == 'quadrotor':
                            text += "The <" + node["class_name"] +">(" + str(node["id"]) + ") is with me LAND " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"
                        elif edge["relation_type"] == "LEADING TO":
                            text += "The <" + node["class_name"] +">(" + str(node["id"]) + ") is " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"
                        else:
                            text += "The <" + node["class_name"] +">(" + str(node["id"]) + ") is " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"
        for edge in observation["edges"]:
            if edge["relation_type"] == "WITH" and agent_class == 'quadrotor':
                in_basket = False
                text += "I have a <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + ") with me. " 
                for edges in observation["edges"]:
                    if edges["to_id"] == edge["to_id"] and edges["relation_type"] == "INSIDE" :
                        text += "<" + id2node[edges["from_id"]]["class_name"] + ">("+ str(edges["from_id"]) + ") is in my <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"
                        in_basket = True    
                if in_basket == False:
                    text += "But nothing is in my <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"
            if edge["relation_type"] == "HOLD" and agent_class != 'quadrotor':
                text += "I am holding a <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + ") in my hand. \n"    
        # print(text)
        return text
    
    def write_log_to_file(self,log_message, file_name=None):
        file_name = self.record_dir
        with open(file_name, 'a') as file:  
            file.write(log_message + '\n')  

    def step(self):
        if self.env.steps == 0:
            pass

        obs = self.env.get_observations()
        id_name_dict = self.env.id_name_dict

        obs2text = ''
        for i in range(self.num_agents):
            obs2text += self.agent_obs2text(obs, i) + '\n'



        ids  = [key for key, value in id_name_dict.items()]
        random.shuffle(ids)


        total_actionlist = []
        total_actionlist_str = ''
        dialogue_record = []
        dialogue_record_str = ''


        for id in ids:
            
            agent_obs = self.agent_obs2text(obs, id)
            task_goal = self.env.goal_instruction


            chat_agent_info = {"class_name": id_name_dict[id][0], 
                            "id": id_name_dict[id][1], 
                            "observation": agent_obs, 
                            "instruction": task_goal, 
                           
                          
                            "action_history": self.action_history,
                            "total_actionlist": total_actionlist
                            }

            agent_action, agent_message, agent_info = self.get_actions(obs, chat_agent_info)

            total_actionlist += agent_info["LLM"]["action_list"]


        total_actionlist_str  = '\n'.join(total_actionlist)
        print(total_actionlist_str)

       
        oracle_prompt = self.oracle_prompt_path 
        with open(oracle_prompt, 'r') as f:
            oracle_prompt = f.read()
        oracle_prompt = oracle_prompt.replace('#AGENT_OBSERVATIONS#', obs2text)
        oracle_prompt = oracle_prompt.replace('#TASK_GOAL#', task_goal)
        oracle_prompt = oracle_prompt.replace('#NUMBER_AGENTS#', str(self.env.num_agent))
        oracle_prompt = oracle_prompt.replace('#ACTION_LIST#', total_actionlist_str)  
        oracle_prompt = oracle_prompt.replace('#ACTION_HISTORY#', self.action_history)

        chat_prompt = [{"role": "user", "content": oracle_prompt}]
        outputs, usage = self.generator(chat_prompt, self.sampling_params)
        self.total_cost += usage
        message = outputs[0]

        self.write_log_to_file(f"@@@@@@@@@@@@@@@@@@@@@@@ Task_ID: {self.env.task_id} @@@@@@@@@@@")
        self.write_log_to_file(f"$$$$$$$$$$$$$$$$$$$$$$$ Step:{self.env.steps} $$$$$$$$$$$$$$$$$$$$$$$")
        self.write_log_to_file(f'''*******************************************************************************************
                               TASK_GOAL: {task_goal}
                               ''')
        self.write_log_to_file("OBSERVATIONS: \n" + obs2text)
        self.write_log_to_file(f'ACTIONLIST: {total_actionlist}')
        self.write_log_to_file(f'MESSAGE: {message}')



        extract_prompt = "\nExtract from the above paragraph the content of the format '<agent>(id): [action] <class_name>(id)' such as '<robot dog>(23): [movetowards] <door>(9)'. Then output the contents of this section. Be careful not to output any superfluous content, exactly in the format given. If there are more than one action in this format, you only extract the best action to be done first in the next step."
        agent_prompt = message + extract_prompt
        chat_prompt = [{"role": "user", "content": agent_prompt}]
        outputs, usage = self.generator(chat_prompt, self.sampling_params)
        self.total_cost += usage
        agent_action = outputs[0]
        self.write_log_to_file(f'AGENT_ACTION: {agent_action}')
        self.write_log_to_file(f'COST: {self.total_cost}')


        plan = self.parse_answer(total_actionlist, agent_action)
       
        self.write_log_to_file(f'parse plan:{plan}')

 
        if plan is None:

            wrong_action_history = f'Since the action ${agent_action}$ you give is not in the list of available actions for all agents currently, no action is executed this round. This means that in the current state, the steps you gave to perform the task are problematic. Please think step by step.'
            self.action_history_list.append(wrong_action_history) 
            numbered_list = [f"[{i+1}]、{item}" for i, item in enumerate(self.action_history_list)]
            self.action_history = '\n'.join(numbered_list[-5:])

            done = self.last_done
            task_results = self.last_task_results
            satisfied = self.last_satisfied
            unsatisfied = self.last_unsatisfied
            self.env.steps += 1
        else:
            try:
                self.action_history_list.append(f"{message}.") 
                numbered_list = [f"[{i+1}]、{item}" for i, item in enumerate(self.action_history_list)]
                self.action_history = '\n'.join(numbered_list[-5:])

                action_splitted_list = plan.split(':')
                agent_class = (re.findall(r'<(.*?)>', action_splitted_list[0]))[0]
                agent_id = int((re.findall(r'\((.*?)\)', action_splitted_list[0]))[0])

                done, task_results,satisfied, unsatisfied,steps= self.env.step(agent_class, agent_id, action_splitted_list[1], self.task_goal)
                self.last_done = done
                self.last_task_results = task_results
                self.last_satisfied = satisfied
                self.last_unsatisfied = unsatisfied
                
            except Exception as e:
                print("Exception occurs when performing action: ", plan)
                self.write_log_to_file(f"Exception occurs when performing action: {plan}")
                raise Exception
        steps = self.env.steps
        self.write_log_to_file(f'ACTION_HISTORY:\n{self.action_history}')    
        self.write_log_to_file('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ ')
        return done, task_results, satisfied, unsatisfied, id, plan, agent_message,steps


    def run(self, random_goal=False, pred_goal=None, cnt_subgoal_info = False):
    
        self.task_goal = copy.deepcopy(self.env.task_goal)

        if pred_goal is not None:
            self.task_goal = copy.deepcopy(pred_goal)  

        saved_info = {'task_id': self.env.task_id,
                      'env_id': self.env.env_id,
                      'task_name': self.env.task_name,
                      'goals': self.task_goal,
                      'action': {i: [] for i in range(self.num_agents)},
                      'subgoals': {i: [] for i in range(self.num_agents)},
                      'finished': None,
                      'init_unity_graph': self.env.graph,
                      'goals_finished': [],
                      'obs': {i: [] for i in range(self.num_agents)},
                      'LLM': {i: [] for i in range(self.num_agents)},
                    }

        success = False
        while True:
            done, task_results, satisfied, unsatisfied, id, agent_action, agent_message,steps  = self.step()

            success = done

            agent_id = id

            saved_info['action'][agent_id].append(agent_action)

            max_setp = 2 * self.env.ground_truth_step_num
            if self.env.steps > max_setp:
                print("---------------------------")
                print("The task failed, exceeding 2 times the number of GT steps")
                print(f"Whether steps in gt*2+1 are successful:{done}")
                print(f" setps: {steps}")
                print("---------------------------")
                self.write_log_to_file(f'''---------------------------
                                       The task failed, exceeding 2 times the number of GT steps
                                       Whether steps in gt*2+1 are successful:{done}
                                       setps: {steps}
                                       ---------------------------
                                       ''')

                success = False
                break


            if success:
                self.write_log_to_file(f'''-------------------------------------
                                        success!
                                        setps: {steps}
                                        --------------------------------
                                        ''')

                break
        saved_info['finished'] = success
     
        return success, self.env.steps, saved_info

    def update_dict(self,key, value, my_dict):
        if key in my_dict:
            # If key is in dictionary, add new value to existing value
            my_dict[key] = value
        else:
            # If key is not in dictionary, add key and value to the dictionary
            my_dict[key] = value
        return my_dict
    


    def parse_answer(self, available_actions, text):

        text = text.replace("_", " ")
        text = text.replace("takeoff from", "takeoff_from")
        text = text.replace("land on", "land_on")

        for i in range(len(available_actions)):
            action = available_actions[i]
            if action in text:
                return action
        self.write_log_to_file('\nFirst action parsing failed!!!')
        print('\nAction parsing failed!!')
        print('!!!')


        for i in range(len(available_actions)):
            action = available_actions[i]
            option = chr(ord('A') + i)

            if f"option {option}" in text or f"{option}." in text.split(' ') or f"{option}," in text.split(' ') or f"Option {option}" in text or f"({option})" in text:
                return action

        print("WARNING! No available action parsed!!! Output plan NONE!\n")

        self.write_log_to_file('\nSecond action parsing failed!!!')

        return None