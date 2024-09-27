import copy
import json
from openai import OpenAIError, OpenAI
import openai
import backoff  


class LLM:
	def __init__(self, source, lm_id, args):
		self.args = args
		self.debug = args.debug
		self.source = args.source
		self.lm_id = args.lm_id
		self.chat = True
		self.total_cost = 0
		self.device = None
		self.record_dir = f'./log/{args.env}.txt'

		if self.source == 'openai':

			api_key = args.api_key  # your openai api key
			organization= args.organization # your openai organization

			client = OpenAI(api_key = api_key, organization=organization)
			if self.chat:
				self.sampling_params = {
					"max_tokens": args.max_tokens,
                    "temperature": args.t,
                    # "top_p": 1.0,
                    "n": args.n
				}

		def lm_engine(source, lm_id, device):
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

	def parse_answer(self, available_actions, text):
		
		text = text.replace("_", " ")
		text = text.replace("takeoff from", "takeoff_from")
		text = text.replace("land on", "land_on")

		for i in range(len(available_actions)):
			action = available_actions[i]
			if action in text:
				return action
		self.write_log_to_file('\nThe first action parsing failed!')
		print('\nAction parsing failed!!')
		

		for i in range(len(available_actions)):
			action = available_actions[i]
			option = chr(ord('A') + i)
			if f"option {option}" in text or f"{option}." in text.split(' ') or f"{option}," in text.split(' ') or f"Option {option}" in text or f"({option})" in text:
				return action

		print("WARNING! No available action parsed!!! Output plan NONE!\n")
		self.write_log_to_file('\nThe second action parsing failed!')
		# return random.choice(available_actions)
		return None

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
		print(agent_node["class_name"],agent_node['id'])
		print(available_plans)
		return plans, len(available_plans), available_plans
			
	def run(self, agent_node, chat_agent_info,current_room, next_rooms, all_landable_surfaces,landable_surfaces, on_surfaces, grabbed_objects, reached_objects,unreached_objecs, on_same_surface_objects,
		room_explored = None):
		info = {}
		info = {"num_available_actions": None,
			"prompts": None,
			"outputs": None,
			"plan": None,
			"action_list": None,
			"cost":self.total_cost, 
			f"<{agent_node['class_name']}>({agent_node['id']}) total_cost": None}


		available_plans, num, available_plans_list = self.get_available_plans(agent_node, next_rooms, all_landable_surfaces,landable_surfaces, on_surfaces, grabbed_objects, reached_objects,unreached_objecs, on_same_surface_objects,
																		 )
		message = None
		agent_available_actions_list = [f"<{agent_node['class_name']}>({agent_node['id']}): " + action for action in available_plans_list]
		info.update({'action_list': agent_available_actions_list})

		return message, info

	def write_log_to_file(self,log_message, file_name=None):
		file_name = self.record_dir
		with open(file_name, 'a') as file:  
			file.write(log_message + '\n')  