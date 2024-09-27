import json
import copy
import re

class Get_env_info:
    def __init__(self, task_id = None, env_id = None, task_name = None, graph = None, task_goal = None, goal_instruction = None, ground_truth_step_num = None, agent = None, num_agent = None) -> None:
        
        self.task_id = task_id
        self.graph = graph
        self.task_goal = task_goal
        self.goal_instruction = goal_instruction[0]
        self.ground_truth_step_num = ground_truth_step_num[0]
        self.agent = agent
        self.num_agent = num_agent
        self.env_id = env_id
        self.task_name = task_name
        self.id2node = {node['id']: node for node in self.graph['nodes']}
        self.id_to_name = {node['id']: node['class_name'] for node in self.graph['nodes']}
        # self.message_said = [None for _ in range(num_agent)]
        self.steps = 0

        self.id_name_dict = {}
        for i, agent_name in enumerate(agent):
            self.id_name_dict[i] = agent_name
    @property
    def all_containers_name(self) -> list:
        r'''
        get all containers in the scene.
        '''

        containers_name = [[{node["id"]:node["class_name"]}] for node in self.graph['nodes'] if 'CONTAINERS' in node['properties']]
        return containers_name

    @property
    def all_goal_objects_name(self) -> list:
        r'''
		 get all objects that related to goal.
		'''  

        goal_objects = []
        for predicate in self.task_goal.keys():
            elements = predicate.split('_')
            # print(elements)
            for x in elements[1:]:
                if x.isdigit():
                    goal_objects += [{self.id2node[int(x)]['id']: self.id2node[int(x)]['class_name']}]
                elif '(' in x:
                    y = x.split('(')[1].split(')')[0]
                    if y.isdigit():
                        goal_objects += [{self.id2node[int(y)]['id']: self.id2node[int(y)]['class_name']}]
                else:
                    goal_objects += [{node["id"]: node["class_name"]} for node in self.graph['nodes'] if node['class_name'] == x]
                # print(goal_objects)

        return goal_objects
    
    @property
    def all_room_name(self) -> list:
        
        r'''
		 get all rooms in the scene.
		'''
        room_name = []
        room_name = [{node["id"]: node["class_name"]} for node in self.graph['nodes'] if node['category'] == 'Rooms']

        return room_name
    
    @property
    def all_room_and_character_id(self) -> list:
        r'''
        get all room_and_agent_ids in the scene.
        '''
        return [node['id'] for node in self.graph['nodes'] if
                            node['category'] == 'Agents' or node['category'] in ['Rooms']]
    
    def get_observations(self):
        r'''
            get observations in the scene.
        '''
        dict_observations = {}
        for agent_id in self.id_name_dict.keys():
         
            dict_observations[agent_id] = self.get_observation(agent_id)
            # self.location[agent_id].append(dict_observations[agent_id]['location'])
        return dict_observations

    def get_observation(self, agent_id):
        # agent_location = self.get_agent_location(agent_id)
        obs, agent_in_room = self.get_visible_node(agent_id)
        return {**obs,'agent_in_room_id':agent_in_room}

    def get_agent_location(self, agent_id):
        graph = self.graph
        agent_node =  [node for node in graph['nodes'] if node['id'] == self.id_name_dict[agent_id][1]]
        return agent_node[0]['obj_transform']['position']
    
    def get_visible_node(self, agent_id):
        r'''
            The rule for obtaining obs in this code is that the agent can observe objects in the same room as it, but objects hidden in the closed container cannot be seen.
        '''
        id2node = {node['id']: node for node in self.graph['nodes']}
        rooms_ids = [node['id'] for node in self.graph['nodes'] if node['category'] == 'Rooms']

        agent_node = [node for node in self.graph['nodes'] if node['id'] == self.id_name_dict[agent_id][1]]

        # find agent
        real_agent_id = agent_node[0]["id"]
        inside_of_what, what_is_inside, edge_from = {}, {}, {}
        #the id of object grabbed by robotic dog or robotic arm
        grabbed_ids = []
        #the id of quadrotor's basket
        with_quadrotor_id = []
        # the id of object inside the quadrotor's basket
        inside_quadrotor_basket_id = []
        for edge in self.graph['edges']:
            if edge['relation_type'] == 'INSIDE':
            
                if edge['to_id'] not in what_is_inside.keys():
                    what_is_inside[edge['to_id']] = []
                what_is_inside[edge['to_id']].append(edge['from_id'])
                # key is container
                inside_of_what[edge['from_id']] = edge['to_id']
                # key is object in the container
            elif edge['relation_type'] == 'HOLD':
                if edge['from_id'] == real_agent_id:
                    grabbed_ids.append(edge['to_id'])
            elif edge["relation_type"] == "WITH":
                    with_quadrotor_id.append(edge["to_id"]) #The basket can only be seen if it is in the same room as the current agent
                    quadrotor_id = edge["from_id"]
                    for ee in self.graph['edges']:
                        if ee['relation_type'] == 'INSIDE' and ee["from_id"] == quadrotor_id:
                            quadrotor_in_room_id = ee["to_id"]

        for edge in self.graph['edges']:
            if edge['relation_type'] == 'INSIDE' and edge["to_id"] in with_quadrotor_id:
                inside_quadrotor_basket_id.append(edge['from_id'])

        room_id = inside_of_what[real_agent_id]
        agents_in_same_room =[] # An agent can observe other agents in the same room
        for edge in self.graph['edges']:
            if id2node[edge['from_id']]['category'] == "Agents" and edge["to_id"] == room_id and edge['relation_type'] == 'INSIDE':
                agents_in_same_room.append(edge['from_id'])

        all_room_ids = []
        for node in self.graph['nodes']:
            if node['category'] == 'Rooms':
                all_room_ids.append(node['id'])
        

        # Note! Some object are not directly in room, but we want to add them. An object may be on surface, and if the surface is inside the room, then the object is also inside the room
        new_inside_room = True
        temp_edges = copy.deepcopy(self.graph["edges"])
        while new_inside_room :
            new_inside_room = False
            for edge in temp_edges:
                if edge['relation_type'] == 'ON' and edge["to_id"] in what_is_inside[room_id] or edge['relation_type'] == 'INSIDE' and edge["to_id"] in what_is_inside[room_id]:
                    what_is_inside[room_id].append(edge["from_id"])
                    what_is_inside[room_id] = list(set(what_is_inside[room_id]))
                    temp_edges.remove(edge)
                    new_inside_room = True
                    break
                    #This is equivalent to iterating to get the objects in the room, because many objects in edge are not displayed in the room, but on a surface or cotainer in the room. Except for individual objects, our definition of edge basically requires from_to to have only one position relation
                    #This method is used because the order of traversal is uncertain, and sometimes to_id has not been traversed yet and has not been added to the what_is_inside list
        doors_ids = []
        for edge in self.graph['edges']:
            if edge['relation_type'] == 'LEADING TO':
                    if edge['to_id'] == room_id:
                        doors_ids.append(edge['from_id'])
                   
        
        object_in_room_ids = what_is_inside[room_id] #Find the object in the same room as the agent

        curr_objects = list(object_in_room_ids)
        while len(curr_objects) > 0:
            objects_inside = []
            for curr_obj_id in curr_objects:
                new_inside = what_is_inside[curr_obj_id] if curr_obj_id in what_is_inside.keys() else []
                objects_inside += new_inside
            
            object_in_room_ids += list(objects_inside)
            curr_objects = list(objects_inside) # curr_objects represent the objects in the container in the room

        # Only objects that are inside the room and not inside something closed
        object_hidden = []
        for key in inside_of_what.keys():
            if inside_of_what[key] not in rooms_ids and 'OPEN' not in id2node[inside_of_what[key]]['states'] and 'OPEN_FOREVER' not in id2node[inside_of_what[key]]['states']:
                object_hidden.append(key)
        
        observable_object_ids = [object_id for object_id in object_in_room_ids if object_id not in object_hidden] + [room_id] + doors_ids + [real_agent_id] #The observation of the agent should include itself
        observable_object_ids += grabbed_ids
        observable_object_ids += agents_in_same_room
        observable_object_ids += all_room_ids
        if room_id == quadrotor_in_room_id:
            observable_object_ids += with_quadrotor_id
            observable_object_ids += inside_quadrotor_basket_id #If it is in the same room as the quadrotor, then the agent can see the quadrotor and the basket on the quadrotor
        observable_object_ids = list(set(observable_object_ids))

        partilly_observable_state = {
                "edges": [edge for edge in self.graph['edges'] if (edge['from_id'] in observable_object_ids and edge['to_id'] in observable_object_ids)],
                "nodes": [id2node[id_node] for id_node in observable_object_ids]
            }

        return partilly_observable_state, room_id
    
    def step(self, class_name, id, agent_action, goal):

        if self.steps > 30:
            print("Warning: too many steps")
        action = self.get_action_name(agent_action)[0]
        first_object_name = self.get_object_name(agent_action)[0]
        first_id = int(self.get_object_id_str(agent_action)[0])
        if action in ['putinto', 'puton']:
            second_object_name = self.get_object_name(agent_action)[1]
            second_id = int(self.get_object_id_str(agent_action)[1])

        room2floor = {}
        for node in self.graph['nodes']:
            if node['category'] == "Rooms":
                for edge in self.graph['edges']:
                    if edge['relation_type'] == 'INSIDE' and self.id2node[edge['from_id']]['category'] == 'Floor' and edge['to_id'] == node['id']:
                        room2floor[node['id']] = edge['from_id']

        if class_name == 'quadrotor':

            for edge in self.graph['edges']:
                if edge['from_id'] == id and edge['relation_type'] == 'WITH':
                     with_quadrotor = edge['to_id']

            if action == 'land_on':
                self.id2node[id]['states'] = ['LAND']
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['to_id'] == first_id and edge['relation_type'] == 'ABOVE':
                        edge['relation_type'] = 'ON'
                new_edge = {
                "from_id": with_quadrotor,
                "to_id": first_id,
                "relation_type": "ON"
                }
                if 'HIGH_HEIGHT' in self.id2node[first_id]['properties']:
                    self.id2node[with_quadrotor]['properties'].append("ON_HIGH_SURFACE")
                    self.id2node[with_quadrotor]['properties'] = list(set(self.id2node[with_quadrotor]['properties']))
                    for edge in self.graph['edges']:
                        if edge['to_id'] == with_quadrotor and edge['relation_type'] == 'INSIDE':
                            self.id2node[edge['from_id']]['properties'].append("ON_HIGH_SURFACE")
                            self.id2node[edge['from_id']]['properties'] = list(set(self.id2node[edge['from_id']]['properties']))
                            # Use the ON_HIGH_SURFACE property to restrict the robot dog from touching objects on the high table
                if 'LOW_HEIGHT' in self.id2node[first_id]['properties'] and 'ON_HIGH_SURFACE' in self.id2node[with_quadrotor]['properties']:
                    self.id2node[with_quadrotor]['properties'].remove("ON_HIGH_SURFACE")
                    for edge in self.graph['edges']:
                        if edge['to_id'] == with_quadrotor and edge['relation_type'] == 'INSIDE':
                            self.id2node[edge['from_id']]['properties'].remove("ON_HIGH_SURFACE")
                            # You can delete the ON_HIGH_SURFACE property only when you land on a low surface

                self.graph['edges'].append(new_edge) #After the quadrotor lands, the position of the basket is located on the surface
                
            if action == 'takeoff_from':
                self.id2node[id]['states'] = ['FLYING']
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['to_id'] == first_id and edge['relation_type'] == 'ON':
                        edge['relation_type'] = 'ABOVE'
                self.graph['edges'].remove({
                "from_id": with_quadrotor,  #with_quadrotor is the id of the basket
                "to_id": first_id,
                "relation_type": "ON"
                })
                #After the quadrotor takes off, the position that the basket is located on the surface is deleted
                self.graph['edges'] = [edge for edge in self.graph['edges'] if not (edge['to_id'] == with_quadrotor and edge['relation_type'] == 'CLOSE')] #The CLOSE position is used to describe the robotic dog is close to an object
                self.id2node[with_quadrotor]['properties'].append("ON_HIGH_SURFACE")
                self.id2node[with_quadrotor]['properties'] = list(set(self.id2node[with_quadrotor]['properties']))
                for edge in self.graph['edges']:
                    if edge['to_id'] == with_quadrotor and edge['relation_type'] == 'INSIDE':
                        self.graph['edges'] = [e for e in self.graph['edges'] if not (e['to_id'] == edge['from_id'] and e['relation_type'] == 'CLOSE')]
                        self.id2node[edge['from_id']]['properties'].append("ON_HIGH_SURFACE")
                        self.id2node[edge['from_id']]['properties'] = list(set(self.id2node[edge['from_id']]['properties']))
                            # Quadrotor flying into the sky also uses the ON_HIGH_SURFACE property to limit the robotic dog from touching objects with the quadrotor

            if action == 'movetowards':
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['relation_type'] == 'ABOVE':
                        if self.id2node[first_id]['category'] != 'Rooms':
                            edge['to_id'] = first_id
                        else:
                            edge['to_id'] = room2floor[first_id]
                if self.id2node[first_id]['category'] == 'Rooms':
                    for edge in self.graph['edges']:
                        if edge['from_id'] == id and edge['relation_type'] == 'INSIDE':
                            edge['to_id'] = first_id      

        if class_name == 'robot dog' or class_name == 'robot_dog':
            if action == 'open':
                if first_object_name == "door":
                    self.id2node[first_id]['states'] = ['OPEN']
                if 'CONTAINERS' in self.id2node[first_id]['properties']:
                    self.id2node[first_id]['states'] = ['OPEN']

            if action == 'close':
                if first_object_name == "door":
                    self.id2node[first_id]['states'] = ['CLOSED']
                if 'CONTAINERS' in self.id2node[first_id]['properties']:
                    self.id2node[first_id]['states'] = ['CLOSED']

            if action == 'grab':
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['to_id'] == first_id and edge['relation_type'] == 'CLOSE':
                        edge['relation_type'] = 'HOLD'
                for edge in self.graph['edges']:
                    if edge['from_id'] == first_id and (edge['relation_type'] == 'INSIDE' or edge['relation_type'] == 'ON'):
                        self.graph['edges'].remove(edge)
                        break

            if action == 'putinto':
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['to_id'] == first_id and edge['relation_type'] == 'HOLD':
                        # self.graph['edges'].remove(edge)
                        edge['relation_type'] = 'CLOSE'
                        break
                new_edge = {
                    "from_id": first_id,
                    "to_id": second_id,
                    "relation_type": "INSIDE"
                }
                self.graph['edges'].append(new_edge)

            if action == 'puton':
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['to_id'] == first_id and edge['relation_type'] == 'HOLD':
                        # self.graph['edges'].remove(edge)
                        edge['relation_type'] = 'CLOSE'
                        break
                new_edge = {
                    "from_id": first_id,
                    "to_id": second_id,
                    "relation_type": "ON"
                }
                self.graph['edges'].append(new_edge)


            if action == 'movetowards':

                if self.id2node[first_id]['category'] == 'Rooms':
                    for edge in self.graph['edges']:
                        if edge['from_id'] == id and edge['relation_type'] == 'INSIDE':
                            edge['to_id'] = first_id  
                        if edge['from_id'] == id and edge['relation_type'] == 'ON':
                            edge['to_id'] = room2floor[first_id]

                    self.graph['edges'] = [edge for edge in self.graph['edges'] if not (edge['from_id'] == id and edge['relation_type'] == 'CLOSE')]
                    # It means that if the agent walks into the room, there is no CLOSE relationship between the agent and other objects

                else:
                    self.graph['edges'] = [edge for edge in self.graph['edges'] if not (edge['from_id'] == id and edge['relation_type'] == 'CLOSE')]

                    new_edge ={
                        "from_id": id,
                        "to_id": first_id,
                        "relation_type": "CLOSE"
                    }
                    self.graph['edges'].append(new_edge)
                    #########################################################
                    if "CONTAINERS" in self.id2node[first_id]['properties'] and ('OPEN' in self.id2node[first_id]['states'] or 'OPEN_FOREVER' in self.id2node[first_id]['states']):
                        new_edges = []
                        for edge in self.graph['edges']:
                            if edge['to_id'] == first_id and edge['relation_type'] == 'INSIDE':
                                new_edge = {
                                    "from_id": id,
                                    "to_id": edge['from_id'],
                                    "relation_type": "CLOSE"
                                }
                                new_edges.append(new_edge)   #When moving towards the container, the agent and the objects in the container are also CLOSE
                        self.graph['edges'].extend(new_edges)
                    #########################################################
                    new_edges = []
                    for edge in self.graph['edges']:      
                        if edge['from_id'] == first_id and (edge['relation_type'] == 'ON' or edge['relation_type'] == 'INSIDE') and self.id2node[edge['to_id']]['category'] != 'Floor':
                            new_edge = {
                                    "from_id": id,
                                    "to_id": edge['to_id'],
                                    "relation_type": "CLOSE"
                                }
                            new_edges.append(new_edge) #Close to an object, but also close to the surface or container on which the object is located, except the floor
                    self.graph['edges'].extend(new_edges)
                    #########################################################
                    if "SURFACES" in self.id2node[first_id]['properties'] and self.id2node[first_id]['category'] != 'Floor':
                        new_edges = []
                        for edge in self.graph['edges']:
                            if edge['to_id'] == first_id and edge['relation_type'] == 'ON':
                                new_edge = {
                                    "from_id": id,
                                    "to_id": edge['from_id'],
                                    "relation_type": "CLOSE"
                                }
                                new_edges.append(new_edge)   #Towards the surface, the agent and the object on the surface are also CLOSE
                        self.graph['edges'].extend(new_edges)   
                    #########################################################

        if class_name == 'robot arm' or class_name == 'robot_arm':
            if action == 'open':
                if 'CONTAINERS' in self.id2node[first_id]['properties']:
                    self.id2node[first_id]['states'] = ['OPEN']

            if action == 'close':
                if 'CONTAINERS' in self.id2node[first_id]['properties']:
                    self.id2node[first_id]['states'] = ['CLOSED']

            if action == 'grab':
                for edge in self.graph['edges']:
                    if edge['from_id'] == first_id and (edge['relation_type'] == 'INSIDE' or edge['relation_type'] == 'ON'):
                        self.graph['edges'].remove(edge)
                        break
                new_edge = {
                    "from_id": id,
                    "to_id": first_id,
                    "relation_type": "HOLD"
                }
                self.graph['edges'].append(new_edge)

            if action == 'putinto':
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['to_id'] == first_id and edge['relation_type'] == 'HOLD':
                        self.graph['edges'].remove(edge)
                        break
                new_edge = {
                    "from_id": first_id,
                    "to_id": second_id,
                    "relation_type": "INSIDE"
                }
                self.id2node[first_id]['properties'].append("ON_HIGH_SURFACE")
                self.id2node[first_id]['properties'] = list(set(self.id2node[first_id]['properties']))
                self.graph['edges'].append(new_edge)

            if action == 'puton':
                for edge in self.graph['edges']:
                    if edge['from_id'] == id and edge['to_id'] == first_id and edge['relation_type'] == 'HOLD':
                        self.graph['edges'].remove(edge)
                        break
                new_edge = {
                    "from_id": first_id,
                    "to_id": second_id,
                    "relation_type": "ON"
                }
                self.id2node[first_id]['properties'].append("ON_HIGH_SURFACE")
                self.id2node[first_id]['properties'] = list(set(self.id2node[first_id]['properties']))
                self.graph['edges'].append(new_edge)


        for node in self.graph['nodes']:
            for key, value in self.id2node.items():
                if node['id'] == key:
                    node.update({"properties": value["properties"] , "states": value["states"]})
        # Update the properties and states of self.id2node to self.graph['nodes']

        # print(goal)
        unsatisfied = copy.deepcopy(goal)
        satisfied = []
        task_results = []
        for goal_descs, nums in goal.items():
            
            
            
            goal_desc = goal_descs.split('_')
            from_id = int((re.findall(r'\((.*?)\)', goal_desc[1]))[0])
            to_id = int((re.findall(r'\((.*?)\)', goal_desc[2]))[0])
            num = nums[0]
            goal_num = 0
            for edge in self.graph['edges']:
                if edge['from_id'] == from_id and edge['to_id'] == to_id and edge['relation_type'] == goal_desc[0].upper():
                    goal_num += 1
                    satisfied.append(edge)
            task_result = {
                goal_descs: goal_num
            }
            task_results.append(task_result)
            if num == goal_num:
                del unsatisfied[goal_descs]
                if len(unsatisfied) == 0:
                    done = True
                    break
            else:

                done = False
                break

        self.steps += 1
        steps = self.steps

        
        return done, task_results, satisfied, unsatisfied,steps

    def get_action_name(self, input_string):
        matches = re.findall(r'\[(.*?)\]', input_string)
        return matches

    def get_object_id_str(self, input_string):
        matches = re.findall(r'\((.*?)\)', input_string)
        return matches
    
    def get_object_name(self, input_string):
        matches = re.findall(r'<(.*?)>', input_string)
        return matches
