import json

with open(f'C:\\Users\\Administrator\\Desktop\\task-json-filter\\env4_full.json') as file:
        data = json.load(file)
# tasklist = [0,3,5,6,7,8,12,13,14,17,18,19] #env0
# fix = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,22,23,24,25,27,28,32,33,34,35,50] #env0
# fix_edge = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,24,27,28,32,33,34,35,50] #env0
# tasklist = [0,2,4,5,6,9,12,13,14,15,17,18,19] #env1
# fix = [0,1,2,3,4,5,6,7,8,9,10,11,20,21,22,23,24,34,35,36,37,38,39,44,45,47]  #env1
# fix_edge = [0,1,2,3,4,5,6,7,8,9,10,11,20,24,34,35,36,37,38,39,44,45,47]   #env1
# tasklist = [0,1,2,4,8,9,12,13,14,15,18,19] #env2
# fix = [0,1,2,3,4,5,6,7,8,18,19,20,21,22,23,24,26,29,30,38,41,42] #env2
# fix_edge = [0,1,2,3,4,5,6,7,8,18,19,20,23,24,26,29,30,38,41,42] #env2
# tasklist = [0,1,3,5,8,9,11,12,13,14,15,18] #env3
# fix = [0,1,2,3,4,5,6,7,8,9,10,11,18,19,20,21] #env3
# fix_edge = [0,1,2,3,4,5,6,7,8,9,10,11,18,19,20] #env3
tasklist = [2,3,4,5,6,8,9,11,13,14,15,16] # env4
fix = [0,1,2,3,4,5,6,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,30,31,32,33] #env4
fix_edge = [1,3,6,7,8,9,10,27,17,12,13,14,15,16,18,19,30,21,22,26,28,31,32,33] #env4
for task in data:
    text = ''
    if task['task_id'] in tasklist:

        text = f'task_id: {task["task_id"]}\n'
        text += str(task["task_goal"])
        text += '\n'
        text += str(task["goal_instruction"])
        text += '\n'
        text += str(task["ground_truth_step_num"])
        text += '\n'
        observation = task['init_graph']
        
        id2node = {node['id']: node for node in observation["nodes"]}
        
        text += '*******************************Agent********************************************\n'
        for node in observation["nodes"] :
            if node["category"] == "Agents" and node['id'] not in [] :
                # agent_node = node
                text += "I am <" + node["class_name"] +">(" + str(node["id"]) + "). "   
                if len(node['properties']) != 0:
                    properties = ', '.join(node['properties'])
                    text += "Now my properties are: " + properties + ". "
                if len(node['states']) != 0:
                    states = ', '.join(node['states'])
                    text += "Now my state is: " + states + ". "
                # if len(node['properties']) != 0 or len(node['states']) != 0:
                text += '\n'
                for edge in observation["edges"]:
                    if edge["from_id"] == node["id"]:
                        text += "I am " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). "
                    if edge["relation_type"] == "WITH":
                        with_quadrotor_id = edge["to_id"]
                if node["class_name"] == "quadrotor":
                    for edge in observation["edges"]:
                        if edge["from_id"] == with_quadrotor_id:
                            text += "\nThe <" + id2node[edge["from_id"]]["class_name"] +">(" + str(edge["from_id"]) + ") is with quadrotor LAND " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + ")."
                            if len(id2node[edge["from_id"]]['properties']) != 0:
                                properties = ', '.join(id2node[edge["from_id"]]['properties'])
                                text += "Its properties are: " + properties + ". "
                            if len(id2node[edge["from_id"]]['states']) !=0 :
                                states = ', '.join(id2node[edge["from_id"]]['states'])
                                text += "Now its state is: " + states + "."
                text += '\n'
        text += '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Nodes~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n'        
        for node in observation["nodes"]:
            if node["category"] != "Agents" and node["category"] != "Rooms" and node['id'] not in fix:
                
                text += "<" + node["class_name"] +">(" + str(node["id"]) + "). "
                if len(node['properties']) != 0:
                    properties = ', '.join(node['properties'])
                    text += "Its properties are: " + properties + ". "
                if len(node['states']) !=0 :
                    states = ', '.join(node['states'])
                    text += "Now its state is: " + states + "."
                text += '\n'
            # if node["category"] != "Agents" and node["category"] != "Rooms":
            #     for edge in observation["edges"]:
            #         if edge["from_id"] == node["id"]: 
            #             text += "The <" + node["class_name"] +">(" + str(node["id"]) + ") is " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"

                

        text += '----------------------------------Edges------------------------------------------------\n'
        for node in observation["nodes"]:
            if node["category"] != "Agents" and node["category"] != "Rooms" and node['id'] not in fix_edge:
                for edge in observation["edges"]:
                    if edge["from_id"] == node["id"]: 
                        text += "The <" + node["class_name"] +">(" + str(node["id"]) + ") is " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"

    
    with open('C:\\Users\\Administrator\\Desktop\\task-json-filter\\check_env4.txt', 'a') as file:  
        file.write(text + '\n')  

#打印fix的object的node和edge
# tasklist = [0]
# for task in data:
#     text = ''
#     if task['task_id'] in tasklist:
#         observation = task['init_graph']
        
#         id2node = {node['id']: node for node in observation["nodes"]}

#         text += '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Fix~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n'        
#         for node in observation["nodes"]:
#             if node['id'] in fix:
                
#                 text += "<" + node["class_name"] +">(" + str(node["id"]) + "). "
#                 if len(node['properties']) != 0:
#                     properties = ', '.join(node['properties'])
#                     text += "Its properties are: " + properties + ". "
#                 if len(node['states']) !=0 :
#                     states = ', '.join(node['states'])
#                     text += "Now its state is: " + states + "."
#                 text += '\n'

#         for node in observation["nodes"]:
#             if  node['id'] in fix_edge:
#                 for edge in observation["edges"]:
#                     if edge["from_id"] == node["id"]: 
#                         text += "The <" + node["class_name"] +">(" + str(node["id"]) + ") is " + edge["relation_type"] + " the <" + id2node[edge["to_id"]]["class_name"] + ">(" + str(edge["to_id"]) + "). \n"

#     with open('C:\\Users\\Administrator\\Desktop\\task-json-filter\\check_env4.txt', 'a') as file:  
#         file.write(text + '\n')  