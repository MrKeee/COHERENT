import numpy as np
# import spacy
# import sentencepiece as spm
from operator import itemgetter
import re


# w2v_model = spacy.load('en_core_web_lg')


def softmax_value(Qs):
    # Qs: list of Q values

    # Softmax weighted sum
    # weight = softmax(Qs, T=1)
    # value = np.dot(np.array(Qs), weight)

    # Log Sum Exp
    value = np.log(np.sum(np.exp(Qs)) / len(Qs))

    return value


# def vectorize(act):
#     v = 0
#     for word in act.split(' '):
#         v += (w2v_model(word).vector / w2v_model(word).vector_norm)
#     v = v / np.sqrt(np.sum(np.square(v)))
#     return v


def find_top_k(act_vec, acts, embedding):
    # act_vec: (N,) / acts: list of actions / embedding: (V, N)
    similarity = embedding @ act_vec
    top_k_idx = (-similarity).argsort()[:10]
    from operator import itemgetter
    top_k_acts = itemgetter(*top_k_idx)(acts)
    top_k_sim = similarity[top_k_idx]
    return top_k_acts, top_k_sim


def find_near_acts(act_vec, acts, embedding, threshold=0.7):
    # act_vec: (N,) / acts: list of actions / embedding: (V, N)
    similarity = embedding @ act_vec
    near_idx = np.where(similarity > threshold)[0].tolist()
    if len(near_idx) == 0:
        near_acts = []
    else:
        near_acts = itemgetter(*near_idx)(acts)
        near_acts = [near_acts] if type(near_acts) is float else list(near_acts)
    return near_acts, near_idx

def find_near_actions(act_vec, acts, embedding, threshold=0.7):
    # act_vec: (N,) / acts: list of actions / embedding: (V, N)
    if act_vec.shape[0] == 0 or embedding.shape[0] == 0 or len(acts) == 0:
        return [], []

    similarity = embedding @ act_vec
    near_idx = np.where(similarity > threshold)[0].tolist()
    if len(near_idx) == 0:
        near_acts = []
    else:
        near_acts = itemgetter(*near_idx)(acts)
        near_acts = [near_acts] if type(near_acts) is str else list(near_acts)
    return near_acts, near_idx



def softmax(a, T=1):
    a = np.array(a) / T
    exp_a = np.exp(a)
    sum_exp_a = np.sum(exp_a)
    y = exp_a / sum_exp_a
    return y


def padding(l, maxlen):
    if len(l) > maxlen:
        l = l[:maxlen]
    else:
        while len(l) < maxlen:
            l.append(0)
    return l


def parse_string(s):
    s = s.strip()
    s = s.replace('\n', ' ')
    s = s.split(' ')
    s = list(filter(('').__ne__, s))
    s = ' '.join(s)
    return s


def get_action_list_valid(acts, step_i, agent_id=0):
    if len(acts)>0:
        actions_parsed = [parse_language_from_action_script(tem) for tem in acts]
        history_actions = []
        for act in actions_parsed:
            if 'movetowards' in act[0]:
                history_actions.append(f'{act[0]} {act[1]}({act[2]})')
            elif act[3] is not None:
                if act[0] == 'putinto':
                    history_actions.append(f'put {act[1]}({act[2]}) into {act[3]}({act[4]})')
                elif act[0] == 'putback':
                    history_actions.append(f'put {act[1]}({act[2]}) on {act[3]}({act[4]})')
            else:
                history_actions.append(f'{act[0]} {act[1]}({act[2]})')
    return history_actions

def parse_language_from_action_script(action_script):
    act_name = re.findall(r"\[([\w\s]+)\]", action_script)[0]

    obj_name = re.findall(r"\<([\w\s]+)\>", action_script)[0]
    obj_id = re.findall(r"\(([\w\s]+)\)", action_script)[0]
    obj_id = int(obj_id)

    if '[putinto]' in action_script or '[puton]' in action_script:
        obj_name2 = re.findall(r"\<([\w\s]+)\>", action_script)[1]
        obj_id2 = re.findall(r"\(([\w\s]+)\)", action_script)[1]
        obj_id2 = int(obj_id2)
    else:
        obj_name2 = None
        obj_id2 = None

    return act_name, obj_name, obj_id, obj_name2, obj_id2

def agent_obs2text(observation, agent_id, id_name_dict):
    text = ""
    observation = observation[agent_id]
    
    id2node = {node['id']: node for node in observation["nodes"]}
    agent_class = id2node[int(id_name_dict[agent_id][1])]["class_name"]
    with_quadrotor_id = None
    for node in observation["nodes"]:
        if node["category"] == "Agents" and id_name_dict[agent_id][1] == node["id"]:
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
        if node["id"] != id_name_dict[agent_id][1] and node["category"] != "Rooms":
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
        if node["id"] != id_name_dict[agent_id][1] and node["category"] != "Rooms":
            for edge in observation["edges"]:
                if edge["from_id"] == node["id"]: 
                # if edge["from_id"] == node["id"] and edge["relation_type"] != "WITH" 
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