
import json
with open(f'C:\\Users\\Administrator\\Desktop\\task-json-filter\\env4_full.json') as file:
        data = json.load(file)


# tasklist = [0,1,3,5,8,9,11,12,13,14,15,18] #env3
# tasklist = [0,1,2,4,8,9,12,13,14,15,18,19] #env2
# tasklist = [0,2,4,5,6,9,12,13,14,15,17,18,19] #env1
# tasklist = [0,3,5,6,7,8,12,13,14,17,18,19] #env0
# tasklist = [2,3,4,5,6,8,9,11,13,14,15,16]  #env4
# for task in data:
#     text = ''
#     if task['task_id'] in tasklist:
#         text += f'task_id: {task["task_id"]}\n'
#         edges = task['init_graph']['edges']
#         jishu = {}
#         for edge in edges:
#             if edge['from_id'] not in jishu:
#                 jishu[edge['from_id']] = 1
#             else:
#                 jishu[edge['from_id']] += 1
#             # print(jishu)
#         text += f'{jishu}\nMore than One Times from_id:'
#         for k , v in jishu.items():
#             if v > 1:
#                 text += f'{k} | '
#         with open('C:\\Users\\Administrator\\Desktop\\task-json-filter\\jianchaedge.txt', 'a') as file:  
#             file.write(text + '\n') 


dic = {}
i = 0
for task in data:
        
        
        lis = []
        for node in task['init_graph']['nodes']:
            lis.append(node['id'])
        dic[i] = lis
        i += 1
print(dic)