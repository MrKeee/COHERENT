import json
import os
from constants import WORLDMODEL_ROOT


def set_state(state, task_name):
    with open(os.path.join(WORLDMODEL_ROOT, '%s.json' % task_name), 'w') as file:
        json.dump(state, file)

def get_state(task_name):
    with open(os.path.join(WORLDMODEL_ROOT, '%s.json' % task_name), 'r') as file:
        return json.load(file)
    
    