import os 
from scipy.spatial.transform import Rotation as R


BENCHMARK_ROOT = os.path.dirname(__file__)
ASSERT_ROOT = os.path.join(BENCHMARK_ROOT, 'assets')
TASK_ROOT = os.path.join(BENCHMARK_ROOT, 'tasks')
WORLDMODEL_ROOT = os.path.join(BENCHMARK_ROOT, 'worldmodels')


franka_defaults = {
    "topdowngrasp": R.from_euler('XYZ', [-180, 0, 180], degrees=True).as_quat(),
    "initEEpose2base": [0.4603976,  0.00552514, 0.38718461, -0.02601395,  0.92235384, -0.00624321,  0.38541884]
}


aliengo_defaults = {
    "height": 0.4,
    "topdowngrasp": R.from_euler('Y', 90, degrees=True).as_quat(),
    "frontreargrasp": [0, 0, 0, 1],
    "leftrightgrasp": R.from_euler('X', 90, degrees=True).as_quat(),
    "initEEpose2base": [0.0382,  0, 0.1605, 0, 0, 0, 1]
}

quadrotor_defaults = {
    "land_height": 0.05,
    "hover_height": 1.5,
    "basket_height": 0.06
}


# from multiprocessing import Manager

# current state of env

class WORLDMODEL:
    state = dict()

# WORLDSTATE = dict()

def get_world_state():
    return WORLDMODEL.state

def set_world_state(state):
    WORLDMODEL.state = state