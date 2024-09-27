from constants import *
from tasks.defaults_Merom_1_int_Task1 import *

cfg = dict()


cfg["render"] = {
  "viewer_width": 1280,
  "viewer_height": 720
}


cfg["scene"] = {
  # "type": "Scene",
  # "floor_plane_visible": True
  
  "type": "InteractiveTraversableScene",
  "scene_model": "Merom_1_int",
  "scene_file": "/home/pjlab/liukehui/COHERENT-OmniGibson/Benchmark/tasks/Merom_1_int_best.json",
  "floor_plane_visible": True
}

cfg["agents"] = [
  {
    "type": "FrankaAgent",
    "name": "franka_0",
    "position": birthpose["franka_0"][:3],
    "rotation": birthpose["franka_0"][3:],
    "obs_modalities": ["rgb", "depth", "proprio"],
    "scale": 1.0,
    "fixed_base": True,
    "self_collision": False,
    "action_normalize": True,
    "action_type": "continuous"
  },
  {
    "type": "AliengoAgent",
    "name": "aliengo_0",
    "position": birthpose["aliengo_0"][:3],  
    "rotation": birthpose["aliengo_0"][3:],   
    "obs_modalities": ["rgb", "depth", "proprio"],
    "scale": 1.0,
    "fixed_base": True,
    "self_collision": False,
    "action_normalize": True,
    "action_type": "continuous"
  },
  {
    "type": "QuadrotorAgent",
    "name": "quadrotor_0",
    "position": [1.7745543718338013, 7, quadrotor_defaults["land_height"]], # birthpose["quadrotor_0"][:3],
    "rotation": [0.        ,  0.        , 0.70710678,  0.70710678],  # birthpose["quadrotor_0"][3:],
    "obs_modalities": ["rgb", "depth", "proprio"],
    "scale": 1.0,
    "fixed_base": True,
    "self_collision": False,
    "action_normalize": True,
    "action_type": "continuous"
  }
]
    

cfg["robots"] = []


cfg["objects"] = [
  {
    "type": "DatasetObject",
    "name": "door_duymuw_0",
    "model": "duymuw",
    "usd_path": os.path.join(ASSERT_ROOT, "door_single/door_single.usda"),
    "position": birthpose["door_duymuw_0"][:3],
    "orientation": birthpose["door_duymuw_0"][3:],  # scalar-rear
    "fixed_base": True,
    "bounding_box": [0.1298, 0.6267, 1.9769]
  },
  {
    "type": "DatasetObject",
    "name": "door_duymuw_1",
    "model": "duymuw",
    "usd_path": os.path.join(ASSERT_ROOT, "door_single/door_single.usda"),
    "position": birthpose["door_duymuw_1"][:3],
    "orientation": birthpose["door_duymuw_1"][3:],  # scalar-rear
    "fixed_base": True,
    "bounding_box": [0.1298, 0.6267, 1.9769]
  },
  {
    "type": "DatasetObject",
    "name": "apple_agveuv_0",
    "category": "apple",
    "model": "agveuv",
    "scale": scale["apple_agveuv_0"],
    "position": birthpose["apple_agveuv_0"][:3],  
    "orientation": birthpose["apple_agveuv_0"][3:],  # scalar-rear
    "fixed_base": False,
  },
  {
    "type": "DatasetObject",
    "name": "orange_nlygru_0",
    "category": "orange",
    "model": "nlygru",
    "position": birthpose["orange_cwcefi_0"][:3],
    "orientation": birthpose["orange_cwcefi_0"][3:],  # scalar-rear
    "fixed_base": False
  },
 
  {
    "type": "DatasetObject",
    "name": "wicker_basket_dgkhyn_0",
    "category": "wicker_basket",
    "model": "dgkhyn",
    "position": birthpose["wicker_basket_dgkhyn_0"][:3],
    "orientation": birthpose["wicker_basket_dgkhyn_0"][3:],  # scalar-rear
    "fixed_base": True,
    "scale": [1,1,0.2]
  }
]





cfg["task"] = {
  "type": "DummyTask"
}


