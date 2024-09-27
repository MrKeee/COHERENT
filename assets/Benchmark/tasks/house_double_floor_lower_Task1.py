from constants import *
from tasks.defaults_house_double_floor_lower_Task1 import *

cfg = dict()


cfg["render"] = {
  "viewer_width": 1280,
  "viewer_height": 720
}


cfg["scene"] = {
  # "type": "Scene",
  # "floor_plane_visible": True
  
  "type": "InteractiveTraversableScene",
  "scene_model": "grocery_store_cafe",
  "scene_file": "/home/pjlab/liukehui/COHERENT-OmniGibson/Benchmark/tasks/house_double_floor_lower_best_Task1.json",
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
    "position": birthpose["quadrotor_0"][:3],
    "rotation": birthpose["quadrotor_0"][3:],
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
    "name": "kebab_cewhbv_0",
    "category": "kebab",
    "model": "ezgwob",
    "scale": 0.75,
    "position": [4.28915, -0.02794, 0.63714],
    "orientation": [0.32674, 0.79499, 0.08707, 0.50364],
    "fixed_base": False
  },
]





cfg["task"] = {
  "type": "DummyTask"
}


