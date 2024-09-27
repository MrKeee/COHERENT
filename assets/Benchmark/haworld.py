import numpy as np
import json
import carb
from pxr import Sdf, Gf
from omni.isaac.core import World
from omni.isaac.sensor import Camera

from scenes.utils.macros import gm


from scenes.objects.light_object import LightObject
from scenes.objects.dataset_object import DatasetObject
from sensors.camera_mover import CameraMover

from agents.franka_agent import FrankaAgent
from agents.aliengo_agent import AliengoAgent
from agents.quadrotor_agent import QuadrotorAgent

DEFAULT_SCENE_CONFIG = {
    "use_floor_plane": True,
    "use_skybox": True,
    "use_viewer_camera": False,
    "use_camera_mover": False,
}

DEFAULT_CAMERA_CONFIG = {
    "camera_height": 256,
    "camera_width": 256,
    "clipping_range": [0.001, 10000000.0],
    "focal_length": 17.0,
    "default_view_camera_position": [],
    "default_view_camera_orientation": []
}


DEFAULT_FRANKA_CONFIG = {
    "position": [1.5, 7.6987, 0],

}

DEFAULT_ALIENGO_CONFIG = {
    "position": [1.5, 7.6987, 0],
    
}

DEFAULT_QUADROTOR_CONFIG = {
    "position": [1.5, 7.6987, 0],
    
}
DEFAULT_AGENT_CONFIG = {
    "franka": DEFAULT_FRANKA_CONFIG,
    "aliengo": DEFAULT_ALIENGO_CONFIG,
    "quadrotor": DEFAULT_QUADROTOR_CONFIG
}


class HAWorld:
    def __init__(
            self, 
            simulation_app,
            physics_dt, 
            rendering_dt, 
            gravity=9.81,
            scene_model="default",
            scene_file=None,
            agent_cfg={},

        ):
        self.simulation_app = simulation_app
        self.gravity = gravity
        self.scene_model = scene_model
        self.scene_file = scene_file
        self.default_agent_cfg = agent_cfg
        self._world = World(physics_dt=physics_dt, rendering_dt=rendering_dt)
        self.franka = None
        self.aliengo = None
        self.quadrotor = None

        # Toggle simulator state once so that downstream omni features can be used without bugs
        # e.g.: particle sampling, which for some reason requires sim.play() to be called at least once
        self._world.play()
        self._world.stop()




        self.initialize()

    def load_scene(self):
        if DEFAULT_SCENE_CONFIG["use_floor_plane"]:
            self._world.scene.add_default_ground_plane(
                                                        z_position=0,
                                                        name="default_ground_plane",
                                                        prim_path="/World/defaultGroundPlane",
                                                        static_friction=0.2,
                                                        dynamic_friction=0.2,
                                                        restitution=0.01,)
        if DEFAULT_SCENE_CONFIG["use_skybox"]:
            skybox = LightObject(
                prim_path="/World/skybox",
                name="skybox",
                light_type="Dome",
                intensity=1500,
                fixed_base=True,
            )
            self._world.scene.add(skybox)
            light_prim = skybox.light_link.prim
            light_prim.GetAttribute("color").Set(Gf.Vec3f(1.07, 0.85, 0.61))
            light_prim.GetAttribute("texture:file").Set(Sdf.AssetPath(m.DEFAULT_SKYBOX_TEXTURE))

        if DEFAULT_SCENE_CONFIG["use_viewer_camera"]:
            self._viewer_camera = Camera(
                prim_path="/World/viewer_camera",
                name="viewer_camera",
                frequency=20,
                resolution=(DEFAULT_CAMERA_CONFIG["camera_height"], DEFAULT_CAMERA_CONFIG["camera_width"]),
                position=DEFAULT_CAMERA_CONFIG["default_view_camera_position"],
                orientation=DEFAULT_CAMERA_CONFIG["default_view_camera_orientation"]
            )
            # We update its clipping range and focal length so we get a good FOV and so that it doesn't clip
            # nearby objects (default min is 1 m)
            self._viewer_camera.set_clipping_range(near_distance=DEFAULT_CAMERA_CONFIG["clipping_range"][0], 
                                                   far_distance=DEFAULT_CAMERA_CONFIG["clipping_range"][1])
            self._viewer_camera.set_focal_length(DEFAULT_CAMERA_CONFIG["focal_length"])

            self._world.scene.add(self._viewer_camera)

            # camera mover
            if self.default_scene_cfg["use_viewer_camera"]:
                self._camera_mover = CameraMover(self._viewer_camera)

        if self.scene_model is not None and self.scene_file is not None:
            self._load_objects_from_scene_file()
            # self._load_metadata_from_scene_file()
        
    
    def _load_objects_from_scene_file(self):
        """
        Loads scene objects based on metadata information found in the current USD stage's scene info
        (information stored in the world prim's CustomData)
        """
        # Grab objects info from the scene file
        with open(self.scene_file, "r") as f:
            scene_info = json.load(f)
        init_info = scene_info["objects_info"]["init_info"]
        init_state = scene_info["state"]["object_registry"]
        init_systems = scene_info["state"]["system_registry"].keys()

        # Create desired systems
        # for system_name in init_systems:
        #     get_system(system_name)

        # Iterate over all scene info, and instantiate object classes linked to the objects found on the stage
        # accordingly
        for obj_name, obj_info in init_info.items():
            # Create object class instance
            obj = DatasetObject(**obj_info["args"])
            # Import into the simulator
            self._world.scene.add(obj)
            # Set the init pose accordingly
            obj.set_position_orientation(
                position=init_state[obj_name]["root_link"]["pos"],
                orientation=init_state[obj_name]["root_link"]["ori"],
            )

    def _load_metadata_from_scene_file(self):
        """
        Loads metadata from self.scene_file and stores it within the world prim's CustomData
        """
        with open(self.scene_file, "r") as f:
            scene_info = json.load(f)

        # Write the metadata
        # for key, data in scene_info.get("metadata", dict()).items():
        #     og.sim.write_metadata(key=key, data=data)



    def _should_load_object(self, obj_info):
        """
        Helper function to check whether we should load an object given its init_info. Useful for potentially filtering
        objects based on, e.g., their category, size, etc.

        Subclasses can implement additional logic. By default, this returns True

        Args:
            obj_info (dict): Dictionary of object kwargs that will be used to load the object

        Returns:
            bool: Whether this object should be loaded or not
        """
        return True



    def load_agents(self):
        for load_agent_name, cfg in self.default_agent_cfg.items():
            if load_agent_name == "franka":
                self.franka = FrankaAgent(self._world, **cfg)
            elif load_agent_name == "aliengo":
                self.aliengo = AliengoAgent(self._world, **cfg)
            elif load_agent_name == "quadrotor":
                self.quadrotor = QuadrotorAgent(self._world, **cfg)


    def reset(self):
        pass

    def step(self, render=True):

        self._world.step(render=render) # execute one physics step and one rendering step

    def initialize(self):

        # Update internal settings
        # Update the physics settings
        # This needs to be done now, after an initial step + stop for some reason if we want to use GPU
        # dynamics, otherwise we get very strange behavior, e.g., PhysX complains about invalid transforms
        # and crashes
        self._set_physics_engine_settings()
        self._set_renderer_settings()


        # load
        self.load_scene()
        self.load_agents()


        self._world.reset()
        self.simulation_app.update()




    def _set_physics_engine_settings(self):
        """
        Set the physics engine with specified settings
        """
        assert self._world.is_stopped(), f"Cannot set simulator physics settings while simulation is playing!"
        self._world._physics_context.set_gravity(value=-self.gravity)
        # Also make sure we invert the collision group filter settings so that different collision groups cannot
        # collide with each other, and modify settings for speed optimization
        self._world._physics_context.set_invert_collision_group_filter(True)
        self._world._physics_context.enable_ccd(gm.ENABLE_CCD)
        self._world._physics_context.enable_flatcache(gm.ENABLE_FLATCACHE)

        # Enable GPU dynamics based on whether we need omni particles feature
        if gm.USE_GPU_DYNAMICS:
            self._world._physics_context.enable_gpu_dynamics(True)
            self._world._physics_context.set_broadphase_type("GPU")
        else:
            self._world._physics_context.enable_gpu_dynamics(False)
            self._world._physics_context.set_broadphase_type("MBP")

        # Set GPU Pairs capacity and other GPU settings
        self._world._physics_context.set_gpu_found_lost_pairs_capacity(gm.GPU_PAIRS_CAPACITY)
        self._world._physics_context.set_gpu_found_lost_aggregate_pairs_capacity(gm.GPU_AGGR_PAIRS_CAPACITY)
        self._world._physics_context.set_gpu_total_aggregate_pairs_capacity(gm.GPU_AGGR_PAIRS_CAPACITY)
        self._world._physics_context.set_gpu_max_particle_contacts(gm.GPU_MAX_PARTICLE_CONTACTS)

    def _set_renderer_settings(self):
        # TODO: For now we are setting these to some reasonable high-performance values but these can be made configurable.
        carb.settings.get_settings().set_bool("/rtx/reflections/enabled", False)  # Can be True with a 10fps penalty
        carb.settings.get_settings().set_bool("/rtx/indirectDiffuse/enabled", True)  # Can be False with a 5fps gain
        carb.settings.get_settings().set_bool("/rtx/directLighting/sampledLighting/enabled", True)
        carb.settings.get_settings().set_int("/rtx/raytracing/showLights", 1)
        carb.settings.get_settings().set_float("/rtx/sceneDb/ambientLightIntensity", 0.1)
        # TODO: Think of better setting defaults. Below works well for indoor-only scenes, but if skybox is the only light source then this looks very bad
        # carb.settings.get_settings().set_int("/rtx/domeLight/upperLowerStrategy", 3)  # "Limited image-based"
