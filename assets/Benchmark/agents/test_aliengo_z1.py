from isaacgym import gymutil
from isaacgym import gymapi
from isaacgym import gymtorch
from isaacgym.terrain_utils import *
from isaacgym.torch_utils import *
from legged_gym import LEGGED_GYM_ROOT_DIR
import numpy as np
import torch
import cv2
import time
import math

def set_camera(viewer, position, lookat):
    """ Set camera position and direction
    """
    cam_pos = gymapi.Vec3(position[0], position[1], position[2])
    cam_target = gymapi.Vec3(lookat[0], lookat[1], lookat[2])
    gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

damping = 0.2
def control_ik(dpose):
    global damping, j_eef, num_envs
    # solve damped least squares
    j_eef_T = torch.transpose(j_eef, 1, 2)
    lmbda = torch.eye(6, device=device) * (damping ** 2)
    u = (j_eef_T @ torch.inverse(j_eef @ j_eef_T + lmbda) @ dpose).view(num_envs, 7)
    return u

def orientation_error(desired, current):
    cc = quat_conjugate(current)
    q_r = quat_mul(desired, cc)
    return q_r[:, 0:3] * torch.sign(q_r[:, 3]).unsqueeze(-1)

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(description="test")

# configure sim
sim_params = gymapi.SimParams() # 初始化参数
sim_params.up_axis = gymapi.UP_AXIS_Z # 设置Z轴竖直向上
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81) # 设置重力
if args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.substeps = 1
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu
elif args.physics_engine == gymapi.SIM_FLEX and not args.use_gpu_pipeline:
    sim_params.flex.shape_collision_margin = 0.25
    sim_params.flex.num_outer_iterations = 4
    sim_params.flex.num_inner_iterations = 10
else:
    raise Exception("GPU pipeline is only available with PhysX")

# sim_params.use_gpu_pipeline = args.use_gpu_pipeline
sim_params.use_gpu_pipeline = True
device = args.sim_device if args.use_gpu_pipeline else 'cpu'

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    raise Exception("Failed to create sim")

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")

asset_root = f"{LEGGED_GYM_ROOT_DIR}/resources/robots/"
asset_file = "aliengo/urdf/new-aliengo-z1.urdf"
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.collapse_fixed_joints = True
asset_options.replace_cylinder_with_capsule = True
asset_options.flip_visual_attachments = True

asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# set up dof
asset_dof_props = gym.get_asset_dof_properties(asset)
asset_dof_props['driveMode'] = gymapi.DOF_MODE_POS
asset_dof_props['stiffness'] = 200
asset_dof_props['damping'] = 10
asset_num_dofs = gym.get_asset_dof_count(asset)

# default_dof_pos = torch.tensor([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1, -1.5, -0.1, 1, -1.5]) # only aliengo
# default_dof_pos = torch.tensor([0.0, 0.5, -0.5, 0.0,  0.1,  0.1])  # only z1
default_dof_pos = torch.tensor([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1, -1.5, -0.1, 1, -1.5, 0.0, 0.5, -0.5, 0.0,  0.1,0.1,0.0])  # aliengo+z1

aliengo_link_dict = gym.get_asset_rigid_body_dict(asset)
gripper_index = aliengo_link_dict['gripperMover']

# default pose
pose = gymapi.Transform()
pose.p.z = 1.0

# set up the env grid
num_envs = 1
num_per_row = int(np.sqrt(num_envs))
env_spacing = 4.0
env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

# set random seed
np.random.seed(42)

envs = []
handles = []
gripper_idxs = []
for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, 2)
    envs.append(env)
    aliengo_handle = gym.create_actor(env, asset, pose, "aliengo", i, 1)
    handles.append(aliengo_handle)

    gym.set_actor_dof_properties(env, aliengo_handle, asset_dof_props)
    gym.set_actor_dof_states(env, aliengo_handle, default_dof_pos, gymapi.STATE_ALL)
    gripper_handle = gym.find_actor_rigid_body_handle(env, aliengo_handle,"gripperMover")
    gripper_pose = gym.get_rigid_transform(env, gripper_handle)
    gripper_idx = gym.find_actor_rigid_body_index(env, aliengo_handle,"gripperMover", gymapi.DOMAIN_SIM)
    gripper_idxs.append(gripper_idx)

gym.prepare_sim(sim)
num_dofs = gym.get_sim_dof_count(sim)

# add ground plane
plane_params = gymapi.PlaneParams() # 初始化地面参数
plane_params.normal = gymapi.Vec3(0, 0, 1) # 地面法向量沿z轴
gym.add_ground(sim, plane_params) 

pos = gymtorch.wrap_tensor(gym.acquire_actor_root_state_tensor(sim))[0,:3]
set_camera(viewer, pos + torch.tensor([1,1,2],device=device), pos)


_jacobian = gym.acquire_jacobian_tensor(sim, "aliengo")
jacobian = gymtorch.wrap_tensor(_jacobian)
j_eef = jacobian[:,gripper_index-1,:,12:19]  # [num_envs,6,7]
_massmatrix = gym.acquire_mass_matrix_tensor(sim, "aliengo")
mm = gymtorch.wrap_tensor(_massmatrix)[:,12:19,12:19]

_rb_states = gym.acquire_rigid_body_state_tensor(sim)
rb_states = gymtorch.wrap_tensor(_rb_states)

_dof_states = gym.acquire_dof_state_tensor(sim)
dof_states = gymtorch.wrap_tensor(_dof_states)
dof_pos = dof_states[:, 0].view(num_envs, 19, 1)
dof_vel = dof_states[:, 1].view(num_envs, 19, 1)
pos_action = torch.zeros_like(dof_pos).squeeze(-1)
effort_action = torch.zeros_like(pos_action)

# visualize
axes_geom = gymutil.AxesGeometry(0.1)
sphere_rot = gymapi.Quat.from_euler_zyx(0.5 * math.pi, 0, 0)
sphere_pose = gymapi.Transform(r=sphere_rot)
sphere_geom = gymutil.WireframeSphereGeometry(0.03, 12, 12, sphere_pose, color=(1, 0, 0))
goal = gymapi.Transform()
while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # refresh tensors
    gym.refresh_rigid_body_state_tensor(sim)
    gym.refresh_dof_state_tensor(sim)
    gym.refresh_jacobian_tensors(sim)
    gym.refresh_mass_matrix_tensors(sim)
    gripper_pos = rb_states[gripper_idxs, :3]
    gripper_rot = rb_states[gripper_idxs, 3:7]

    # compute goal position and orientation
    goal_pos = torch.Tensor([[0.2,0.3,1.5]]).to(device)
    goal_rot = torch.Tensor([[1,0,0,0]]).to(device)

    # compute position and orientation error
    pos_err = goal_pos - gripper_pos
    orn_err = orientation_error(goal_rot, gripper_rot)
    dpose = torch.cat([pos_err, orn_err], -1).unsqueeze(-1)

    # draw 
    goal.p.x = goal_pos[0][0]
    goal.p.y = goal_pos[0][1]
    goal.p.z = goal_pos[0][2]
    gymutil.draw_lines(axes_geom, gym, viewer, envs[0], goal)
    gymutil.draw_lines(sphere_geom, gym, viewer, envs[0], goal)

    pos_action[:, 12:19] = dof_pos.squeeze(-1)[:, 12:19] + control_ik(dpose)
    pos_action[:, 18] = 0
    gym.set_dof_position_target_tensor(sim, gymtorch.unwrap_tensor(pos_action))

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)

            