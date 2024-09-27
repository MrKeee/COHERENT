# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.motion_generation import ArticulationKinematicsSolver, interface_config_loader, LulaKinematicsSolver
from omni.isaac.core.articulations import Articulation


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for Franka robot.  This class loads a LulaKinematicsSovler object

    Args:
        robot_articulation (Articulation): An initialized Articulation object representing this Franka
        end_effector_frame_name (Optional[str]): The name of the Franka end effector.  If None, an end effector link will
            be automatically selected.  Defaults to None.
    """

    def __init__(
            self, 
            robot_articulation: Articulation, 
            end_effector_frame_name: str,
            robot_description_path: str,
            urdf_path: str
        ) -> None:
        self.end_effector_frame_name = end_effector_frame_name
        kinematics_config = dict()
        kinematics_config["robot_description_path"] = robot_description_path
        kinematics_config["urdf_path"] = urdf_path
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)

        return
