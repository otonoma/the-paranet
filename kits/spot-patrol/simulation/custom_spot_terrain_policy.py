# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import io
import numpy as np
import torch
from typing import List, Optional
import os
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path

class CustomSpotFlatTerrainPolicy(PolicyController):
    """The Spot quadruped"""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "spot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        color: Optional[str] = None,
    ) -> None:
        """
        Initialize robot and load RL policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): articulation root path
            name (str) -- name of the quadruped
            usd_path (Optional[str]) -- robot usd filepath
            position (Optional[np.ndarray]) -- position of the robot
            orientation (Optional[np.ndarray]) -- orientation of the robot
            color (Optional[str]) -- color of the robot, either "blue" or "green"
        """
        self._stage = get_current_stage()
        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)
        self._gains_applied = False

        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                if color == "blue":
                    prim.GetReferences().AddReference("blue_spot.usd")
                elif color == "green":
                    prim.GetReferences().AddReference("green_spot.usd")

        self.robot = SingleArticulation(
            prim_path=self._prim_path,
            name=name,
            position=position,
            orientation=orientation,
        )

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        assets_root_path = get_assets_root_path()
        self.load_policy(
            assets_root_path + "/Isaac/Samples/Policies/Spot_Policies/spot_policy.pt",
            assets_root_path + "/Isaac/Samples/Policies/Spot_Policies/spot_env.yaml",
        )

        self._action_scale = 0.2
        self._previous_action = np.zeros(12)
        self._policy_counter = 0
        self.default_pos = np.array(
            [0.1, -0.1, 0.1, -0.1, 0.9, 0.9, 1.1, 1.1, -1.5, -1.5, -1.5, -1.5]
        )
 
    def _compute_observation(self, command):
        """
        Compute the observation vector for the policy

        Argument:
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(48)
        # Base lin vel
        obs[:3] = lin_vel_b
        # Base ang vel
        obs[3:6] = ang_vel_b
        # Gravity
        obs[6:9] = gravity_b
        # Command
        obs[9:12] = command
        # Joint states
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[12:24] = current_joint_pos - self.default_pos
        obs[24:36] = current_joint_vel
        # Previous Action
        obs[36:48] = self._previous_action

        return obs

    def advance(self, dt, command):
        """
        Compute the desired torques and apply them to the articulation

        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        """
        if not self._gains_applied:
            self.robot.get_articulation_controller().set_gains(
                np.zeros(12) + 60, np.zeros(12) + 1.5
            )
            self._gains_applied = True 

        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        action = ArticulationAction(
            joint_positions=self.default_pos + (self.action * self._action_scale)
        )
        self.robot.apply_action(action)

        self._policy_counter += 1

    def initialize(self, physics_sim_view=None):
        self.robot.initialize(physics_sim_view=physics_sim_view)
        self.robot.get_articulation_controller().set_effort_modes("force")
        self.robot.get_articulation_controller().switch_control_mode("position")

    def post_reset(self) -> None:
        self.robot.post_reset()