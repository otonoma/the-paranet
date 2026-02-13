import math
import os
import omni
import carb
import omni.kit.commands
import numpy as np

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.physx import _range_sensor
from pxr import UsdGeom
from typing import Optional

from .math_util import euler_from_quaternion
from .vfh import VFH

CLOUD_CLIP_DIST = 10

ASSET_PATH = os.environ.get(
    "ASSET_PATH",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)


# This class is used to trace the lidar data
class LidarDebugTrace:
    def __init__(self):
        self.hist = []

    def dump(self):
        for obj in self.hist:
            carb.log_info(f"POS {obj['origin']} ROT {obj['rot']}")
            carb.log_info(f"DEPTH {obj['depth']}")
            carb.log_info(f"AZIMUTH {obj['azimuth']}")
            carb.log_info(f"CLOUD {obj['cloud']}")


class CustomSpotFlatTerrainPolicy(PolicyController):
    """The Spot quadruped"""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "spot",
        color: str = None,
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize robot and load RL policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath in the directory
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot

        """
        self._stage = get_current_stage()
        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)
        self._gains_applied = False

        assets_root_path = get_assets_root_path()
        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                if not color:
                    asset_path = (
                        f"{assets_root_path}/Isaac/Robots/BostonDynamics/spot/spot.usd"
                    )
                else:
                    asset_path = os.path.join(ASSET_PATH, f"{color}_spot.usd")

                carb.log_info(f"Adding {asset_path} to stage ...")
                prim.GetReferences().AddReference(asset_path)

        self.robot = SingleArticulation(
            prim_path=self._prim_path,
            name=name,
            position=position,
            orientation=orientation,
        )

        self._dof_control_modes: List[int] = list()

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

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
        self._decimation = 10

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
                np.zeros(12) + 60, np.zeros(12) + 3.0
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

    def post_reset(self):
        self.robot.post_reset()


# This class represents a Spot robot in the simulation environment
class Spot:
    def __init__(self, root_path, drop_loc, color):
        self.name = " ".join(root_path[1:].split("/"))
        self.prefix = "_".join(root_path[1:].split("_")) + "_"
        self.root_path = root_path
        self.body_path = root_path + "/spot" if color else root_path
        self.drop_loc = drop_loc
        self.color = color

        self.cmd = np.zeros(3)
        self.iter = 0
        self.moving = False
        self.callback = None
        self.trace = LidarDebugTrace()

    # Converts a grid position to a world position for the Spot robot
    def grid_world_position(self, pos):
        return np.append(pos, 0.8)

    # Returns the approximate location of the Spot robot based on its lidar sensor
    def get_approx_loc(self):
        pos, _ = self.lidar.get_world_pose()
        return (pos[0], pos[1])

    # Sets up the scene for the Spot robot, including its controller and lidar sensor
    def setup_scene(self):
        self.controller = CustomSpotFlatTerrainPolicy(
            prim_path=self.root_path,
            name=self.name,
            position=self.grid_world_position(self.drop_loc),
            color=self.color,
        )

        self.controller.robot.set_joints_default_state(self.controller.default_pos)

        _, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="Lidar",
            parent=self.body_path + "/body",
            rotation_rate=0.0,
            horizontal_fov=180,
        )
        UsdGeom.XformCommonAPI(prim).SetTranslate((0.5, 0.0, -0.12))
        self.lidar_path = self.body_path + "/body/Lidar"
        self.lidar = SingleRigidPrim(self.lidar_path)
        self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

    def setup_post_reset(self):
        self.controller.initialize()
        self.nav = VFH(self.controller.robot, id=self.name)

    def setup_post_load(self):
        self.controller.initialize()
        self.nav = VFH(self.controller.robot, id=self.name)

    # This method is called on each physics step to update the Spot robot's state
    def on_physics_step(self, step_size, default_cmd):
        if self.moving:
            # Update the navigation state every 5 iterations
            # and update the command every 10 iterations
            if (self.iter % 5) == 0:
                # Get the world cloud from the lidar sensor
                cloud = self.get_world_cloud()
                if cloud is not None:
                    # Update the navigation with the current cloud data
                    # this will be used for path planning
                    self.nav.update_cloud(cloud)
                if (self.iter % 10) == 0:
                    # Get the next command from the navigation system
                    # this will be used to control the robot
                    self.cmd = self.nav.get_command(trace=self.trace)
                    self.trace = LidarDebugTrace()
                    if self.nav.done:
                        self.moving = False
                        self.iter = 0
                        cb = self.callback
                        if cb is not None:
                            # the callback may immediately call run_nav, so clear this beforehand
                            self.callback = None
                            cb()
            cmd = self.cmd
        else:
            cmd = default_cmd
        self.controller.advance(step_size, cmd)
        self.iter += 1

    # Returns the world cloud data from the lidar sensor
    # This data is used for navigation and obstacle avoidance
    # It filters out points that are too close to the robot
    # and checks the robot's orientation to avoid pointing too much towards the floor
    # If the robot is pointing too much towards the floor, it returns None
    def get_world_cloud(self):
        depth = self.lidarInterface.get_linear_depth_data(self.lidar_path)
        azimuth = self.lidarInterface.get_azimuth_data(self.lidar_path)
        idx = depth[:, 0] < CLOUD_CLIP_DIST
        origin, rot = self.lidar.get_world_pose()
        rot = euler_from_quaternion(rot)
        deg_rot = np.round(rot / math.pi * 180)
        penalty = abs(deg_rot[0]) + deg_rot[1]
        if penalty >= 3:
            # ignore if pointing too much towards the floor
            return None
        x = depth[idx, 0] * np.cos(azimuth[idx] + rot[2]) + origin[0]
        y = depth[idx, 0] * np.sin(azimuth[idx] + rot[2]) + origin[1]
        cloud = np.stack([x, y], axis=-1)
        self.trace.hist.append(
            {
                "depth": depth[idx, 0],
                "azimuth": azimuth[idx],
                "rot": deg_rot,
                "cloud": cloud,
                "origin": origin,
            }
        )
        return cloud

    # Runs the navigation task for the Spot robot
    # It sets the target location and starts the navigation process
    # The callback function is called when the navigation is complete
    # The midpoint parameter indicates whether to use a midpoint for navigation
    # If midpoint is True, the robot will navigate to a midpoint before reaching the target
    def run_nav(self, loc, callback, midpoint=False):
        self.callback = callback
        target = self.grid_world_position(loc)
        pos, _ = self.lidar.get_world_pose()
        carb.log_info(f"{self.name} NAV {pos[0:2]} -> {target[0:2]}")
        self.nav.clear()
        self.nav.set_target(target[0:2], midpoint=midpoint)
        self.moving = True

    # Stops the navigation task if the robot is currently moving
    # This is useful if the robot needs to stop immediately
    def stop_nav(self):
        if self.moving:
            self.nav.stop()

    # Returns the current location of the Spot robot
    # This is an approximate location based on the lidar sensor
    def get_loc(self):
        return self.get_approx_loc()

    # Returns the current direction of the Spot robot in degrees
    # The direction is calculated from the robot's world pose
    def get_direction(self):
        _, rot = self.lidar.get_world_pose()
        rot = euler_from_quaternion(rot)
        deg = math.degrees(rot[2])
        if deg < 0:
            deg = 360 + deg
        return deg

    # Converts a grid position to a world position
    def get_distance(self, loc):
        pos, _ = self.lidar.get_world_pose()
        target = self.grid_world_position(loc)
        return np.linalg.norm([pos[0:2], target[0:2]])
