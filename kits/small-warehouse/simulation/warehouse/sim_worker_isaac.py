import os
import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation

import carb
from pxr import Gf, Sdf

import omni.client
import omni.anim.graph.core as ag
from omni.anim.people import PeopleSettings
from omni.isaac.core.utils import prims
from omni.usd import get_stage_next_free_path
from omni.isaac.nucleus import get_assets_root_path
from typing import Callable
from dataclasses import dataclass, field

@dataclass
class SimWorkerState:
    """Holds the current pose of a person (position and orientation)."""
    position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))

class SimWorker:
    """Represents a simulated person with animation and physics callbacks."""

    setting_dict = carb.settings.get_settings()
    people_asset_folder = setting_dict.get(PeopleSettings.CHARACTER_ASSETS_PATH)
    character_root_prim_path = setting_dict.get(PeopleSettings.CHARACTER_PRIM_PATH)
    assets_root_path = None

    if not character_root_prim_path:
        character_root_prim_path = "/World/Characters"

    if people_asset_folder:
        assets_root_path = people_asset_folder
    else:
        root_path = get_assets_root_path()
        if root_path is not None:
            assets_root_path = f"{root_path}/Isaac/People/Characters"

    @staticmethod
    def _transverse_prim(stage, stage_prefix):
        """Recursively search under `stage_prefix` for a SkelRoot prim."""
        prim = stage.GetPrimAtPath(stage_prefix)
        if prim.GetTypeName() == "SkelRoot":
            return prim, stage_prefix

        children = prim.GetAllChildren()
        if not children:
            return None, None
        
        for child in children:
            child_path = stage_prefix + "/" + child.GetName()
            prim_child, child_stage_prefix = SimWorker._transverse_prim(stage, child_path)
            if prim_child is not None:
                return prim_child, child_stage_prefix
            
        return None, None
    
    def __init__(
        self,
        world,
        stage_prefix: str,
        character_name: str = None,
        work_position=[0.0, 0.0, 0.0],
        work_yaw=0.0,
    ) -> None:
        """Initialize the person in the world, spawn the USD asset, and set up callbacks."""
        self._world = world
        self._stage = self._world.stage

        self._state = SimWorkerState()
        self._state.position = np.array(work_position)
        self._state.orientation = Rotation.from_euler('z', work_yaw, degrees=False).as_quat()

        self._initial_state = SimWorkerState()
        self._initial_state.position = np.array(work_position)
        self._initial_state.orientation = Rotation.from_euler('z', work_yaw, degrees=False).as_quat()

        self._target_position = np.array(work_position)
        self._target_speed = 0.0
        self._target_yaw = float(work_yaw)

        self._stage_prefix = get_stage_next_free_path(
            self._stage,
            SimWorker.character_root_prim_path + '/' + stage_prefix,
            False
        )

        self._character_name = character_name
        self.character_usd_path = SimWorker.get_character_usd_path(character_name)

        # spawn and rig
        self.spawn_character(self.character_usd_path, self._stage_prefix, work_position, work_yaw)
        self.add_animation_graph()

        # Initialise goal queue here:
        self._goals: list[tuple[int, np.ndarray, float, float]] = []
        self._goal_statuses: dict[int, bool] = {}
        self._callbacks: dict[int, Callable] = {}
        self._next_goal_id: int = 0

        # register callbacks after setup _goals
        self._sim_running = False
        self._world.add_physics_callback(self._stage_prefix + "/state", self.update_state)
        self._world.add_physics_callback(self._stage_prefix + "/update", self.update)
        self._world.add_timeline_callback(self._stage_prefix + "/start_stop_sim", self.on_simulation_toggle)

    @property
    def state(self) -> SimWorkerState:
        """Return the current SimWorkerState: (position and orientation)."""
        return self._state

    def on_simulation_toggle(self, event) -> None:
        """Track whether the simulation is running or stopped."""
        if self._world.is_playing() and not self._sim_running:
            self._sim_running = True
        if self._world.is_stopped() and self._sim_running:
            self._sim_running = False

    def setup_post_reset(self):
        """Reset internal state after simulation reset."""
        # clear pending goals
        self._goals.clear()
        self._goal_statuses.clear()
        self._callbacks.clear()
        self._next_goal_id = 0
        # reset animation to idle
        if self.character_graph:
            self.character_graph.set_variable("Walk", 0.0)
            self.character_graph.set_variable("Action", "Idle")

    def setup_post_load(self):
        """Reapply animation graph after scene load."""
        self.add_animation_graph()

    def update(self, dt: float) -> None:
        """Update animation variables each physics step to walk or idle, processing queued goals."""
        if not self.character_graph and self.skel_root_prim:
            self.character_graph = ag.get_character(self.skel_root_path)

        if not self._goals:
            # no pending goals: idle
            if self.character_graph:
                self.character_graph.set_variable("Walk", 0.0)
                self.character_graph.set_variable("Action", "Idle")
            return

        # process current goal
        goal_id, pos, yaw, speed = self._goals[0]
        distance = np.linalg.norm(pos - self._state.position)
        REACHED_GOAL_THRE = 0.3
        if distance > REACHED_GOAL_THRE and self.character_graph:
            self.character_graph.set_variable("Action", "Walk")
            self.character_graph.set_variable(
                "PathPoints",
                [carb.Float3(self._state.position), carb.Float3(pos)]
            )
            self.character_graph.set_variable("Walk", speed)
        else:
            # orient to final yaw
            if isinstance(self.character_prim.GetAttribute("xformOp:orient").Get(), Gf.Quatf):
                quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw).GetQuat())
            else:
                quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw).GetQuat()
            self.character_prim.GetAttribute("xformOp:orient").Set(quat)

            # mark goal success
            self._goal_statuses[goal_id] = True
            # invoke callback
            cb = self._callbacks.pop(goal_id, None)
            if cb:
                cb()
            # pop and start next if exists
            self._goals.pop(0)
            if self._goals:
                _, next_pos, next_yaw, next_speed = self._goals[0]
                self.update_target_pose(next_pos.tolist(), next_yaw, next_speed)

    def add_goal(self, position, yaw, walk_speed=1.0) -> int:
        """Append a new goal to visit; returns a goal_id for tracking."""
        gid = self._next_goal_id
        self._next_goal_id += 1
        self._goals.append((gid, np.array(position), yaw, walk_speed))
        self._goal_statuses[gid] = False
        # if this is the only goal, immediately start moving
        if len(self._goals) == 1:
            self.update_target_pose(position, yaw, walk_speed)
        return gid

    def is_goal_successful(self, goal_id: int) -> bool:
        """Return True once the specific goal has been reached."""
        return self._goal_statuses.get(goal_id, False)

    def update_target_pose(self, position, yaw, walk_speed=1.0) -> None:
        """Set a new target pose and walking speed for the character.

        Args:
            position (list): [x, y, z] coordinates to move toward.
            yaw (float): Target orientation about the Z axis in radians.
            walk_speed (float): Walking speed value.
        """
        self._target_position = np.array(position)
        self._target_yaw = float(yaw)
        self._target_speed = walk_speed

        # Immediately orient the character towards the desired yaw so that the
        # animation system starts from the correct pose.
        if isinstance(self.character_prim.GetAttribute("xformOp:orient").Get(), Gf.Quatf):
            quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), self._target_yaw).GetQuat())
        else:
            quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), self._target_yaw).GetQuat()
        self.character_prim.GetAttribute("xformOp:orient").Set(quat)

    def update_state(self, dt: float) -> None:
        """Retrieve the current transform from the animation graph and update SimWorkerState:."""
        if not self.character_graph and self.skel_root_prim:
            self.character_graph = ag.get_character(self.skel_root_path)

        if self.character_graph:
            pos = carb.Float3(0, 0, 0)
            rot = carb.Float4(0, 0, 0, 0)
            self.character_graph.get_world_transform(pos, rot)

            self._state.position = np.array([pos[0], pos[1], pos[2]])
            self._state.orientation = np.array([rot.x, rot.y, rot.z, rot.w])

    def spawn_character(self, usd_file, stage_name, work_position, work_yaw) -> None:
        """Spawn the character USD under the specified stage path with initial pose."""
        if not self._stage.GetPrimAtPath(SimWorker.character_root_prim_path):
            prims.create_prim(SimWorker.character_root_prim_path, "Xform")

        if not self._stage.GetPrimAtPath(SimWorker.character_root_prim_path + "/Biped_Setup"):
            setup_prim = prims.create_prim(
                SimWorker.character_root_prim_path + "/Biped_Setup",
                "Xform",
                usd_path=SimWorker.assets_root_path + "/Biped_Setup.usd"
            )
            setup_prim.GetAttribute("visibility").Set("invisible")

        self.character_prim = prims.create_prim(stage_name, "Xform", usd_path=usd_file)
        self.character_prim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(float(work_position[0]), float(work_position[1]), float(work_position[2]))
        )
        if isinstance(self.character_prim.GetAttribute("xformOp:orient").Get(), Gf.Quatf):
            self.character_prim.GetAttribute("xformOp:orient").Set(
                Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(work_yaw)).GetQuat())
            )
        else:
            self.character_prim.GetAttribute("xformOp:orient").Set(
                Gf.Rotation(Gf.Vec3d(0, 0, 1), float(work_yaw)).GetQuat()
            )

        prim, path = SimWorker._transverse_prim(self._stage, stage_name)
        self.skel_root_path = path
        self.skel_root_prim = prim

    def add_animation_graph(self) -> None:
        """Apply the biped animation graph to the spawned character skel root."""
        if not self.skel_root_prim:
            return

        animation_graph = self._stage.GetPrimAtPath(
            SimWorker.character_root_prim_path + "/Biped_Setup/CharacterAnimation/AnimationGraph"
        )

        omni.kit.commands.execute(
            "RemoveAnimationGraphAPICommand",
            paths=[Sdf.Path(self.skel_root_prim.GetPrimPath())]
        )
        omni.kit.commands.execute(
            "ApplyAnimationGraphAPICommand",
            paths=[Sdf.Path(self.skel_root_prim.GetPrimPath())],
            animation_graph_path=Sdf.Path(animation_graph.GetPrimPath())
        )
        self.character_graph = ag.get_character(self.skel_root_path)

    @staticmethod
    def get_character_usd_path(agent_name: str) -> str | None:
        """Return the full USD file path for a given character folder under assets_root_path."""
        agent_folder = f"{SimWorker.assets_root_path}/{agent_name}"
        result, properties = omni.client.stat(agent_folder)
        if result != omni.client.Result.OK:
            carb.log_error("Character folder does not exist.")
            return None
        character_usd = SimWorker.find_usd_file(agent_folder)
        return f"{agent_folder}/{character_usd}" if character_usd else None

    @staticmethod
    def find_usd_file(character_folder_path: str) -> str | None:
        """Find and return the first .usd file name in the specified folder."""
        result, folder_list = omni.client.list(character_folder_path)
        if result != omni.client.Result.OK:
            carb.log_error(f"Unable to read character folder path at {character_folder_path}")
            return None
        for item in folder_list:
            if item.relative_path.endswith(".usd"):
                return item.relative_path
        carb.log_error(f"Unable to find a .usd file in {character_folder_path} character folder")
        return None
    
    ################### Methods for skill interaction ###################
    def get_loc(self):
        """Get the current location of the person."""
        pos = self._state.position
        return (pos[0], pos[1])

    def get_initial_loc(self):
        """Get the current location of the person."""
        pos = self._initial_state.position
        return (pos[0], pos[1])

    def get_distance(self, loc):
        """Get the distance to a target position."""
        return float(np.linalg.norm(self._state.position[:2] - np.array(loc)))

    def run_nav(self, target, callback):
        """Move towards a target [x, y], invoking callback when done."""
        current_z = self._state.position[2]
        gid = self.add_goal([target[0], target[1], current_z], self._target_yaw)
        self._callbacks[gid] = callback
