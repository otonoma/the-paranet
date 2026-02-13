import os
import sys

root_dir = os.path.join(os.path.dirname(__file__), "..")
sys.path.append(os.path.join(root_dir, "libs"))

import numpy as np
import omni

from isaacsim.core.prims import SingleRigidPrim
from isaacsim.sensors.physx import _range_sensor  # Imports the python bindings to interact with Lidar sensor
from pxr import UsdGeom

from math_util import euler_from_quaternion
from vfh import VFH
from .custom_spot_terrain_policy import CustomSpotFlatTerrainPolicy

CLOUD_CLIP_DIST = 10


class Spot:
    def __init__(self, root_path, GRID_DIM, GRID_SIZE, drop_loc, color):
        self.name = " ".join(root_path[1:].split("/"))
        self.prefix = "_".join(root_path[1:].split("_")) + "_"
        self.root_path = root_path

        self.GRID_DIM = np.array(GRID_DIM)
        self.GRID_SIZE = np.array(GRID_SIZE)
        WORLD_SIZE = (self.GRID_DIM - 1) * self.GRID_SIZE
        self.GRID_WORLD_ORIGIN = -WORLD_SIZE / 2.0
        self.drop_loc = drop_loc

        self.cmd = np.zeros((3,))

        self.iter = 0
        self.moving = False
        self.callback = None
        self.color = color

    def grid_world_position(self, grid_loc):
        col, row = grid_loc
        pos = np.array([col, row]) * self.GRID_SIZE + self.GRID_WORLD_ORIGIN
        return np.append(pos, 0.8)

    def get_approx_loc(self):
        pos, _ = self.lidar.get_world_pose()
        loc = np.round((pos[0:2] - self.GRID_WORLD_ORIGIN) / self.GRID_SIZE).astype(
            np.int32
        )
        return (loc[0], loc[1])

    def setup_scene(self) -> None:
        self.spot = CustomSpotFlatTerrainPolicy(
            prim_path=self.root_path,
            name=self.name,
            position=self.grid_world_position(self.drop_loc),
            color=self.color,
        )
        self.spot.robot.set_joints_default_state(self.spot.default_pos)
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="Lidar",
            parent=self.root_path + "/body",
            # draw_lines=True,
            rotation_rate=0.0,
            horizontal_fov=180,
        )
        UsdGeom.XformCommonAPI(prim).SetTranslate((0.5, 0.0, -0.12))
        self.lidar_path = self.root_path + "/body/Lidar"
        self.lidar = SingleRigidPrim(self.lidar_path)
        self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

    def setup_post_load(self):
        self.spot.initialize()
        self.nav = VFH(self.spot.robot)

    def on_physics_step(self, step_size, keyboard_cmd) -> None:
        if self.moving:
            if (self.iter % 5) == 0:
                cloud = self.get_world_cloud()
                self.nav.update_cloud(cloud)
                if (self.iter % 10) == 0:
                    self.cmd = self.nav.get_command()
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
            cmd = keyboard_cmd
        self.spot.advance(step_size, cmd)
        self.iter += 1

    def get_world_cloud(self):
        depth = self.lidarInterface.get_linear_depth_data(self.lidar_path)
        azimuth = self.lidarInterface.get_azimuth_data(self.lidar_path)
        idx = depth[:, 0] < CLOUD_CLIP_DIST
        origin, rot = self.lidar.get_world_pose()
        rot = euler_from_quaternion(rot)
        x = depth[idx, 0] * np.cos(azimuth[idx] + rot[2]) + origin[0]
        y = depth[idx, 0] * np.sin(azimuth[idx] + rot[2]) + origin[1]
        return np.stack([x, y], axis=-1)

    def run_nav(self, loc, callback):
        if self.moving:
            raise Exception(f"{self.name}: received command while moving")
        self.callback = callback
        target = self.grid_world_position(loc)
        pos, _ = self.lidar.get_world_pose()
        print(self.name, "NAV", pos[0:2], target[0:2])
        cloud = self.get_world_cloud()
        self.nav.clear()
        self.nav.set_target(target[0:2])
        self.nav.update_cloud(cloud)
        self.moving = True

    def stop_nav(self):
        if self.moving:
            self.nav.stop()

    def get_loc(self):
        return self.get_approx_loc()

    def get_distance(self, loc):
        pos, _ = self.lidar.get_world_pose()
        target = self.grid_world_position(loc)
        return np.linalg.norm([pos[0:2], target[0:2]])
