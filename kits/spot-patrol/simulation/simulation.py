import carb
import numpy as np
import omni
import omni.appwindow  # Contains handle to keyboard

from isaacsim.core.prims import SingleRigidPrim as RigidPrim
from isaacsim.core.utils import stage as stage_utils
from isaacsim.util.debug_draw import _debug_draw
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.storage.native import get_assets_root_path

from .spot_sim import Spot

class Simulation:
    def __init__(self, GRID_DIM, GRID_SIZE) -> None:
        self.GRID_DIM = np.array(GRID_DIM)
        self.GRID_SIZE = np.array(GRID_SIZE)
        WORLD_SIZE = (self.GRID_DIM - 1) * self.GRID_SIZE
        self.GRID_WORLD_ORIGIN = -WORLD_SIZE / 2.0

        self._world_settings = {}
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 500.0
        self._world_settings["rendering_dt"] = 10.0 / 500.0

        self._base_command = [0.0, 0.0, 0.0]
        self.steps_per_sec = 1 / self._world_settings["physics_dt"]
        self.select_idx = 0

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [2.0, 0.0, 0.0],
            "UP": [2.0, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-2.0, 0.0, 0.0],
            "DOWN": [-2.0, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -2.0, 0.0],
            "RIGHT": [0.0, -2.0, 0.0],
            # right command
            "NUMPAD_4": [0.0, 2.0, 0.0],
            "LEFT": [0.0, 2.0, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 2.0],
            "N": [0.0, 0.0, 2.0],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -2.0],
            "M": [0.0, 0.0, -2.0],
        }

    def setup_scene(self) -> None:

        usd_file_path = "scene.usd"
        stage_utils.add_reference_to_stage(
            usd_path=usd_file_path, prim_path="/World/scene"
        )
        assets_root_path = get_assets_root_path()
        cone_usd_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd"
        for i in range(2):
            for j in range(2):
                path = f"/World/Cone_{i}_{j}"
                stage_utils.add_reference_to_stage(usd_path=cone_usd_path, prim_path=path)
                cone = RigidPrim(path)
                cone.set_world_pose(
                    position=np.array(
                        [-6 + (2 * i - 1) * 0.3, -2 + (2 * j - 1) * 0.3, 0.0]
                    )
                )

        self.spots = []
        self.debug_map = {}

    def add_spot(self, id, loc):
        path = f"/World/Spot_{id}"
        self.spots.append(Spot(path, self.GRID_DIM, self.GRID_SIZE, loc))
        spot = self.spots[-1]
        spot.setup_scene()
        return spot

    def add_color_spot(self, id, loc, color):
        path = f"/World/Spot_{id}"
        self.spots.append(Spot(path, self.GRID_DIM, self.GRID_SIZE, loc, color))
        spot = self.spots[-1]
        spot.setup_scene()
        return spot

    async def setup_post_load(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )
        self._world.add_physics_callback(
            "physics_step", callback_fn=self.on_physics_step
        )
        self._physics_ready = False
        await self._world.play_async()
        for i in range(len(self.spots)):
            self.spots[i].setup_post_load()

        # Map background
        self.cube = VisualCuboid(
            prim_path="/World/MapPanel",
            name="MapPanel",
            position=np.array([-11, 10.1, 2.5]),
            scale=np.array([4.4, 0.1, 3.4]),
            color=np.array([0.1, 0.1, 0.1]),
        )

        self.draw_map()

    def on_physics_step(self, step_size) -> None:
        if self._physics_ready == True:
            for i in range(len(self.spots)):
                cmd = self._base_command if i == self.select_idx else np.zeros((3,))
                self.spots[i].on_physics_step(step_size, cmd)
        else:
            self._physics_ready = True

    def update_map(self, id, path):
        self.debug_map[id] = path
        self.draw_map()

    def draw_map(self):
        draw = _debug_draw.acquire_debug_draw_interface()
        draw.clear_lines()
        # (points_1, points_2, sizes, colors) = self.floor_lines()
        points_1 = []
        points_2 = []
        sizes = []
        colors = []
        palette = [[0.122, 0.376, 0.71, 1.0], [0.176, 0.6, 0.235, 1.0]]
        ids = sorted(list(self.debug_map.keys()))
        for idx, id in enumerate(ids):
            path = self.debug_map[id]
            for i in range(len(path) - 1):
                [x1, y1] = path[i]
                [x2, y2] = path[i + 1]
                points_1.append([-13 + x1, 10, 1 + y1])
                points_2.append([-13 + x2, 10, 1 + y2])
                sizes.append(5.0)
                colors.append(palette[idx])
        draw.draw_lines(points_1, points_2, colors, sizes)

    def floor_lines(self):
        [x1, y1] = self.GRID_WORLD_ORIGIN
        [x2, y2] = self.GRID_WORLD_ORIGIN + (self.GRID_DIM - 1) * self.GRID_SIZE
        points_1 = [[x1, y1, 0.0], [x1, y2, 0.0], [x2, y2, 0.0], [x2, y1, 0.0]]
        points_2 = [[x1, y2, 0.0], [x2, y2, 0.0], [x2, y1, 0.0], [x1, y1, 0.0]]
        sizes = [5.0, 5.0, 5.0, 5.0]
        color = [0.0, 0.0, 0.0, 1.0]
        colors = [color, color, color, color]
        return (points_1, points_2, sizes, colors)

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(
                    self._input_keyboard_mapping[event.input.name]
                )
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(
                    self._input_keyboard_mapping[event.input.name]
                )
        return True
