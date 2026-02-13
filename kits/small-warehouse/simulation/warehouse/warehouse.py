import os
import math
import carb
import numpy as np
import omni.appwindow

from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

from .franka_actor import create_franka_instance
from .spot_actor import create_spot_instance
from .sim_worker_actor import create_sim_worker_instance

from paranet_agent import connector
from paranet_agent.actor import deploy

from .spot_isaac import Spot
from .franka_isaac import FrankaSim
from .sim_worker_isaac import SimWorker

ASSET_PATH = os.environ.get(
    "ASSET_PATH", os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)


class WarehouseSim:
    def __init__(self, sim_actors_cfg: dict) -> None:
        super().__init__()

        # Store the configuration for simulation actors
        self.sim_actors_cfg = sim_actors_cfg

        self._world_settings = {
            "stage_units_in_meters": 1.0,
            "physics_dt": 1.0 / 200.0,
            "rendering_dt": 8.0 / 200.0,
        }

        self._world = World(**self._world_settings)
        self._world.warehouse = self

        # Initialize physics step counters
        self._physics60_steps = 0
        self._physics200_steps = 0

        self.frankas = []
        self.franka_actors = []
        self.spots = []
        self.spot_actors = []
        self.sim_workers = []
        self.sim_workers_actors = []
            
    def setup_scene(self) -> None:
        self._world.scene.add_default_ground_plane(
            z_position=-0.02,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )

        # Load the USD file for the warehouse
        usd_file_path = os.path.join(ASSET_PATH, "Warehouse.usd")
        carb.log_info(f"Adding {usd_file_path} to stage …")
        add_reference_to_stage(usd_path=usd_file_path, prim_path="/World")

        # Adding frankas to the scene
        for arm_cfg in self.sim_actors_cfg.get("robot_arm", []):
            name = arm_cfg["name"]
            pos = np.array(arm_cfg["position"])
            self._world.scene.add(
                Franka(prim_path=f"/World/{name}", name=name, position=pos)
            )

        # Adding spots to the scene
        for spot_cfg in self.sim_actors_cfg.get("robot_quadruped", []):
            name = spot_cfg["name"]
            loc = spot_cfg["position"]
            color = spot_cfg.get("color", "default")
            spot = Spot(f"/World/{name}", loc, color)
            spot.setup_scene()
            self.spots.append(spot)
            self.spot_actors.append(create_spot_instance(name, spot))

        # Adding sim sim_workers
        for worker_cfg in self.sim_actors_cfg.get("sim_workers", []):
            stage_prefix = worker_cfg["stage_prefix"]
            character_name = worker_cfg["character_name"]
            work_position = worker_cfg.get("work_position", [0.0, 0.0, 0.0])
            yaw = worker_cfg.get("yaw", 0.0)
            sim_worker = SimWorker(self._world, stage_prefix, character_name, work_position, yaw)
            self.sim_workers.append(sim_worker)
            self.sim_workers_actors.append(create_sim_worker_instance(stage_prefix, sim_worker))

    async def setup_post_load(self) -> None:

        # Create Franka instances and their actors
        for arm in self.sim_actors_cfg.get("robot_arm", []):
            name = arm["name"]
            repair_position = arm["repair_position"]
            franka_obj = self._world.scene.get_object(name)
            franka_sim = FrankaSim(name, franka_obj, self, repair_position)
            self.frankas.append(franka_sim)
            self.franka_actors.append(create_franka_instance(name, franka_sim, self))

        # Setup post-load for Franka robots
        for spot in self.spots:
            spot.setup_post_load()

        # Setup post-load for Sim Workers
        for sim_worker in self.sim_workers:
            sim_worker.setup_post_load()

        self._physics_ready = False
        self._world.add_physics_callback("sim_step", self.on_physics_step)

    async def setup_post_reset(self) -> None:
        self._physics_ready = False
        for franka in self.frankas:
            franka.post_reset()
        for spot in self.spots:
            spot.setup_post_reset()
        for sim_worker in self.sim_workers:
            sim_worker.setup_post_reset()

    def on_physics_step(self, step_size) -> None:

        # Calculate the current step in terms of 60Hz
        t60 = math.floor(self._physics200_steps / 200 * 60)

        # Perform physics step for Franka robots at 60Hz
        if t60 == self._physics60_steps:
            # Perform physics step for Franka robots
            for franka in self.frankas:
                franka.physics_step()
            self._physics60_steps += 1

        # Perform physics step for Spot robots at 200Hz
        if self._physics_ready:
            # Perform physics step for Spot robots
            for spot in self.spots:
                spot.on_physics_step(step_size, np.zeros((3,)))
            self._physics200_steps += 1
        else:
            carb.log_info("Physics Start")
            self._physics_ready = True

    def start_actors(self):
        carb.log_info("Starting connector…")
        try:
            connector.start()
            carb.log_info("Connector started")
        except Exception as e:
            carb.log_error(f"connector.start() failed: {e}")

        for kind, actor_list in [
            ("spot", self.spot_actors),
            ("franka", self.franka_actors),
            ("worker", self.sim_workers_actors),
        ]:
            for actor in actor_list:
                try:
                    carb.log_info(f"Registering {kind} actor {getattr(actor, 'id', repr(actor))}…")
                    actor.register()
                    carb.log_info("Registered")
                except Exception as e:
                    carb.log_error(f"{kind} actor failed to register: {e}")

        carb.log_info("Deploying to isaac…")
        try:
            deploy("isaac", restart=True)
            carb.log_info("Deploy succeeded. Actors started.")
        except Exception as e:
            carb.log_error(f"deploy() failed: {e}")

    # Stop the actors and the connector
    def stop_actors(self):
        connector.stop()
        carb.log_info("Actors stopped")

    # Cleanup the world by removing physics callbacks
    def world_cleanup(self):
        if self._world.physics_callback_exists("sim_step"):
            self._world.remove_physics_callback("sim_step")
