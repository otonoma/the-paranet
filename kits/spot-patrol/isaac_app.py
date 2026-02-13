from omni.isaac.kit import SimulationApp
from paranet_agent import actor, connector
from paranet_agent.actor import BaseActor, Conversation
from actors.spot_phy import create_spot_instance, Circuit, TaskStatus

import os
import sys
import asyncio

######## Simulation actor


@actor.skill_request(subject="simulation", action="reset", response=TaskStatus)
class ResetRequest:
    pass


@actor.skill_request(subject="patrol", action="assignment", response=TaskStatus)
class PatrolRequest:
    circuit: Circuit


@actor.actor
class SimControl(BaseActor):
    def assign_patrol(self, patrol, to):
        (a, b) = patrol
        circuit = Circuit(col1=a[0], row1=a[1], col2=b[0], row2=b[1])
        self.send_request(PatrolRequest(circuit=circuit), to=to)


######## Start app framework

simulation_app = SimulationApp({"headless": False})


######## Load extensions

# Must be imported after instantiating SimulationApp
import carb
import omni
from omni.kit.actions.core import get_action_registry
import omni.physx.bindings._physx as physx_bindings
from isaacsim.core.api.world import World
from isaacsim.core.utils.viewports import set_camera_view
import isaacsim.core.utils.stage as stage_utils

setting = carb.settings.get_settings()
setting.set_bool(physx_bindings.SETTING_DISPLAY_SIMULATION_OUTPUT, False)

close_windows = set(
    [
          "Content",
          "Console",
          "Stage",
          "Layer",
          "Render Settings",
          "Property",
          "Semantics Schema Editor",
          "Isaac Sim Assets [Beta]",
    ]
)
windows = omni.ui.Workspace.get_windows()
for window in windows:
    if str(window) in close_windows:
        window.destroy()

simulation_app.update()

# Must be imported after enabling extension dependencies
from simulation import Simulation
from libs.map import GRID_DIM, GRID_SIZE, GRID_BL, GRID_BR, GRID_TL, GRID_TR

######## Create world

stage_utils.create_new_stage()
my_sim = Simulation(GRID_DIM, GRID_SIZE)
my_sim._world = World(**my_sim._world_settings)


async def setup():
    ids = ["000", "001"]
    colors = ["blue", "green"]
    patrols = [(GRID_BL, GRID_BR), (GRID_TR, GRID_TL)]

    my_sim.setup_scene()
    spots = [
        my_sim.add_color_spot(ids[i], patrols[i][0], colors[i]) for i in range(len(ids))
    ]
    actors = [create_spot_instance(ids[i], spots[i], my_sim) for i in range(len(ids))]

    set_camera_view(
        eye=[-10.4, -26.5, 10],
        target=[-2, -7, 0],
        camera_prim_path="/OmniverseKit_Persp",
    )

    action_registry = get_action_registry()
    action = action_registry.get_action(
        "omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera"
    )
    action.execute()

    await my_sim._world.reset_async()
    await my_sim._world.pause_async()
    await my_sim.setup_post_load()

    connector.start()

    sim_ctrl = SimControl()
    actor.register_actor(sim_ctrl, requests=[PatrolRequest, ResetRequest])

    for i in range(len(actors)):
        actors[i].register()

    await actor.deploy("isaac", restart=False)

    await asyncio.sleep(3)
    sim_ctrl.send_request(ResetRequest())

    # start initial patrols
    sim_ctrl.assign_patrol(patrols[0], to=f"spot_{ids[0]}")
    sim_ctrl.assign_patrol(patrols[1], to=f"spot_{ids[1]}")


asyncio.ensure_future(setup())


######## Simulation loop

while simulation_app.is_running():
    my_sim._world.step(render=True)
simulation_app.close()
