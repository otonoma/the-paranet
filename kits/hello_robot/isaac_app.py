import asyncio

######## Start Isaac Sim's app framework

try:
    from omni.isaac.kit import SimulationApp
except ModuleNotFoundError:
    from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})

######## Load extensions

# Must be imported after instantiating SimulationApp
try:
    from omni.isaac.core import World
    import omni.isaac.core.utils.stage as stage_utils
    from omni.isaac.core.utils.extensions import enable_extension
except ModuleNotFoundError:
    from isaacsim.core.api import World
    import isaacsim.core.utils.stage as stage_utils
    from isaacsim.core.utils.extensions import enable_extension

from paranet_agent import connector, actor

### enable any extra extensions here
# e.g. enable_extension("omni.isaac.conveyor")

# Must be imported after enabling extension dependencies
from simulation import Simulation

######## Create instance of Isaac Sim's World

stage_utils.create_new_stage()
world = World()

######## Create your simulation

my_sim = Simulation(world)

async def setup():
    my_sim.setup_scene()
    await world.reset_async()
    await world.pause_async()

    await my_sim.setup_post_load()

    connector.start()
    actor.deploy("isaac")

    await world.play_async()

######## Initialize the simulation

asyncio.ensure_future(setup())

######## Simulation loop

while simulation_app.is_running():
    my_sim._world.step(render=True)
simulation_app.close()
