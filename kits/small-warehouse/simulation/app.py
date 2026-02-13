import os
import asyncio
import argparse
import yaml
import omni.kit.app

from isaacsim.simulation_app import SimulationApp

# Set the base directory and configuration directory
BASE_DIR = os.path.dirname(__file__)
CFG_DIR = os.path.join(BASE_DIR, "config")
available = [f for f in os.listdir(CFG_DIR) if f.endswith((".yml", ".yaml"))]

# Ensure the configuration directory exists
parser = argparse.ArgumentParser()
parser.add_argument("--config", "-c", choices=available, default="sim_config.yml")
args = parser.parse_args()

# Load the configuration file
config_path = os.path.join(CFG_DIR, args.config)
with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)

# Check if the configuration file was loaded successfully
headless = cfg.get("headless", False)
low_res = cfg.get("low_res", False)

# Initialize the simulation application with the specified configuration
win_w, win_h = 1920, 1080
buf_w, buf_h = 1440, 900
if low_res:
    win_w //= 2
    win_h //= 2
    buf_w //= 2
    buf_h //= 2

# Create the simulation application instance
experience = os.path.join(os.environ['EXP_PATH'], 'isaacsim.exp.full.kit')
simulation_app = SimulationApp(
    {
        "headless": headless,
        "window_width": win_w,
        "window_height": win_h,
        "width": buf_w,
        "height": buf_h,
    }, experience
)

# Delay extension-related imports until after app starts
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import create_new_stage

# Enable necessary extensions
for ext in [
    "omni.kit.scripting",
    "omni.warehouse_creator",
    "omni.anim.graph.core",
    "omni.anim.graph.schema",
    "omni.anim.graph.ui",
    "omni.anim.people",
]:
    enable_extension(ext)

# Update the simulation application to ensure all extensions are loaded
simulation_app._app.update()

# Import the WarehouseSim class after the application is set up
from warehouse.warehouse import WarehouseSim

# Create a new stage for the simulation
create_new_stage()
warehouse = WarehouseSim(cfg.get("sim_actors", {}))


# Set up the scene and actors
async def setup():
    warehouse.setup_scene()

    await warehouse._world.reset_async()
    await warehouse._world.pause_async()
    await warehouse.setup_post_load()

    action_registry = omni.kit.actions.core.get_action_registry()

    if low_res:
        # switches to camera lighting
        action = action_registry.get_action(
            "omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera"
        )
        action.execute()
    # Start the actors in the warehouse
    warehouse.start_actors()


# Run the setup function asynchronously
asyncio.ensure_future(setup())

# Wait for the setup to complete
while simulation_app.is_running():
    warehouse._world.step(render=True)

# Close the simulation application when done
simulation_app.close()
