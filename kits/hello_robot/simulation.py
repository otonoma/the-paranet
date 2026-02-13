import numpy as np

# Handle 4.2 and 4.5+ import differences
from omni.isaac.version import get_version
try:
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.wheeled_robots.robots import WheeledRobot
    from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
except ModuleNotFoundError:
    from isaacsim.core.utils.nucleus import get_assets_root_path
    from isaacsim.wheeled_robots.robots import WheeledRobot
    from isaacsim.wheeled_robots.controllers.differential_controller import DifferentialController

from actors.jetbot import Jetbot

##### Create a class that represents your robot
class JetbotSim:
    def __init__(self, robot):
        self._robot = robot
        self._controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
        self._command = np.array([0.0, 0.0])

    def initialize(self):
        self._robot.initialize()

    def physics_step(self, step_size):
        self._robot.apply_wheel_actions(self._controller.forward(self._command))

    # Public API for actor class to control the robot
    def set_command(self, cmd):
        self._command = np.array(cmd)


#### Create a main simulation class to create the scene and create instances of your robot classes

class Simulation:
    def __init__(self, world):
        self._world = world

    # Sets up the scene with USD objects
    def setup_scene(self):
        # Need to create a ground plane if not loading a USD scene
        self._world.scene.add_default_ground_plane()

        # Get Isaac Sim version to determine correct path
        version_info = get_version()
        major_version = int(version_info[2])
        if major_version < 5:
            jetbot_usd_path = get_assets_root_path() + "/Isaac/Robots/Jetbot/jetbot.usd"
        else:
            jetbot_usd_path = get_assets_root_path() + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        
        # Add the robot to the scene
        self._jetbot = JetbotSim(WheeledRobot(
            prim_path="/World/Jetbot",
            name="my_jetbot",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jetbot_usd_path,
            position=np.array([0, 0.0, 0.1]),
        ))

    async def setup_post_load(self):
        # robot initialization needs to be done in post load
        self._jetbot.initialize()

        # create and register actors
        self._jetbot_actor = Jetbot(robot=self._jetbot)
        self._jetbot_actor.register()

        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)

    # Called each time step, update each of your robots
    def physics_step(self, step_size):
        self._jetbot.physics_step(step_size)
