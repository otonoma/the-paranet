from enum import Enum
from isaacsim.robot.manipulators.examples.franka.controllers import (
    RMPFlowController,
    PickPlaceController,
)
from isaacsim.robot.manipulators.examples.franka.tasks import (
    FollowTarget as FollowTargetTask,
)
import numpy as np


# Define the states for the Franka robot
class FrankaState(Enum):
    IDLE = 0  # Home position
    MOVING = 1


class FrankaSim:
    def __init__(self, name, franka, sim, repair_pos):
        self.name = name
        self.sim = sim
        self._repair_position = np.array(repair_pos)
        self.target = None
        self.cb = None
        self.state = FrankaState.IDLE  # Initialize with IDLE state
        self.controller = RMPFlowController(
            name="franka_moving", robot_articulation=franka
        )
        self.franka = franka
        self.steps = 125
        self._home_position, _ = franka.gripper.get_world_pose()

    @property
    def home(self):
        """Home position of Franka end effector"""
        return self._home_position

    @property
    def repair_position(self):
        """Repair position of franka (defined in config file)"""
        return self._repair_position
    
    # Action to move the Franka robot to a specified position
    # x, y, z are relative to the home position
    def action_goto(self, x, y, z, cb):
        self.target = self._home_position + np.array((x, y, z))
        self.state = FrankaState.MOVING
        self.cb = cb

    # Reset the Franka robot to its home position
    # and set the state to IDLE
    # Also resets the gripper to its opened position
    # Callback is called when the action is complete
    # This is used to notify the caller that the action is done
    def reset(self):
        self.controller.reset()
        self.franka.gripper.set_joint_positions(
            self.franka.gripper.joint_opened_positions
        )
        self.state = FrankaState.IDLE
        self.steps = 125
        if self.cb:
            self.cb("done")
            self.cb = None

    # Forward physics step for the Franka robot
    def physics_step(self):
        # State machine logic
        if self.state == FrankaState.IDLE:
            # In IDLE state, just maintain the home position
            pass
        elif self.state == FrankaState.MOVING:
            self.steps -= 1  # Decrement the step counter
            actions = self.controller.forward(target_end_effector_position=self.target)
            self.franka.apply_action(actions)
            if self.steps == 0:
                self.reset()

    def post_reset(self):
        self.reset()
