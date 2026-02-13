import asyncio
import carb
from paranet_agent import actor
from paranet_agent.actor import BaseActor, Conversation

IDLE_STATE = "idle"
MOVING_STATE = "moving"


# Position class: represents input data for actor
@actor.input
class Position:
    x: float
    y: float


# Target class: represents target position
@actor.type
class Target:
    x: float
    y: float


# Task Status class: represents the status of spot
@actor.type
class SpotTaskStatus:
    status: str


# Distance class: represents distance
@actor.type
class DistanceResult:
    distance: float


# Creates and registers actor
def create_spot_instance(id, robot, sim=None):
    # Actor class definition
    @actor.actor(name=id, subject="spot")
    class SpotPhy(BaseActor):
        id: str
        robot: object
        sim: object
        target: list[float] = None
        conversation: Conversation = None

        state: str = IDLE_STATE

        # Moves robot to target postion
        @actor.skill(id="spot@1.0.0", response=SpotTaskStatus)
        def spot_goto(self, x: float, y: float, conversation: Conversation) -> None:
            carb.log_info(f"GOTO COMMAND RECEIVED: {x}, {y}")
            asyncio.ensure_future(self.run_spot_goto(x, y, conversation))

        # Asynchronous method to run the goto command
        async def run_spot_goto(self, x: float, y: float, conversation: Conversation):
            carb.log_info(f"RUN GOTO: {x}, {y}, state={self.state}")
            if self.state == IDLE_STATE:
                carb.log_info(f"Starting navigation to {x}, {y}")
                self.state = MOVING_STATE
                self.target = [x, y]
                self.conversation = conversation
                # Function in spot_isaac, triggers call back for completion
                self.robot.run_nav(self.target, self)
            else:
                conversation.send_response(SpotTaskStatus(status=self.state))

        # Returns current position of the robot
        @actor.skill(id="spot@1.0.0")
        def get_position(self, conversation: Conversation) -> Target:
            pos = self.robot.get_loc()
            return Target(x=pos[0], y=pos[1])

        # Returns the distance to a target position
        @actor.skill(id="spot@1.0.0")
        def distance(
            self, x: float, y: float, conversation: Conversation
        ) -> DistanceResult:
            dist = self.robot.get_distance((x, y))
            return DistanceResult(distance=dist)

        # Returns the current state
        @actor.skill(id="spot@1.0.0")
        def get_status(self, conversation: Conversation) -> SpotTaskStatus:
            return SpotTaskStatus(status=self.state)

        # Returns current target coordinates
        @actor.skill(id="spot@1.0.0")
        def get_target(self, conversation: Conversation) -> Target:
            if self.target is None:
                pos = self.robot.get_loc()
                carb.log_info(f"GET TARGET: No target set, returning current pos {pos}")
                return Target(x=pos[0], y=pos[1])
            carb.log_info(f"GET TARGET: {self.target}")
            return Target(x=self.target[0], y=self.target[1])

        # Callback from robot when action complete
        def __call__(self):
            carb.log_info(f"Worker GOTO COMPLETE FOR {self.conversation}")
            self.state = IDLE_STATE
            if self.conversation:
                self.conversation.send_response(SpotTaskStatus(status=self.state))
                self.conversation = None

        # Registers the actor
        def register(self):
            actor.register_actor(self)

    return SpotPhy(robot=robot, id=id, sim=sim)


# Delay function to simulate asynchronous behavior
async def delay(ms, callback):
    await asyncio.sleep(ms / 1000)
    callback()
