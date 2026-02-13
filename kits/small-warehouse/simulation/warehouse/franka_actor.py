from paranet_agent import actor
from paranet_agent.actor import BaseActor, Conversation
from enum import Enum


# Enum for Franka states
class FrankaState(Enum):
    IDLE_STATE = "idle"
    DISABLED_STATE = "disabled"


@actor.type
class FrankaStatus:
    status: str


# Position class: represents location data for actor
@actor.type
class Location:
    x: float
    y: float


def create_franka_instance(id, robot, sim=None):
    @actor.actor(name=id, subject="franka")
    class FrankaPhy(BaseActor):
        id: str
        robot: object
        sim: object
        state: FrankaState = FrankaState.IDLE_STATE

        # Registers itself as an actor
        def __post_init__(self):
            self.register()

        @actor.skill(id="franka@1.0.0")
        def get_location(self, conversation: Conversation) -> Location:
            position = self.robot.home
            return Location(x=position[0], y=position[1])
        
        @actor.skill(id="franka@1.0.0")
        def get_repair_location(self, conversation: Conversation) -> Location:
            position = self.robot.repair_position
            return Location(x=position[0], y=position[1])

        # Disables Franka (moves to a rest pose and blocks other actions)
        @actor.skill(id="franka@1.0.0")
        def disable(self) -> FrankaStatus:
            if self.state != FrankaState.DISABLED_STATE:
                self.robot.action_goto(-1, 0.0, -1, cb=self)  # values were changed here
                self.state = FrankaState.DISABLED_STATE
            return FrankaStatus(status=self.state.value)
      
        # Enables Franka (returns to home)
        @actor.skill(id="franka@1.0.0")
        def enable(self) -> FrankaStatus:
            if self.state == FrankaState.DISABLED_STATE:
                self.state = FrankaState.IDLE_STATE
                self.robot.action_goto(0.0, 0.0, 0.0, cb=self)
            return FrankaStatus(status=self.state.value)

        # Returns the current status/state of the Franka
        @actor.skill(id="franka@1.0.0")
        def status(self, conversation: Conversation) -> FrankaStatus:
            return FrankaStatus(status=self.state.value)

        # Callback function to be called when the action is complete
        def __call__(self, result=None):
            print("CALLBACK", self.id, self.state)

        def register(self):
            actor.register_actor(self)

    return FrankaPhy(robot=robot, id=id, sim=sim)
