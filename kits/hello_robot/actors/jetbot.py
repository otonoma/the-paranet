from paranet_agent import actor, connector
from paranet_agent.actor import BaseActor, Conversation

@actor.input
class SetSpeed:
  left: float
  right: float

@actor.type
class Speed:
  left: float
  right: float

@actor.type
class TaskStatus:
  status: str

@actor.actor(name='jetbot')
class Jetbot(BaseActor):
    # id: str
    status: str = "Stopped"
    robot: object

    @actor.skill(subject="jetbot", response=TaskStatus)
    def forward(self, conv: Conversation) -> None:
        self.robot.set_command([3.0, 3.0])
        self.status = "Moving"
        conv.send_response(TaskStatus(status=self.status))

    @actor.skill(subject="jetbot", response=TaskStatus)
    def backward(self, conv: Conversation) -> None:
        self.robot.set_command([-3.0, -3.0])
        self.status = "Moving"
        conv.send_response(TaskStatus(status=self.status))

    @actor.skill(subject="jetbot", response=TaskStatus)
    def stop(self, conv: Conversation) -> None:
        self.robot.set_command([0.0, 0.0])
        self.status = "Stopped"
        conv.send_response(TaskStatus(status=self.status))

    @actor.skill(subject="jetbot", response=TaskStatus)
    def getStatus(self, conv: Conversation) -> None:
        conv.send_response(TaskStatus(status=self.status))

    # Registers this actor instance with the SDK
    def register(self):
       actor.register_actor(self)