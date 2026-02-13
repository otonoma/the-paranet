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
class SimWorkerTarget:
    x: float
    y: float


# Task Status class: represents the status of SimWorker
@actor.type
class SimWorkerTaskStatus:
    status: str


# Distance class: represents distance
@actor.type
class SimWorkerDistanceResult:
    distance: float


# Creates and registers actor
def create_sim_worker_instance(id, person, sim=None):
    # Actor class definition
    @actor.actor(name=id, subject="sim_worker")
    class SimWorkerPhy(BaseActor):
        id: str
        person: object
        sim: object
        target: list[float] = None
        conversation: Conversation = None

        state: str = IDLE_STATE

        # Moves person to target position
        @actor.skill(id="sim_worker@1.0.0", response=SimWorkerTaskStatus)
        def worker_goto(self, x: float, y: float, conversation: Conversation) -> None:
            carb.log_info(f"[SimWorker:{self.id}] GOTO command received: {x}, {y}")
            asyncio.ensure_future(self.run_worker_goto(x, y, conversation))

        # Asynchronous method to run the goto command
        async def run_worker_goto(self, x: float, y: float, conversation: Conversation):
            carb.log_info(f"[SimWorker:{self.id}] RUN GOTO (state={self.state})")
            if self.state == IDLE_STATE:
                carb.log_info(f"[SimWorker:{self.id}] Starting navigation to {x}, {y}")
                self.state = MOVING_STATE
                self.target = [x, y]
                self.conversation = conversation
                # Function in sim_worker_isaac, triggers call back for completion
                self.person.run_nav(self.target, self)
            else:
                carb.log_warn(f"[SimWorker:{self.id}] Busy, state={self.state}")
                conversation.send_response(SimWorkerTaskStatus(status=self.state))

        # Returns current position of the person
        @actor.skill(id="sim_worker@1.0.0")
        def get_position(self, conversation: Conversation) -> SimWorkerTarget:
            pos = self.person.get_loc()
            carb.log_info(f"[SimWorker:{self.id}] Current position: {pos}")
            return SimWorkerTarget(x=pos[0], y=pos[1])
                 
        # Returns initial position of the person
        @actor.skill(id="sim_worker@1.0.0")
        def get_initial_position(self, conversation: Conversation) -> SimWorkerTarget:
            pos = self.person.get_initial_loc()
            carb.log_info(f"[SimWorker:{self.id}] Initial position: {pos}")
            return SimWorkerTarget(x=pos[0], y=pos[1])

        # Returns the distance to a target position
        @actor.skill(id="sim_worker@1.0.0")
        def distance(
            self, x: float, y: float, conversation: Conversation
        ) -> SimWorkerDistanceResult:
            dist = self.person.get_distance((x, y))
            carb.log_info(f"[SimWorker:{self.id}] Distance to ({x},{y}) = {dist}")
            return SimWorkerDistanceResult(distance=dist)

        # Returns the current state
        @actor.skill(id="sim_worker@1.0.0")
        def get_status(self, conversation: Conversation) -> SimWorkerTaskStatus:
            carb.log_info(f"[SimWorker:{self.id}] Status requested → {self.state}")
            return SimWorkerTaskStatus(status=self.state)

        # Returns current target coordinates
        @actor.skill(id="sim_worker@1.0.0")
        def get_target(self, conversation: Conversation) -> SimWorkerTarget:
            if self.target is None:
                pos = self.person.get_loc()
                carb.log_info(f"[SimWorker:{self.id}] No target set, returning current pos {pos}")
                return SimWorkerTarget(x=pos[0], y=pos[1])
            carb.log_info(f"[SimWorker:{self.id}] Current target: {self.target}")
            return SimWorkerTarget(x=self.target[0], y=self.target[1])

        # Callback from person when action complete
        def __call__(self):
            carb.log_info(f"[SimWorker:{self.id}] Worker GOTO COMPLETE for {self.conversation}")
            self.state = IDLE_STATE
            if self.conversation:
                self.conversation.send_response(SimWorkerTaskStatus(status=self.state))
                self.conversation = None

        # Registers the actor
        def register(self):
            carb.log_info(f"[SimWorker:{self.id}] Registering actor")
            actor.register_actor(self)

    return SimWorkerPhy(person=person, id=id, sim=sim)

# Delay function to simulate asynchronous behavior
async def delay(ms, callback):
    carb.log_info(f"Delay {ms}ms before callback")
    await asyncio.sleep(ms / 1000)
    callback()
