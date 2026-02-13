import asyncio

from paranet_agent import actor, connector
from paranet_agent.actor import BaseActor

########## Simple actor

## Greeter Actor

# Declare types used as skill arguments and responses

@actor.type
class Greeting:
  message: str

# Declare the actor class

@actor.actor
class Greeter(BaseActor):
  @actor.skill
  def greet(self, name: str) -> Greeting:
    return Greeting(message = 'HELLO ' + name)

########## Startup

# Create an instance of the actor and register it

actor.register_actor(Greeter())

# Start connector service
connector.start()

# Deploy all actors
actor.deploy('hello_py', restart=False)

# Run a main loop
loop = asyncio.events.get_event_loop()
loop.run_until_complete(connector.get_task())