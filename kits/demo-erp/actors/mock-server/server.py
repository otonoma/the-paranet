import asyncio
from paranet_agent import actor, connector
from paranet_agent.actor import BaseActor, Conversation

@actor.type
class ServerStatus:
    status: str

@actor.actor
class Server(BaseActor):

    @actor.skill(subject="server", response=ServerStatus)
    def wait(self, conv: Conversation) -> None:
        asyncio.ensure_future(self.run_wait(conv))

    async def run_wait(self, conv: Conversation):
        print(f"Async call called")
        await asyncio.sleep(4)
        conv.send_response(ServerStatus(status="done"))

actor.register_actor(Server())
connector.start()
actor.deploy("Server", restart=False)

loop = asyncio.get_event_loop()
loop.run_until_complete(connector.get_task())
