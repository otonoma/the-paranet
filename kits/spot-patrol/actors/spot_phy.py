import os
import sys

root_dir = os.path.join(os.path.dirname(__file__),'..')
sys.path.append(os.path.join(root_dir,'libs'))

import asyncio
from typing import Optional
from paranet_agent import actor, connector
from paranet_agent.actor import BaseActor, Conversation

from map import get_patrol_path, get_fastest_path, create_patrol_around, recompute_patrol_path

IDLE_STATE = 'idle'
PATROL_STATE = 'patrol'
INSPECT_WALK_STATE = 'inspect/walk'
INSPECT_STATE = 'inspect'
RETURN_STATE = 'return'


@actor.input
class Circuit:
  row1: int
  col1: int
  row2: int
  col2: int

@actor.input
class Location:
  row: int
  col: int

@actor.type
class TaskStatus:
    status: str

@actor.type
class DistanceResult:
  distance: float

def create_spot_instance(id, robot, sim=None):
  @actor.actor(name=f'spot_phy_{id}', subject='spot_phy')
  class SpotPhy(BaseActor):
    id: str
    robot: object
    sim: object
    target: list[int] = None
    patrol: tuple = None
    suspended_patrol: tuple = None
    path: list[list[int]] = None
    inspect_loc: tuple = None
    conversation: Conversation = None
    state: str = IDLE_STATE
    next_state: str = IDLE_STATE

    @actor.skill(id='spot_phy@1.0.0')
    def get_distance(self, col: int, row: int) -> DistanceResult:
      loc = self.robot.get_loc()
      path = get_fastest_path(loc, [col, row])
      return DistanceResult(distance=len(path))

    @actor.skill(id='spot_phy@1.0.0')
    def start_patrol(self, circuit: Circuit) -> None:
      start = [circuit.col1, circuit.row1]
      finish = [circuit.col2, circuit.row2]
      self.set_patrol(start, finish, False)

    @actor.skill(id='spot_phy@1.0.0')
    def end_patrol(self) -> None:
      self.next_state = IDLE_STATE

    @actor.skill(response=TaskStatus, id='spot_phy@1.0.0')
    def inspect(self, col: int, row: int, conv: Conversation) -> None:
      self.next_state = INSPECT_WALK_STATE
      self.inspect_loc = (col, row)
      self.conversation = conv
      if self.state == PATROL_STATE:
        self.robot.stop_nav()
      elif self.state == IDLE_STATE:
        self.run_next()

      if self.sim is not None:
        self.sim.update_map(self.id, [])

    @actor.skill(id='spot_phy@1.0.0')
    def request_cover(self, circuit: Circuit, avoid: Optional[Location]) -> TaskStatus:
      print(self.id, 'COVER', circuit, avoid)
      if self.state == IDLE_STATE or self.state == PATROL_STATE:
        if avoid is not None:
          new_patrol = create_patrol_around(avoid.col, avoid.row)
        if self.state == PATROL_STATE:
          self.suspended_patrol = self.patrol
          print('COVER PATROL', new_patrol)
          # self.patrol is the next walk, so current is opposite direction
          cur_dir = not self.patrol[2]
          if cur_dir == new_patrol[2]:
            # next walk should be opposite of current
            new_patrol = (new_patrol[1], new_patrol[0], not new_patrol[2])

          # extend current walk to new starting point of next walk
          self.path = recompute_patrol_path(self.path, self.robot.get_loc(), new_patrol[0], cur_dir)
          self.robot.stop_nav()
        else:
          self.suspended_patrol = None

        self.set_patrol(new_patrol[0], new_patrol[1], new_patrol[2])

        return TaskStatus(status = "OK")
      else:
        return TaskStatus(status = "N/A")

    @actor.skill(id='spot_phy@1.0.0')
    def end_cover(self) -> TaskStatus:
      if self.suspended_patrol is None:
        self.next_state = IDLE_STATE
      else:
        (start, finish, clockwise) = self.suspended_patrol
        self.suspended_patrol = None

        # self.patrol is the next walk, so current is opposite direction
        cur_dir = not self.patrol[2]
        if cur_dir == clockwise:
          # next walk should be opposite of current
          (start, finish, clockwise) = (finish, start, not clockwise)
        self.set_patrol(start, finish, clockwise)

      return TaskStatus(status = "OK")

    def set_patrol(self, start, finish, clockwise):
      self.patrol = (start, finish, clockwise)
      self.next_state = PATROL_STATE
      print('SET PATROL', self.patrol)
      if self.state == IDLE_STATE:
        self.run_next()

      if self.sim is not None:
        self.sim.update_map(self.id, get_patrol_path(start, finish, clockwise))

    def run_next(self):
      print(self.id, 'STATE', self.state, '->', self.next_state)
      self.state = self.next_state

      if self.state == PATROL_STATE:
        self.next_state = PATROL_STATE
        (start, finish, clockwise) = self.patrol
        self.path = get_patrol_path(start, finish, clockwise)
        print(id, 'NEW PATROL', self.path)
        # reverse for next time
        self.patrol = (finish, start, not clockwise)
        self.run_patrol()
      elif self.state == INSPECT_WALK_STATE:
        self.next_state = INSPECT_STATE
        self.path = get_fastest_path(self.robot.get_loc(), self.inspect_loc)
        self.path.pop(0)
        print(id, 'NEW INSPECT', self.path)
        self.run_inspect()
      elif self.state == INSPECT_STATE:
        self.next_state = IDLE_STATE if self.patrol is None else RETURN_STATE
        self.run_inspect()
      elif self.state == RETURN_STATE:
        if self.conversation:
          self.conversation.send_response(TaskStatus(status='done'))

        self.next_state = PATROL_STATE
        (start, finish, clockwise) = self.patrol
        path1 = get_fastest_path(self.robot.get_loc(), start)
        path2 = get_fastest_path(self.robot.get_loc(), finish)
        # walk to closest end point
        if len(path1) <= len(path2):
          # path1 ends at current patrol start
          self.path = path1
        else:
          # path2 ends at current patrol finish, so reverse
          self.patrol = (finish, start, not clockwise)
          self.path = path2
        self.run_return()

        if self.sim is not None:
          self.sim.update_map(self.id, get_patrol_path(start, finish, clockwise))

    def run_path(self):
      if len(self.path) > 0:
        self.target = self.path[0]
        self.path = self.path[1:]
        robot.run_nav(self.target, self)
        return True
      else:
        return False

    def run_patrol(self):
      if not self.run_path():
        self.run_next()

    def run_inspect(self):
      if self.state == INSPECT_WALK_STATE:
        if not self.run_path():
          self.run_next()
      elif self.state == INSPECT_STATE:
        asyncio.ensure_future(delay(30000, self.run_next))

    def run_return(self):
      if not self.run_path():
        self.run_next()

    # callback from robot when action complete
    def __call__(self):
      if self.state == PATROL_STATE:
        self.run_patrol()
      elif self.state == INSPECT_STATE or self.state == INSPECT_WALK_STATE:
        self.run_inspect()
      elif self.state == RETURN_STATE:
        self.run_return()

    def register(self):
      actor.register_actor(self)

  return SpotPhy(robot=robot, id=id, sim=sim)

async def delay(ms, callback):
  await asyncio.sleep(ms/1000)
  callback()
