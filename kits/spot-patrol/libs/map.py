import numpy as np

GRID_DIM = [5, 4]
GRID_SIZE = [6.0, 6.0]

LEFT = 0
NEAR_LEFT = LEFT + 1
RIGHT = GRID_DIM[0] - 1
NEAR_RIGHT = RIGHT - 1
TOP = GRID_DIM[1] - 1
NEAR_TOP = TOP - 1
BOTTOM = 0
NEAR_BOTTOM = BOTTOM + 1

GRID_BL = [LEFT, BOTTOM]
GRID_TL = [LEFT, TOP]
GRID_TR = [RIGHT, TOP]
GRID_BR = [RIGHT, BOTTOM]

def is_equal(pt1, pt2):
  return pt1[0] == pt2[0] and pt1[1] == pt2[1]   

def is_top(pt):
  return pt[1] == TOP

def is_bottom(pt):
  return pt[1] == BOTTOM

def is_left(pt):
  return pt[0] == LEFT

def is_right(pt):
  return pt[0] == RIGHT

def is_above(pt1, pt2):
  return pt1[1] > pt2[1]

def is_below(pt1, pt2):
  return pt1[1] < pt2[1]

def is_left_of(pt1, pt2):
  return pt1[0] < pt2[0]

def is_right_of(pt1, pt2):
  return pt1[0] > pt2[0]

def is_straight(pt1, pt2, CW):
  if pt1[0] == pt2[0]:
    # left/right side
    if pt1[1] == pt2[1]:
      return False
    if CW:
      return is_below(pt1, pt2) if pt1[0] == LEFT else is_above(pt1, pt2)
    else:
      return is_below(pt1, pt2) if pt1[0] == RIGHT else is_above(pt1, pt2)
  elif pt1[1] == pt2[1]:
    # top/bottom side
    if CW:
      return is_left_of(pt1, pt2) if pt1[1] == TOP else is_right_of(pt1, pt2)
    else:
      return is_left_of(pt1, pt2) if pt1[1] == BOTTOM else is_right_of(pt1, pt2)
  return False

def get_patrol_path(start, finish, clockwise=True):
  """
  Generate walking path from given start to finish points on the on grid perimeter.
  Returns waypoints to traverse the path.  Corners are cut, i.e. the traversed
  path is not restricted to the perimeter.
  """

  # if start == finish go all the way around
  force_step = is_equal(start, finish)

  path = [start]
  cur = start
  steps = 0
  while force_step or not is_equal(cur, finish):
    if  is_straight(cur, finish, clockwise):
      # finish is straight ahead on same side
      cur = finish
    elif clockwise:
      if is_bottom(cur):
        cur = [LEFT, NEAR_BOTTOM] if cur[0] <= NEAR_LEFT else [NEAR_LEFT, BOTTOM]
      elif is_top(cur):
        cur = [RIGHT, NEAR_TOP] if cur[0] >= NEAR_RIGHT else [NEAR_RIGHT, TOP]
      elif is_left(cur):
        cur = [NEAR_LEFT, TOP] if cur[1] >= NEAR_TOP else [LEFT, NEAR_TOP]
      else: # is_right
        cur = [NEAR_RIGHT, BOTTOM] if cur[1] <= NEAR_BOTTOM else [RIGHT, NEAR_BOTTOM]
    else: # counter clockwise
      if is_bottom(cur):
        cur = [RIGHT, NEAR_BOTTOM] if cur[0] >= NEAR_RIGHT else [NEAR_RIGHT, BOTTOM]
      elif is_top(cur):
        cur = [LEFT, NEAR_TOP] if cur[0] <= NEAR_LEFT else [NEAR_LEFT, TOP]
      elif is_left(cur):
        cur = [NEAR_LEFT, BOTTOM] if cur[1] <= NEAR_BOTTOM else [LEFT, NEAR_BOTTOM]
      else: # is_right
        cur = [NEAR_RIGHT, TOP] if cur[1] >= NEAR_TOP else [RIGHT, NEAR_TOP]
    path.append(cur)
    force_step = False
    steps += 1
    if steps == 10:
      break

  return path

def recompute_patrol_path(path, current_pos, finish, CW):
  new_path = get_patrol_path(current_pos, finish, CW)
  print('RECOMPUTE', finish, path, CW, new_path)
  new_path.pop(0)
  return new_path

def get_fastest_path(start, finish):
  path1 = get_patrol_path(start, finish, True)
  path2 = get_patrol_path(start, finish, False)
  if len(path1) <= len(path2):
    return path1
  else:
    return path2

def incr_pos(pos, CW):
  if is_equal(pos, GRID_TL):
    return [pos[0]+1, pos[1]] if CW else [pos[0], pos[1]-1]
  elif is_equal(pos, GRID_TR):
    return [pos[0], pos[1]-1] if CW else [pos[0]-1, pos[1]]
  elif is_equal(pos, GRID_BR):
    return [pos[0]-1, pos[1]] if CW else [pos[0], pos[1]+1]
  elif is_equal(pos, GRID_BL):
    return [pos[0], pos[1]+1] if CW else [pos[0]+1, pos[1]]
  elif is_left(pos):
    return [pos[0], pos[1]+1] if CW else [pos[0], pos[1]-1]
  elif is_right(pos):
    return [pos[0], pos[1]-1] if CW else [pos[0], pos[1]+1]
  elif is_top(pos):
    return [pos[0]+1, pos[1]] if CW else [pos[0]-1, pos[1]]
  else:
    return [pos[0]-1, pos[1]] if CW else [pos[0]+1, pos[1]]

def create_patrol_around(col, row):
  a = incr_pos(incr_pos([col, row], True), True)
  b = incr_pos(incr_pos([col, row], False), False)
  return (a, b, True)

if __name__ == '__main__':
  for x in get_patrol_path([2,0], [2,0]):
    print('A', x)
  for x in get_patrol_path([0,0], [0,0], False):
    print('B', x)
  for x in get_patrol_path((0,0), (4,0), False):
    print('C', x)
