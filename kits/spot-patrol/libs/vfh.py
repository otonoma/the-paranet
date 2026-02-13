import sys
sys.path.append('./libs')

import math
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from math_util import euler_from_quaternion

#################### utility functions

def norm_angle(a):
    if a > math.pi:
        return math.pi - a
    elif a < -math.pi:
        return -a - math.pi
    return a

def target_angle(position, target):
    err = target - position
    rad = math.atan2(err[1], err[0])
    return norm_angle(rad)

# Calculate the desired correction to rotate from the
# current to a target angle (radians)
# returns +angle for CCW and -angle for CW

def calc_rotation_correction(current, target):
    ccw_dist = target - current if target >= current else target + 2*math.pi - current
    if ccw_dist > math.pi:
      # CW is shorter
      cw_dist = 2*math.pi - ccw_dist
      return -cw_dist
    else:
      return ccw_dist

# return degress as integers in range -180 to 180
def points_to_deg(x,y):
  # numpy arctan2 is ccw from +x axis: 0, 90, 180(-180), -90, 0 (in radians)
  angles = np.arctan2(y, x)
  return np.round(angles/math.pi*180).astype(np.int32)

# add incr to a and keep result in -180 to 180 range
def add_deg(a, incr):
  a += incr
  if a < -180:
    return 180-(-180-a)
  elif a > 180:
    return -180+(a-180)
  return a

def in_arc(a, b, d):
  if b >= a:
    return d >= a and d < b
  else:
    # cross -180
    return in_arc(a, 180, d) or in_arc(-180, b, d)

def arc_len_ccw(a, b):
  if b >= a:
    return b-a
  else:
    # cross -180
    return b+360-a

def deg_left(a, b):
  return arc_len_ccw(a, b)

def deg_right(a, b):
  return arc_len_ccw(b, a)

def deg_diff(a, b):
  return min(arc_len_ccw(a, b), arc_len_ccw(b, a))

def grid_copy(origin_a, grid_a, origin_b, grid_b):
  # find overlap and update
  ha, wa = grid_a.shape
  hb, wb = grid_b.shape
  x1 = max(origin_a[0], origin_b[0])
  x2 = min(origin_a[0]+wa, origin_b[0]+wb)
  y1 = max(origin_a[1], origin_b[1])
  y2 = min(origin_a[1]+ha, origin_b[1]+hb)
  if x2 > x1 and y2 > y1:
    grid_a[y1-origin_a[1]:y2-origin_a[1], x1-origin_a[0]:x2-origin_a[0]] = grid_b[y1-origin_b[1]:y2-origin_b[1], x1-origin_b[0]:x2-origin_b[0]]


#################### OccupancyMap

OCC_SCALE = 0.1
OCC_RES = 100
OCC_THRESH = 2 # meters
OCC_V_MAX = 90 # degrees
OCC_V_MIN = 30 # degrees
MAP_RADIUS = 5

def prepare_cloud(position, cloud):
  position = np.round(position[0:2]/OCC_SCALE).astype(np.int32)
  cloud = np.round(cloud[:,0:2]/OCC_SCALE).astype(np.int32)
  xmin = position[0]-MAP_RADIUS
  xmax = position[0]+MAP_RADIUS
  ymin = position[1]-MAP_RADIUS
  ymax = position[1]+MAP_RADIUS
  if cloud.size > 0:
    xmin = int(min(xmin, np.min(cloud[:,0])))
    xmax = int(max(xmax, np.max(cloud[:,0])))
    ymin = int(min(ymin, np.min(cloud[:,1])))
    ymax = int(max(ymax, np.max(cloud[:,1])))

  origin = np.array([xmin, ymin])
  grid_size = (ymax-ymin+1, xmax-xmin+1)

  position = position - origin
  if cloud.size > 0:
    cloud = (cloud - origin).astype(np.int32)

  return position, cloud, origin, grid_size

class OccupancyMap:
  def __init__(self, target, position, cloud):
    position, cloud, origin, grid_size = prepare_cloud(position, cloud)

    self.origin = origin
    self.position = position
    self.target = np.round(target/OCC_SCALE) - self.origin

    self.grid = np.zeros(grid_size)
    if cloud.size > 0:
      self.grid[cloud[:,1],cloud[:,0]] = 1
    self.d_max = math.ceil(math.sqrt(2*OCC_RES**2))
    self.report = True

  def set_target(self, target):
    self.target = np.round(target/OCC_SCALE) - self.origin

  def update(self, position, cloud):
    position, cloud, origin, grid_size = prepare_cloud(position, cloud)
    self.position = position
    if origin.tolist() == self.origin.tolist():
      resize = np.array(grid_size) - np.array(self.grid.shape)
      if resize[0] < 0:
        self.grid = self.grid[0:grid_size[0],:]
      elif resize[0] > 0:
        self.grid = np.pad(self.grid, ((0,resize[0]),(0,0)))
      if resize[1] < 0:
        self.grid = self.grid[:,0:grid_size[1]]
      elif resize[1] > 0:
        self.grid = np.pad(self.grid, ((0,0),(0,resize[1])))
      self.grid[cloud[:,1],cloud[:,0]] = 1
      return

    # target is relative, translate to new origin
    self.target = self.origin + self.target - origin

    grid = np.zeros(grid_size)
    grid_copy(origin, grid, self.origin, self.grid)
    grid[cloud[:,1],cloud[:,0]] = 1
    self.origin = origin
    self.grid = grid

  def select(self):
    target = self.target - self.position
    target_dir = points_to_deg([target[0]], [target[1]])[0]
    #print(self.position, 'to', self.target, target_dir)
    locs = np.where(self.grid > 0)
    locs = np.stack([locs[1],locs[0]],axis=-1) - self.position
    dist = np.sqrt(locs[:,0]**2+locs[:,1]**2)
    mag = -(self.d_max-dist)
    hist = np.zeros((361,))
    ai = 180+points_to_deg(locs[:,0], locs[:,1])
    #print(ai-180)
    hist[ai] = mag
    win = 5
    boundary = (win-1)//2
    low = -180+boundary
    high = 180-boundary
    view = sliding_window_view(hist, win)
    hist = np.round(np.min(view, axis=1))
    clear = hist > (-self.d_max+OCC_THRESH/OCC_SCALE)
    if np.all(clear):
      return [target_dir, 0, 360, 'a', target_dir, target_dir, target_dir, '']

    valleys = []
    valley = False
    for i in range(len(clear)):
      if clear[i]:
        if not valley:
          start = low+i
          d = hist[i]
          valley = True
        else:
          d = min(d, hist[i])
      else:
        if valley:
          valleys.append([start,low+i,d])
          valley = False
    if valley:
      valleys.append([start,low+len(clear)-1,d])
    if len(valleys) >= 2:
      if valleys[0][0] == low and valleys[-1][1] == high:
        valleys[0][0] = valleys[-1][0]
        valleys[0][2] = min(valleys[0][2],valleys[-1][2])
        valleys.pop()
    valleys = [v for v in valleys if arc_len_ccw(v[0],v[1]) >= OCC_V_MIN]
    #print('V',valleys)

    best = [add_deg(target_dir, 179), 180]
    for v in valleys:
      [a,b,_] = v
      gap = arc_len_ccw(a, b)
      if gap < OCC_V_MIN:
        continue

      unobstructed = in_arc(a,b,target_dir)
      if gap >= OCC_V_MAX:
        d_a = deg_diff(a, target_dir)
        d_b = deg_diff(b, target_dir)
        if d_a < d_b:
          if unobstructed and d_a > OCC_V_MAX//2:
            # target dir is more than OCC_V_MAX//2 in either direction, so direct at it
            new_dir = target_dir
          else:
            new_dir = add_deg(a, OCC_V_MAX//2)
          sel = 'a'
        else:
          if unobstructed and d_b > OCC_V_MAX//2:
            # target dir is more than OCC_V_MAX//2 in either direction, so direct at it
            new_dir = target_dir
          else:
            new_dir = add_deg(b, -OCC_V_MAX//2)
          sel = 'b'
      else:
        new_dir = add_deg(a, gap//2)
        sel = 'm'

      if unobstructed:
        divert = ''
      elif deg_left(new_dir, target_dir) < deg_right(new_dir, target_dir):
        divert = 'R'
      else:
        divert = 'L'

      dist = deg_diff(new_dir, target_dir)
      if dist < best[1]:
        best = [new_dir, dist, gap, sel, a, b, target_dir, divert]
    
    if self.report and best[0]!=target_dir:
      self.dump(scale=2)
      print(self.position, self.target, best)
      self.report=False

    return best
  
  def dump(self, scale=4):
    padding = (np.ceil(np.array(self.grid.shape) / scale) * scale - np.array(self.grid.shape)).astype(np.int32)
    M = np.pad(self.grid.astype(np.int32), ((0, padding[0]),(0, padding[1])))
    M = M.reshape((-1, scale, M.shape[1]//scale, scale))
    M = M.transpose((0,2,1,3))
    M = np.add.reduce(M.reshape((M.shape[0],M.shape[1],scale*scale)), axis=-1)
    loc = np.floor(np.array(self.position) / scale).astype(np.int32)
    tgt = np.floor(np.array(self.target) / scale).astype(np.int32)
    M[loc[1],loc[0]] = 99;
    if tgt[1] >= 0 and tgt[1] < M.shape[0] and tgt[0] >= 0 and tgt[0] < M.shape[1]:
      M[tgt[1],tgt[0]] = -1;
    print('ORIGIN', self.origin)
    print(np.flipud(M))


#################### VFH

ROT_ERR_THRESHOLD = math.radians(3)
WALK_ROT_THRESHOLD = math.radians(5)
TARGET_DIST_THRESHOLD = 0.3

class VFH:
  def __init__(self, robot, quaternion=True):
    self.robot = robot
    self.done = True
    self.map = None
    self.quaternion = quaternion
    self.rotating = False
    self.stopping = False

  def clear(self):
    self.map = None
    self.rotating = False
    self.stopping = False
    self.done = True

  def stop(self):
    self.stopping = True

  def set_target(self, target):
    self.target = target
    if self.map is not None:
      self.map.set_target(target)
    self.done = False
    self.iter = 0

  def update_cloud(self, cloud):
    position, _ = self.robot.get_world_pose()
    if self.map is not None:
       self.map.update(position, cloud)
    else:
       self.map = OccupancyMap(self.target, position, cloud)

  def update_position(self):
    position, _ = self.robot.get_world_pose()
    if self.map is not None:
      self.map.set_position(position)

  def get_command(self, debug=False):
    self.iter += 1
    cmd = np.zeros((3,))

    goto = self.target
    pos, rot = self.robot.get_world_pose()
    if self.quaternion:
      rot = euler_from_quaternion(rot)

    target_dist = np.linalg.norm(goto[:2] - pos[0:2])
    if target_dist < TARGET_DIST_THRESHOLD:
      self.done = True
      return cmd

    sel = self.map.select()
    if debug:
      print('SELECT', sel)

    # check rotation
    want = sel[0]/180*math.pi
    rot_err = calc_rotation_correction(rot[2], want)

    if abs(rot_err) > ROT_ERR_THRESHOLD:
        # left = 2, right = -2
        cmd[2] = 2 if rot_err > 0 else -2
        if abs(rot_err) > WALK_ROT_THRESHOLD:
          self.rotating = True
          return cmd
    self.rotating = False

    if self.stopping:
      self.done = True
      return cmd
  
    cmd[0] = 2
    return cmd

class TestRobot:
  def __init__(self,position):
    self.position = position
    self.rotation = np.array([0.,0.,0.])

  def get_world_pose(self):
    return self.position, self.rotation

  def apply(self, cmd, dy=0.1):
    if cmd[0] >= 1:
      print('APPLY', cmd)
      rad = self.rotation[2]
      v = np.array([math.cos(rad),math.sin(rad)]) * dy
      self.position += v
    if cmd[2] >= 1:
      self.rotation[2] += math.radians(1)
    elif cmd[2] <= -1:
      self.rotation[2] -= math.radians(1)

  def dump(self):
    print(self.position, math.degrees(self.rotation[2]))

if __name__ == '__main__':
  cloud = np.array([[i/6, 0.5+i/30, 0.5] for i in range(7)])
  me = TestRobot(np.array([-1.,-1.]))
  nav = VFH(me,quaternion=False)
  nav.set_target(np.array([1., 1.5]))
  nav.update_cloud(cloud)
  me.dump()
  for i in range(50):
    me.apply(nav.get_command())
    while nav.rotating:
      me.apply(nav.get_command())
    me.dump()
    if nav.done:
      break
    nav.update_cloud(cloud)
    if (i % 5) == 0:
      print('ITER',i)
      nav.map.dump(scale=2)
  nav.map.dump(scale=2)
