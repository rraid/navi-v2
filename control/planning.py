from threading import Thread
import cv2
from scipy.ndimage.interpolate import rotate

#Actions: N = 0   W = 1   S = 2   E = 3
#         NW = 4  SW = 5  SE = 6  NE = 7
class AStar(Thread):
  rotate = np.array([(0,1),(-1,0),(0,-1),(1,0),(-1,1),(-1,-1),(1,-1),(1,1)])
  angle  = np.array([math.atan2(p[1], p[0]) * 180 / math.pi for p in rotate])
  
  def __init__(self, pathmap):
    Thread.__init__(self)
    self.stopstate = False
    self.c_space = None
    self.energyCost = 0.01
    self.nextGoal = None
    #self.self = cv2.imread('robot.png')
    self.xPotential = np.zeros(pathmap.shape[:2]) # flow map
    self.yPotential = np.zeros(pathmap.shape[:2])
    self.pathmap = pathmap

  def setNextGoal(self, pose):
    self.nextGoal = pose

  def reachedGoal(self, pose):
    return np.array_equal(pose[0:2], self.nextGoal[0:2])

  def getValidActions(self, pose, c_space):
    actions = np.zeros(8)
    for action in range(8):
      cost = getCost(pose,action,1,c_space)
      if cost < 0.5:
        actions[action] = 1
    return actions

  def getSuccessors(self, pose, action, dt):
    return (pose[0] + rotation[action][0], \
            pose[1] + rotation[action][1], \
            angle[action])

  def getCost(self, pose, action, dt, c_space):
    #cost = np.zeros(2)
    #newPose = getSuccessors(pose,action,dt)
    #rotatedSelf = rotate(self.self,newPose[2])
    #rotSizex,rotSize_y = rotatedSelf.shape[0]/2,rotatedSelf.shape[1]/2
    
    #cost[0] = np.multiply(rotatedSelf, self.c_space[pose[0]-rotSize_x:pose[0]+rotSize_x, pose[1]-rotSize_y:pose[1]+rotSize_y])
    
    #cost[1] = np.multiply(rotatedSelf, self.c_space[pose[0]-rotSize_x + rotate[theta,0]:pose[0]+rotSize_x+ rotate[theta,0], pose[1]-rotSize_y+rotate[theta,1]:pose[1]+rotSize_y]+rotate[theta,1])

    # make it simple in order to speed things up, do not account for theta :(
    x = pose[0] + rotate[action][0]
    y = pose[1] + rotate[action][1]
    return c_space[y, x] * self.energy_cost

  def distanceCost(self, pose1, pose2):
    return np.linalg.norm(pose2[0:2]-pose1[0:2]) * self.energyCost

  def retraceParents(self,node):
    path = np.empty(0)
    while node != None:
      np.append(path,node)
      node = node.parent
    return path

  def computePath(self, pose):
    closed = np.zeros(self.c_space.shape, dtype=np.uint8)
    opened = []
    gScore = np.zeros(self.c_space.shape, dtype=np.float32)
    fScore = np.zeros(self.c_shape.shape, dtype=np.float32)
    xPotential = np.zeros(self.pathmap.shape[:2], dtype=np.int8)
    yPotential = np.zeros(self.pathmap.shape[:2], dtype=np.int8)
    c_space = np.copy(self.c_space)

    opened.append((self.nextGoal[:2], 0))
    while len(opened) > 0:
      # this can be parallelizable on a GPU, instead of 1 do k kernel calls
      minitem = min(opened, key=lambda s: s[1])
      opened.remove(minitem)
      current = minitem[0]
      #if np.array_equal(current, pose[:2]):
      #  return retraceParents(current)
      closed[current[1], current[0]] = 1
      validActions = self.getValidActions(pose, c_space)
      for action in range(len(validActions)):
        if validActions[action] != 1:
          continue
        successor = getSuccessors(current, action, 1)[:2]
        if closed[successor[1], successor[0]] == 1:
          continue
        tentative_gScore = gScore[current[1], current[0]] + \
            self.getCost(current, successor)
        #tentative_fScore = tentative_gScore + \
        #    self.distanceCost(successor, pose[:2])
        tentative_fScore = tentative_gScore
        if sum([np.array_equal(successor, n) for n in opened]) == 0:
          opened.append((successor, tentative_fScore))
        elif tentative_gScore >= gScore[successor[1], successor[0]]:
          continue
        xPotential = current[0] - successor[0]
        yPotential = current[1] - successor[1]
        gScore[successor[1], successor[0]] = tentative_gScore
        fScore[successor[1], successor[0]] = tentative_fScore

    self.xPotential = xPotential
    self.yPotential = yPotential

  def getNextState(pose):
    return (xPotential[pose[0]] + pose[0], yPotential[pose[1]] + pose[1]])

  def setLocalizer(self, localizer):
    self.localizer = localizer

  def setConstraintSpace(self, collisionSpace):
    # hack for the planner to plan faster
    robotMask = np.ones((3, 3))
    collisionSpace = convolve2d(collisionSpace, robotMask, mode="same")
    self.c_space = pathmap + collisionSpace
    # set up walls
    self.c_space[:,0] = 1.0
    self.c_space[:,self.c_space.shape[1]-1] = 1.0
    self.c_space[0,:] = 1.0
    self.c_space[self.c_space.shape[0]-1,:] = 1.0

  def run(self):
    lastTime = time.time()
    while not self.stopstate:
      currTime = time.time()
      # grab the distribution from the localizer
      #distribution = self.localizer.predict()
      # select the most likely position to start planning on
      #pose = list(np.argwhere(distribution == np.amax(distribution))[0, :])
      # plan the flow map on that position
      #self.computePath(pose)
      self.computePath(np.array([0,0,0]))
      print "[PLANNING] process time:", time.time() - currTime

  def stop(self):
    self.stopstate = True
