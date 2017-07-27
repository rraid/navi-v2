from threading import Thread
import cv2
from scipy.ndimage.interpolate import rotate

#Actions: N = 0   W = 1   S = 2   E = 3
#         NW = 4  SW = 5  SE = 6  NE = 7
class AStar(Thread):
  rotate = np.array([(0,1),(-1,0),(0,-1),(1,0),(-1,1),(-1,-1),(1,-1),(1,1)])
  angle  = np.array([0, 270, 180, 90, 315, 225, 135, 45])
  
  def __init__(self):
    Thread.__init__(self)
    self.stopstate = False
    self.c_space = None
    self.energyCost = 0.01
    self.nextGoal = None
    self.self = cv2.imread('robot.png')
    # create a variable to store the localizer object by reference
    self.localizer = None
    
  def run(self):
    while not self.stopstate:
      # grab the distribution from the localizer
      distribution = self.localizer.predict()
      # select the most likely position to start planning on
      pose = list(np.argwhere(distribution == np.amax(distribution))[0, :])
      # plan the flow map on that position
      self.computePath(pose)

  def setLocalizer(self, localizer):
    self.localizer = localizer

  def setConstraintSpace(self, pathmap, collisionSpace):
    self.c_space = pathmap + collisionSpace

  def setNextGoal(self, pose):
    self.nextGoal = pose

  def reachedGoal(self, pose):
    if pose[0:2] = nextGoal[0:2]:
      return True
    else:
      return False

  def getValidActions(self, pose):
    actions = np.zeros(8)
    for action in range(8):
      cost = getCost(pose,action,1)
      if cost[0] <.1 and cost[1] <.1:
        actions[action] = 1
    return actions

  def getSuccessors(self, pose, action, dt):
    return (pose[0] + rotation[action], pose[1] + rotation[action], angle[action])

  def getCost(self, pose, action, dt):
    cost = np.zeros(2)
    newPose = getSuccessors(pose,action,dt)
    rotatedSelf = rotate(self.self,newPose[2])
    rotSizex,rotSize_y = rotatedSelf.shape[0]/2,rotatedSelf.shape[1]/2
    
    cost[0] = np.multiply(rotatedSelf, self.c_space[pose[0]-rotSize_x:pose[0]+rotSize_x, pose[1]-rotSize_y:pose[1]+rotSize_y])
    
    cost[1] = np.multiply(rotatedSelf, self.c_space[pose[0]-rotSize_x + rotate[theta,0]:pose[0]+rotSize_x+ rotate[theta,0], pose[1]-rotSize_y+rotate[theta,1]:pose[1]+rotSize_y]+rotate[theta,1])
    return cost

  def distanceCost(self, pose1):
    return numpy.linalg.norm(self.nestGoal[0:2]-pose1[0:2]) * self.energyCost

  def retraceParents(self,node):
    path = np.empty(0)
    while node != None:
      np.append(path,node)
      node = node.parent
    return path

  def computePath(self, pose):
    closedSet = np.empty(0)
    orthogonal = 1
    diagonal = 1.4
    start = Node(0,distanceCost(pose[0:2]),None,pose[0:2])
    openSet = np.array([start])
    while not openSet.empty():
      current = min(openSet, key=lambda a: a.getF())
      if current.position == self.nextGoal[0:2]:
        return retraceParents(current)
        
      x,y = current.position
      isValid = getValidActions((x,y))
      pos = np.add(rotate,(x,y))
      for theta in range(8):
        if isValid(theta) == False:
          continue
          
        if direction < 4:
          travel = orthogonal
        else:
          travel = diagonal
          
        for check in closedSet:
          if pos[theta] == check.position:
            break
        else:
          for check in openSet:
            if pos[theta] == check.position:
              newG = current.g + travel
              if newG > check.g:
                check.g = newG
                check.parent = current
                break    
          else:
            np.append(openSet,Node(distanceCost(pos[theta]),current.g+travel,current,pos[theta]))
            
      np.append(closedSet,current)
      openSet = np.delete(openSet,np.argwhere(openSet==current))
      
    else:
      print "No Path Found"
    
  def stop(self):
    self.stopstate = True
    
    
class Node():
  def __init__(self,g,h,parent,position):
    self.g = g
    self.h = h
    self.parent = parent
    self.position = position
  def getF(self):
    return self.g + self.h
  
