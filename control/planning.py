from threading import Thread
import cv2

#Actions: stay = 0  moveForware = 1 turn to-  E=2  NE=3  N=4  NW=5  W=6 SW=7  S=8 SE=9

class AStar(Thread):

  def __init__(self):
    Thread.__init__(self)
    self.stopstate = False
    self.c_space = None
    self.energyCost = 0.01
    self.self = cv2.imread('robot.png')
    
  def run(self):
    while not self.stopstate:
      

  def setConstraintSpace(self, pathmap, collisionSpace):
    self.c_space = pathmap + collisionSpace

  def setNextGoal(self, pose):
    global nextGoal
    nextGoal = pose

  def reachedGoal(self, pose):
    if pose[0:2] = nextGoal[0:2]:
      return True
    else:
      return False

  def getValidActions(self, pose):
    actions = np.array([0])
    forward = np.multiply(self.self, self.c_space[pose[0]-(self.self.shape[0]/2):pose[0]+(self.self.shape[0]/2),pose[1]-(rotatedSelf.shape[1]/2) + 1:pose[1]+(rotatedSelf.shape[1]/2) + 1])
    if forward <= .1:
      actions = np.append(actions,[1])
    for theta in range(8):
      rotatedSelf = rotate(self.self,theta * 45)
      rotation= np.multiply(rotatedSelf, self.c_space[pose[0]-(rotatedSelf.shape[0]/2):pose[0]+(rotatedSelf.shape[0]/2), pose[1]-(rotatedSelf.shape[1]/2):pose[1]+(rotatedSelf.shape[1]/2)])
      if rotation <= .1:
        actions = np.append(actions,theta)
    return actions

  def getSuccessors(self, pose, action, dt):
    pass

  def getCost(self, pose, action, dt):
    pass

  def distanceCost(self, pose1, pose2):
    return numpy.linalg.norm(pose2[0:2]-pose1[0:2]) * self.energyCost

  def retraceParents(self, startingPose, goalPose, parents):
    # Note: can use this as a helper function for your A*
    pass

  def computePath(self, pose):
    pass
  def stop(self):
    self.stopstate = True
