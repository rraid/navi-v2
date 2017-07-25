class AStar:
  def __init__(self):
    pass

  def setConstraintSpace(self, pathmap, collisionSpace):
    pass

  def setNextGoal(self, pose):
    pass

  def reachedGoal(self, pose):
    pass

  def getValidActions(self, pose):
    pass

  def getSuccessors(self, pose, action, dt):
    pass

  def getCost(self, pose, action, dt):
    pass

  def distanceCost(self, pose1, pose2):
    pass

  def retraceParents(self, startingPose, goalPose, parents):
    # Note: can use this as a helper function for your A*
    pass

  def computePath(self, pose):
    pass
