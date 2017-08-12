import sys
sys.path.append("../control")
import planning
import numpy as np
import cv2
import time
from scipy.misc import imresize

import matplotlib.pyplot as plt

if __name__ == "__main__":
  world = np.flipud(cv2.imread("../perception/pathmap_scaled.png", \
      cv2.IMREAD_GRAYSCALE) > 128).astype(np.float32)

  """
  world = np.flipud(1 - np.array([
    [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ],
    [ 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1 ],
    [ 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1 ],
    [ 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1 ],
    [ 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1 ],
    [ 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1 ],
    [ 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1 ],
    [ 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1 ],
    [ 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1 ],
    [ 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1 ],
    [ 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 ],
    [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ]],
    dtype=np.float32))
  """
  #world = imresize(world, (320, 320), "nearest").astype(np.float32) / 255.0
  #print world

  planner = planning.AStar(world)
  start = (130,51)
  goal = (244,190)
  #start = (1, 1)
  #goal = (18, 18)
  #start = (16, 16)
  #goal = (288, 288)
  planner.setNextGoal(goal)
  planner.setConstraintSpace(np.zeros(world.shape))

  world = cv2.cvtColor(world, cv2.COLOR_GRAY2BGR)

  starttime = time.time()
  planner.computePath(start)
  print("Time taken to find path:" + str(time.time() - starttime))

  #X, Y = np.meshgrid(np.arange(0, 20, 1), np.arange(0, 20, 1))
  #Q = plt.quiver(X, Y, np.flipud(planner.xPotential), -np.flipud(planner.yPotential), sc#ale=1/0.015)
  #plt.show()

  path = [start]
  curr = start
  with open("path.txt", "w") as fp:
    while not planner.reachedGoal(curr):
      world[curr[1], curr[0]] = [0, 0, 1]
      curr = planner.getNextState(curr)
      path.append(curr)
      fp.write(str(curr) + "\n")
    #cv2.imshow("world", world)
    #cv2.waitKey(30)
  world[curr[1], curr[0]] = [0, 0, 1]
  #cv2.imshow("world", imresize(np.flipud(world), (400, 400), "nearest"))
  cv2.imshow("world", np.flipud(world))
  cv2.waitKey(0)
