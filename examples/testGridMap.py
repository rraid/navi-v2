import sys
sys.path.append("../perception/")
import gridmap
import cv2
import numpy as np
import time
import math
from scipy.misc import imresize

if __name__ == "__main__":
  grid = gridmap.GridMap()
  grid.initializeEmpty((275, 275))

  positions = np.zeros((550, 550, 36))
  positions[25, 50, -9] = 0.5
  positions[50, 50, -8] = 0.5
  positions[75, 50, -7] = 0.5
  positions[100, 50, -6] = 0.5
  positions[125, 50, -5] = 0.5
  positions[150, 50, -4] = 0.5
  positions[175, 50, -3] = 0.5
  positions[200, 50, -2] = 0.5
  positions[225, 50, -1] = 0.5
  positions[250, 50, 0] = 0.5

  collisions = np.zeros((49, 49))
  angles = np.linspace(-55, 55, 1000) + 90
  offx = 24
  offy = 24
  for i in range(1000):
    radius = max(0, i - 750) * 0.05 + 10
    x = int(math.cos(math.radians(angles[i])) * radius) + offx
    y = int(math.sin(math.radians(angles[i])) * radius) + offy
    collisions[y, x] = 1.0

  cv2.imshow("collisions", imresize(np.flipud(collisions), (300, 300), "nearest"))

  for i in range(5):
    starttime = time.time()
    restrict_range = [[0, 120], [0, 120], [0, 36]]
    grid.updateCollisions(positions, collisions, restrict_range)
    print "time taken:", time.time() - starttime

  gridMap = np.flipud(grid.predict())
  cv2.imshow("grid", gridMap * 255)
  cv2.waitKey(0)
