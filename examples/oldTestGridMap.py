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

  positions = [\
  [25, 50, -90],
  [50, 50, -80],
  [75, 50, -70],
  [100, 50, -60],
  [125, 50, -50],
  [150, 50, -40],
  [175, 50, -30],
  [200, 50, -20],
  [225, 50, -10],
  [250, 50, 0]]
  #positions = list(np.matlib.repmat(positions, 500, 1))

  collisions = np.zeros((49, 49))
  angles = np.linspace(-55, 55, 1000) + 90
  offx = 24
  offy = 24
  for i in range(1000):
    radius = max(0, i - 750) * 0.05 + 10
    x = int(math.cos(math.radians(angles[i])) * radius) + offx
    y = int(math.sin(math.radians(angles[i])) * radius) + offy
    collisions[y, x] = 1.0

  collisions = imresize(collisions, (600, 600), "nearest")
  collisions /= np.amax(collisions)

  cv2.imshow("collisions", imresize(np.flipud(collisions * 255), (300, 300), "nearest"))
  cv2.waitKey(0)

  for i in range(5):
    starttime = time.time()
    grid.updateCollisions(positions, collisions)
    print "time taken:", time.time() - starttime

  gridMap = np.flipud(grid.predict())
  cv2.imshow("grid", gridMap * 255)
  cv2.waitKey(0)
