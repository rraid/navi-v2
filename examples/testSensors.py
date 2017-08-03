import sys
sys.path.append("../perception")
sys.path.append("../device/")
import devhub
import perception
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2
from matplotlib import cm

def displayDistribution(name, grid):
  plt.imshow(np.flipud(grid) * 255.0, cmap=cm.gray)
  plt.show()

devhub.init()
time.sleep(1)

values = devhub.getZedReadings()

while True:

  values = devhub.getZedReadings()
  values2 = devhub.getLidarReadings()
  grid = perception.getCollisionDistribution(perception.getZedDistribution(values),perception.getLidarDistribution(values2))
  cv2.imshow("dist", np.flipud(grid))
  cv2.waitKey(10)

#displayDistribution("Sonar", grid)
#grid = perception.getLidarDistribution(values)
#displayDistribution("Lidar", grid)
#grid = perception.getZEDDistribution(values)
#displayDistribution("ZED", grid)
