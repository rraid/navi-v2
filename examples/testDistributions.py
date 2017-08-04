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
perception.mapShape = (681, 968)
values = devhub.getZedReadings()

while True:
  values = np.array(devhub.getGPSReadings())
  #values = np.array([-74.458470, 40.522266])
  
  print "new values after scale (image size): ", values
  values = perception.getGPSDistribution(values)
  print np.amax(values)
  #values2 = devhub.getLidarReadings()
  #grid = perception.getCollisionDistribution(perception.getZedDistribution#(values),perception.getLidarDistribution(values2))
  cv2.imshow("dist", np.flipud(values>0) * 10.0)
  cv2.waitKey(10)

#displayDistribution("Sonar", grid)
#grid = perception.getLidarDistribution(values)
#displayDistribution("Lidar", grid)
#grid = perception.getZEDDistribution(values)
#displayDistribution("ZED", grid)
