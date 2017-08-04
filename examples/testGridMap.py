import sys
sys.path.append("../perception/")
import perception
import gridmap
sys.path.append("../device/")
import devhub
import pfilter
import cv2
import numpy as np
import time
import math
from scipy.misc import imresize

if __name__ == "__main__":
  devhub.init()
  grid = gridmap.GridMap()
  grid.initializeEmpty((681, 968))

  perception.mapShape = (681,968)
  localizer = pfilter.ParticleFilter()
  localizer.initializeUniformly(5000, (968, 681, 360))
  #gridmap = localizer.predict()
  
  for i in range(400):
    starttime = time.time()
    
    gps = perception.getGPSDistribution(devhub.getGPSReadings())
    print gps.shape
    compass = perception.getCompassDistribution(devhub.getCompassReadings())
    print compass.shape
    collisions = perception.getCollisionDistribution(
      perception.getLidarDistribution(devhub.getLidarReadings()),
      perception.getZedDistribution(devhub.getZedReadings()))
    print collisions.shape
    
    localizer.updatePosition(0.0, 0.0)
    localizer.observePosition(gps, compass, collisions)
    positions = localizer.getDistribution()
    
    print "starting grid mapping"
    grid.updateCollisions(positions, collisions)
    print "time taken:", time.time() - starttime

    gridMap = np.flipud(grid.predict())
    cv2.imshow("grid", gridMap * 255)
    cv2.waitKey(10)
