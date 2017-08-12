import sys
sys.path.append("../perception")
sys.path.append("../device/")
import devhub
import perception
import pfilter
import numpy as np
import cv2
import time

[-74.462292, 40.521202]
[-74

if __name__ == "__main__":
  devhub.init()
  localizer = pfilter.ParticleFilter()
  localizer.initializeUniformly(5000, (968,681, 360))

  gridmap = localizer.predict()
  cv2.imshow("grid", np.flipud(gridmap))
  cv2.waitKey(0)

  #robotPos = np.array([100.0, 100.0])
  #perception.mapShape = (550, 550)
  #perception.mapShape = [681,968]
  starttime = time.time()
  
  
  while devhub.getCompassReadings() == None:
    print "Compass is None"
    time.sleep(1)
    
  for i in range(80):
    print "LOOPING"
    currtime = time.time()
    #robotPos += np.array([0.0, 0.0])
    starttime = currtime

    gps = perception.getGPSDistribution(devhub.getGPSReadings())
    compass = perception.getCompassDistribution(devhub.getCompassReadings())
    collisions = perception.getCollisionDistribution(
      perception.getLidarDistribution(devhub.getLidarReadings()),
      perception.getZedDistribution(devhub.getZedReadings()))

    localizer.updatePosition(0.0, 0.0)
    print("Time taken to update:", time.time() - currtime)

    gridmap = localizer.predict()
    cv2.imshow("grid", np.flipud(gridmap))
    cv2.waitKey(30)

    currtime = time.time()
    localizer.observePosition(gps, compass, collisions)
    print("Time taken to observe:", time.time() - currtime)

    gridmap = localizer.predict()
    cv2.imshow("grid", np.flipud(gridmap))
    cv2.waitKey(30)
