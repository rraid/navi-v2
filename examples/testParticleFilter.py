import sys
sys.path.append("../perception/")
import perception
import pfilter
import numpy as np
import cv2
import time

if __name__ == "__main__":
  localizer = pfilter.ParticleFilter()
  localizer.initializeUniformly(5000, (550, 550, 360))

  gridmap = localizer.predict()
  cv2.imshow("grid", np.flipud(gridmap))
  cv2.waitKey(0)

  robotPos = np.array([100.0, 100.0])
  perception.mapShape = (550, 550)

  starttime = time.time()
  for i in range(40):
    currtime = time.time()
    robotPos += np.array([1.0, 1.0])
    starttime = currtime

    gps = perception.getGPSDistribution(robotPos)
    compass = perception.getCompassDistribution(-45.0)
    collisions = np.zeros((49, 49))

    localizer.updatePosition(1.0, 1.0)
    print("Time taken to update:", time.time() - currtime)

    gridmap = localizer.predict()
    cv2.imshow("grid", np.flipud(gridmap))
    cv2.waitKey(30)

    currtime = time.time()
    localizer.observePosition(gps, compass, collisions, np.zeros((550, 550)))
    print("Time taken to observe:", time.time() - currtime)

    gridmap = localizer.predict()
    cv2.imshow("grid", np.flipud(gridmap))
    cv2.waitKey(30)
