import sys
sys.path.append("../perception/")
import localizer
import cv2
import numpy as np
import time
from scipy.signal import correlate, correlate2d, convolve, convolve2d

def gauss(cov, size=15):
  X = np.arange(0, size, 1.0) - (size-1) / 2.0
  Z = np.exp(-1.0/(2.0 * cov) * (np.multiply(X, X)))
  return Z / np.sum(Z)

def gauss2d(cov, size=15):
  X = np.arange(0, size, 1.0) - (size-1) / 2.0
  Y = np.arange(0, size, 1.0) - (size-1) / 2.0
  X, Y = np.meshgrid(X, Y)
  Z = np.exp(-1.0/(2.0 * cov) * (np.multiply(X, X) + np.multiply(Y, Y)))
  return Z / np.sum(Z)

if __name__ == "__main__":
  gridmap = np.zeros((550, 550), dtype=np.float32)
  gridmap[100, 200] = 1.0
  gridmap[280, 170] = 1.0

  localize = localizer.Localizer()
  localize.initializeUniformly(gridmap.shape)
  #localize.start()

  robotPos = [50.0, 25.0]
  gpsnoise = gauss2d(2.0, 9)
  compassnoise = gauss(1.0, 7)

  print("Showing window")
  cv2.imshow("window", np.flipud(np.sum(localize.predict(), axis=2)))
  cv2.waitKey(30)

  for i in range(200):
    print("starting iteration", i)
    start = time.time()
    localize.setSpeeds(1.0, 1.0)
    gps = np.zeros((gridmap.shape))
    gps[int(robotPos[1] * 2), int(robotPos[0] * 2)] = 1.0
    gps = correlate2d(gps, gpsnoise, mode="same")
    compass = np.zeros((36, ))
    compass[9] = 1.0
    compass = convolve(compass, compassnoise, mode="same")
    collisions = np.zeros((49, 49))
    localize.setObservation(gps, compass, collisions)

    localize.updatePosition(1.0, 1.0)
    print("update:", time.time() - start)
    localize.observePosition(gps, compass, collisions, gridmap)
    print("observe:", time.time() - start)

    print("Showing window")
    cv2.imshow("window", np.flipud(np.sum(localize.predict(), axis=2)))
    cv2.waitKey(30)
    #time.sleep(1.0)
    robotPos[0] += 2.0
    robotPos[1] += 2.0

  #localize.stop()
