import sys
sys.path.append("../perception/")
sys.path.append("../device/")
import perception
import devhub
import numpy as np
import cv2
if __name__ == "__main__":
  devhub.init()
  grid = cv2.imread("../perception/pathmap_scaled.png", cv2.IMREAD_GRAYSCALE) / 2
  grid = np.array([np.copy(grid), np.zeros(grid.shape), np.zeros(grid.shape)]) 
  grid = np.rollaxis(grid, -1)
  grid = np.rollaxis(grid, -1)
  while True:
    position = devhub.getGPSReadings()
    print position
    gps = perception.getGPSDistribution(position)
    gps = np.rollaxis(np.array([gps, gps, gps]), -1)
    gps = np.rollaxis(gps, -1)
    gps = gps + np.flipud(grid)
    print gps.shape
    cv2.imshow("dsaf;lk",np.flipud(gps)*255)
    cv2.waitKey(10)
