import sys
sys.path.append("../perception/")
sys.path.append("../device/")
import perception
import devhub
import numpy as np
import cv2
if __name__ == "__main__":
  devhub.init()
  while True:
    position = devhub.getGPSReadings()
    gps = perception.getGPSDistribution(position)
    cv2.imshow("dsaf;lk",np.flipud(gps)*255)
    cv2.waitKey(10)
