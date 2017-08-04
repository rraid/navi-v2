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

while True:
  values = devhub.getCompassReadings()
  distribution = perception.getCompassDistribution(values)
  distribution = np.repeat(distribution, 10, axis=0)
  plot = np.repeat(np.reshape(np.arange(0.0, 1.0, 0.01) + 0.01, (100, 1)), 360, axis=1)
  plot = (plot <= distribution) * 1.0
  cv2.imshow("plot", np.flipud(plot))
  cv2.waitKey(10)
