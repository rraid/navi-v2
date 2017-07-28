import sys
sys.path.append("../perception")
import perception
import numpy as np

def displayDistribution(name, grid):
  import matplotlib.pyplot as plt
  from matplotlib import cm

  plt.imshow(np.flipud(grid) * 255.0, cmap=cm.gray)
  plt.show()


values = [50,50,50,80,50,50,50,50,50]

grid = perception.getSonarDistribution(values)
displayDistribution("Sonar", grid)
#grid = perception.getLidarDistribution(values)
#displayDistribution("Lidar", grid)
#grid = perception.getZEDDistribution(values)
#displayDistribution("ZED", grid)
