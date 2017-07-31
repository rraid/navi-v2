import sys
sys.path.append("../perception/")
import perception
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

if __name__ == "__main__":
  plt.imshow(np.flipud(perception.getGPSDistribution((124, 150)) * 255.0),
      cmap=cm.gray)
  plt.show()
