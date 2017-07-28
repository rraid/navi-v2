import sys
sys.path.append("../perception/")
import perception
import signal
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
  distribution = perception.getCompassDistribution(0)
  x = np.arange(len(distribution))
  plt.bar(x, distribution)
  plt.ylim([0.0, 1.0])
  plt.show()
