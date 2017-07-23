import numpy as np
from scipy import signal
import math

def displayDistribution(grid):
  import matplotlib.pyplot as plt
  from matplotlib import cm

  plt.imshow(grid, cmap=cm.gray)
  plt.show()


grid = np.zeros((100,100))
grid[50,50] = 1

kernel = np.zeros((65,65))
for i in range(kernel.shape[0]):
	for j in range(kernel.shape[1]):
		kernel[i,j] = math.exp(-1*(((((i-32)**2) + (((j-32)-10)**2))/20)))

displayDistribution(signal.convolve2d(grid,kernel))
