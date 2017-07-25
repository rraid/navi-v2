import perception

def displayDistribution(grid):
  import matplotlib.pyplot as plt
  from matplotlib import cm

  plt.imshow(grid, cmap=cm.gray)
  plt.show()


values = [50,50,50,50,50,50,50,50,50]

grid = perception.getSonarDistribution(values)
#grid = perception.getLidarDistribution(values)
#grid = perception.getZEDDistribution(values)
displayDistribution(grid)



