import math
import numpy as np
from scipy.signal import convolve2d, correlate2d
from scipy.ndimage.interpolate import rotate
import cv2
from threading import Thread

class GridMap(Thread):
  def __init__(self):
    self.x_inc = 0.5
    self.x_meters = 0
    self.y_inc = 0.5
    self.y_meters = 0
    self.theta_inc = 10
    self.n_theta = 36
    self.grid = None
    self.stopstate = False
    self.collisions = None
    self.positions = None

  def initializeEmpty(self, shape):
    self.x_meters = shape[1]
    self.y_meters = shape[0]
    self.grid = np.zeros((int(self.y_meters / self.y_inc),
                          int(self.x_meters / self.x_inc)), dtype=np.float32)

  def updateCollisions(self, positions, collisions):
    assert type(self.grid) != type(None)
    # probability decreases with every call (depends on a regular call interval)
    self.grid *= 0.8

    # convolve the collisions with the positions and add it in
    assert(np.sum(positions) == 1.0) # be careful! super slow
    obstacles = np.zeros((positions.shape[0], positions.shape[1]))
    for i in range(positionDistribution.shape[2]):
      obstacles += convolve2d(positions[:,:,i], rotate(collisions, 10 * i))

    # add collisions
    self.grid = np.clip(self.grid + obstacles, 0.0, 1.0)

  def setPositions(self, positions):
    self.positions = positions

  def setCollisions(self, collisions):
    self.collisions = collisions

  def predict(self):
    return np.copy(self.grid)

  def run(self):
    lastTime = time.time()
    while not self.stopstate:
      currTime = time.time()
      if currTime - lastTime < 0.05:
        pass
      lastTime = currTime
      if type(self.positions) != type(None) and len(self.positions) != 0 and \
         type(self.collisions) != type(None) and len(self.collisions) != 0:
        self.updateCollisions(np.copy(self.positions), np.copy(self.collisions))
        print "[GRIDMAP] process time:", time.time() - lastTime

  def stop(self):
    self.stopstate = True
