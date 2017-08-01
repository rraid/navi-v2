import math
import numpy as np
import cv2
import time
from threading import Thread
import mxnet as mx

def rotateImage(image, angle):
  image_center = tuple(np.array(image.shape)/2)
  rot_mat = cv2.getRotationMatrix2D(image_center,-angle,1.0) # negative due to flipud
  result = cv2.warpAffine(image, rot_mat, image.shape,flags=cv2.INTER_CUBIC)
  return result

class GridMap(Thread):
  def __init__(self):
    Thread.__init__(self)
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
    self.lastTime = time.time()

  def initializeEmpty(self, shape):
    self.x_meters = shape[1]
    self.y_meters = shape[0]
    self.grid = np.zeros((int(self.y_meters / self.y_inc),
                          int(self.x_meters / self.x_inc)), dtype=np.float32)

  def updateCollisions(self, positions, collisions):
    assert type(self.grid) != type(None)

    # convolve the collisions with the positions and add it in
    obstacles = mx.nd.zeros((1, 1, positions.shape[0], positions.shape[1]),
        ctx=mx.gpu(0))
    collisions = np.flipud(np.fliplr(collisions))
    update = np.zeros((36, collisions.shape[0], collisions.shape[1]))
    for theta in range(positions.shape[2]):
      update[theta,:,:] = rotateImage(collisions, theta * 10)
    location = mx.nd.array(np.rollaxis(positions, -1), ctx=mx.gpu(0))\
        .reshape((1, positions.shape[2], positions.shape[0], positions.shape[1]))
    update = mx.nd.array(update, ctx=mx.gpu(0))\
        .reshape((1, update.shape[0], update.shape[1], update.shape[2]))
    for i in range(location.shape[1]):
      obstacles += mx.nd.Convolution(num_filter=1, no_bias=True,
          data=mx.nd.slice_axis(location, axis=1, begin=i, end=i+1),
          weight=mx.nd.slice_axis(update, axis=1, begin=i, end=i+1),
          kernel=collisions.shape,
          pad=(collisions.shape[0]/2, collisions.shape[1]/2))

    # add collisions
    # probability decreases with every call (depends on a regular call interval)
    print positions.shape
    self.grid = np.clip(self.grid * 0.8 +
        obstacles.reshape(positions.shape[:2]).asnumpy(), 0.0, 1.0)

  def setPositions(self, positions):
    self.positions = positions

  def setCollisions(self, collisions):
    self.collisions = collisions

  def predict(self):
    return self.grid

  def run(self):
    while not self.stopstate:
      currTime = time.time()
      if currTime - self.lastTime < 0.1:
        continue
      self.lastTime = currTime
      if type(self.positions) != type(None) and len(self.positions) != 0 and \
         type(self.collisions) != type(None) and len(self.collisions) != 0:
        self.updateCollisions(np.copy(self.positions), np.copy(self.collisions))
        print("[GRIDMAP] process time:", time.time() - currTime)

  def stop(self):
    self.stopstate = True
