import math
import numpy as np
import cv2
import time
from threading import Thread
import mxnet as mx
from scipy.misc import imresize

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
    self.grid = np.zeros((int(self.y_meters),
                          int(self.x_meters)), dtype=np.float32)

  def updateCollisions(self, positions, collisions, restrict_range=None, \
      particles=True):
    assert type(self.grid) != type(None)

    if not particles:
      theta0 = 0
      theta1 = 36
      top = 0
      bottom = self.grid.shape[0]
      left = 0
      right = self.grid.shape[1]

      if restrict_range != None: # for speedup
        positions = positions[ \
            restrict_range[0][0]:restrict_range[0][1], \
            restrict_range[1][0]:restrict_range[1][1], \
            restrict_range[2][0]:restrict_range[2][1]]
        left   = restrict_range[0][0]
        right  = restrict_range[0][1]
        top    = restrict_range[1][0]
        bottom = restrict_range[1][1]
        theta0 = restrict_range[2][0]
        theta1 = restrict_range[2][1]

      # convolve the collisions with the positions and add it in
      obstacles = mx.nd.zeros((1, 1, positions.shape[0], positions.shape[1]),
          ctx=mx.gpu(0))
      collisions = np.flipud(np.fliplr(collisions))
      update = np.zeros((positions.shape[2], \
          collisions.shape[0], collisions.shape[1]))
      for theta in range(theta0, theta1):
        update[theta-theta0,:,:] = rotateImage(collisions, theta * 10)
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
      self.grid[top:bottom,left:right] = \
          np.clip(self.grid[top:bottom,left:right] * 0.8 +
          obstacles.reshape(positions.shape[:2]).asnumpy(), 0.0, 1.0)

    else:
      #eta = 1.0 / len(positions) # normalization constant
      # divided by 5 to scale correctly from 0.1m to 0.5m
      C = np.clip(collisions, 0.0, 1.0)
      C = imresize(C, (C.shape[0] / 5, C.shape[1] / 5), "bicubic")
      C /= np.amax(C)
      pts = np.argwhere(C > 0.25) - np.array([C.shape[0] / 2, C.shape[1] / 2])
      if len(pts) == 0:
        return
      positions = np.array(positions)
      xy = np.array(positions[:,:2])
      theta = np.array(positions[:,2])\
          .reshape((positions.shape[0], 1)) * 3.14159265359 / 180.0
      cos_theta = np.cos(theta)
      sin_theta = np.sin(theta)
      # the collisions are in y, x format, so do rotation matrix weirdly
      rot = np.concatenate((-sin_theta, cos_theta, cos_theta, sin_theta), axis=1)\
          .reshape((theta.shape[0] * 2, 2)).T
      col = np.array(pts)
      col = np.dot(col, rot) + \
          np.reshape(np.fliplr(xy), (1, xy.size))
      pts = col.reshape((pts.shape[0] * theta.shape[0], 2)).astype(np.int)
      pts[:,1] = np.clip(pts[:,1], 0, self.grid.shape[0]-1)
      pts[:,0] = np.clip(pts[:,0], 0, self.grid.shape[1]-1)
      self.grid = self.grid * 0.8
      self.grid[pts[:,1], pts[:,0]] = 1.0

  def setPositions(self, positions):
    assert(type(positions) == type([])) # only work with particle filter for now
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
        self.updateCollisions(self.positions, self.collisions)
        print("[GRIDMAP] process time:", time.time() - currTime)

  def stop(self):
    self.stopstate = True
