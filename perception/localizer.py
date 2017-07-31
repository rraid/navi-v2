import math
import numpy as np
from scipy.signal import convolve2d
from scipy.ndimage.interpolation import rotate
import cv2
from threading import Thread
import sys
sys.path.append("../control/")
import control
import time
import mxnet as mx

def patchSSD2d(in1, in2, mode="same", boundary="fill"):
  assert len(in1.shape) == 2
  assert len(in2.shape) == 2
  in2 = np.flupud(np.fliplr(in2))
  totWeight = float(np.prod(in2.shape))
  B2 = np.sum(np.multiply(in2, in2))
  AB = 2.0 * convolve2d(in1, in2, mode=mode, boundary=boundary)
  A2 = np.multiply(in1, in1)
  sumsq = (B2 - AB + A2) / totWeight
  # for some reason this can be negative, try to check
  assert np.min(sumsq) >= 0.0
  assert np.max(sumsq) <= 1.0
  # return np.abs(sumsq)
  return sumsq

class Localizer(Thread):
  def __init__(self):
    Thread.__init__(self)
    self.state = None
    self.left = 0.0
    self.right = 0.0
    self.newLeftSpeed = 0.0
    self.newRightSpeed = 0.0
    self.timeUpdated = time.time()
    self.grid = None
    self.stopstate = False
    self.lastTime = time.time()
    self.gps = None
    self.compass = None
    self.collisions = None

  def initializeUniformly(self, sentShape):
    self.state = np.zeros((sentShape[0],sentShape[1],36))
    self.state.fill(1.0 / self.state.size)
    #self.state[200, 200, :] = 1.0
    self.gps = np.zeros((sentShape[0], sentShape[1]))
    self.compass = np.zeros((36, ))
    self.collisions = np.zeros((1, 1))

  def updatePosition(self, left, right):
    currTime = time.time()
    dt = currTime - self.timeUpdated
    self.timeUpdated = currTime
    # use the velocity model (Probabilistic Robotics 5.2)
    vel = 2.0 * math.pi * control.wheel_radius * control.RPM / 60.0
    l = vel * self.left
    r = vel * self.right
    v = (l + r) / 2.0
    w = (r - l) / (2.0 * control.robot_radius)
    # apply a skewed gaussian on the forward motion
    ksize = 9
    x = np.arange(-ksize, ksize+1, 1)
    y = np.arange(-ksize, ksize+1, 1)
    x, y = np.meshgrid(x, y)
    kernel = np.exp(-1.0 / (1.5 * dt) * ((y-v * dt)**2.0 + x**2.0))
    kernel /= np.sum(kernel)
    update = np.zeros((36, kernel.shape[0], kernel.shape[1]))
    for theta in range(self.state.shape[2]):
      update[theta,:,:] = rotate(kernel, theta * 10, reshape=False)
    state = mx.nd.array(np.rollaxis(self.state, -1)).as_in_context(mx.gpu(0))\
        .reshape((1, 36, self.state.shape[0], self.state.shape[1]))
    update = mx.nd.array(update).as_in_context(mx.gpu(0))\
        .reshape((1, update.shape[0], update.shape[1], update.shape[2]))
    for theta in range(self.state.shape[2]):
      self.state[:,:,theta] = mx.nd.Convolution(num_filter=1,
          data=mx.nd.slice_axis(state, axis=1, begin=theta, end=theta+1),
          weight=mx.nd.slice_axis(update, axis=1, begin=theta, end=theta+1),
          no_bias=True, kernel=(ksize*2+1, ksize*2+1), pad=(ksize, ksize))\
          .reshape((self.state.shape[0], self.state.shape[1])).asnumpy()

    # apply a gaussian on the angular motion
    D = np.arange(-40, 50, 10.0) - w * dt
    G = np.exp(-np.multiply(D, D) / 10.0)
    G = np.reshape(G, (1, G.shape[0]))
    G /= np.sum(G)

    # TODO: make this faster with mxnet convolutions
    temp = np.reshape(self.state,
        (self.state.shape[0] * self.state.shape[1], self.state.shape[2]))
    temp = convolve2d(temp, G, mode="same", boundary="wrap")
    self.state = np.reshape(temp, self.state.shape)

    if np.sum(self.state) == 0.0:
      print("[LOCALIZER] Error: probability for position was 0, resetting")
      self.state.fill(1.0 / self.state.size)
    else:
      self.state /= np.sum(self.state)

    self.left = left
    self.right = right

  def observePosition(self, gps, compass, collisions, grid):
    state = mx.nd.array(np.rollaxis(self.state, -1)).as_in_context(mx.gpu(0))\
        .reshape((1, 36, self.state.shape[0], self.state.shape[1]))
    d_gps = mx.nd.array(gps, ctx=mx.gpu(0)).reshape((1, 1, gps.shape[0], gps.shape[1]))
    B2 = np.sum(np.multiply(collisions, collisions))
    for theta in range(self.state.shape[2]):
      if compass[theta] == 0.0:
        self.state[:,:,theta].fill(0)
        continue
      #self.state[:,:,theta] = mx.nd.multiply(compass[theta] * d_gps, \
      #    mx.nd.slice_axis(state, axis=1, begin=theta, end=theta+1))\
      #    .reshape(self.state.shape[:2]).asnumpy()
      F = mx.nd.multiply(compass[theta] * d_gps, \
          mx.nd.slice_axis(state, axis=1, begin=theta, end=theta+1))
      d_collisions = mx.nd.flip(mx.nd.array(rotate(collisions, theta * 10), \
          ctx=mx.gpu(0)), axis=(0, 1))\
          .reshape((1, 1, collisions.shape[0], collisions.shape[1]))
      totWeight = float(d_collisions.size)
      #B2 = mx.nd.sum(mx.nd.multiply(d_collisions, d_collisions))
      AB = 2.0 * mx.nd.Convolution(F, d_collisions, kernel=collisions.shape,
          no_bias=True, num_filter=1,
          pad=(collisions.shape[0]/2, collisions.shape[1]/2))
      A2 = mx.nd.multiply(F, F)
      G = 1.0 - ((B2 - AB + A2) / totWeight)
      assert np.min(G.asnumpy()) >= 0.0
      assert np.max(G.asnumpy()) <= 1.0
      #G = np.abs(G)
      self.state[:,:,theta] = (F * G).reshape(self.state.shape[:2]).asnumpy()
      #self.state[:,:,theta] = np.multiply(F,
      #    1.0 - patchSSD2d(grid, rotate(collisions, theta * 10), mode="same"))
    self.state /= np.sum(self.state)

  def topPercentPredict(self, percent):
    return np.argwhere(self.state>percent)

  def predict(self):
    return self.state

  def setGrid(self, grid):
    self.grid = grid

  def setSpeeds(self, left, right):
    self.newLeftSpeed = left
    self.newRightSpeed = right

  def setObservation(self, gps, compass, collisions):
    # for now, only copy a 49x49 (max) window for collisions
    if type(collisions) != type(None) and \
        collisions.shape[0] > 49:
      midx = collisions.shape[0] / 2
      collsions = collisions[midx - 24 : midx + 25, :]
    if type(collisions) != type(None) and \
        collisions.shape[1] > 49:
      midy = collisions.shape[1] / 2
      collisions = collisions[:, midy - 24 : midy + 25]
    self.collisions = collisions
    self.gps = gps
    self.compass = compass

  def run(self):
    while not self.stopstate:
      currTime = time.time()
      if currTime - self.lastTime < 0.1:
        continue
      self.lastTime = currTime
      self.updatePosition(self.newLeftSpeed, self.newRightSpeed)
      print("[LOCALIZER] process time on update:", time.time() - self.currTime)
      if type(self.state) != type(None) and \
         type(self.grid) != type(None) and \
         type(self.gps) != type(None) and \
         self.gps[0] != None and self.gps[1] != None and \
         type(self.compass) != type(None) and \
         type(self.collisions) != type(None):
        self.observePosition(np.copy(self.gps), np.copy(self.compass), \
            np.copy(self.collisions), np.copy(self.grid))
      print("[LOCALIZER] process time on observe:", time.time() - self.currTime)

  def stop(self):
    self.stopstate = True
