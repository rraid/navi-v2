import math
import numpy as np
from scipy.signal import convolve2d, correlate2d
from scipy.ndimage.interpolate import rotate
import cv2
from threading import Thread
import sys
sys.path.append("../control/control.py")

def patchSSD2d(in1, in2, mode="same", boundary="fill"):
  assert len(in1.shape) == 2
  assert len(in2.shape) == 2
  totWeight = float(np.prod(in2.shape))
  B2 = np.sum(np.multiply(in2, in2))
  AB = 2.0 * correlate2d(in1, in2, mode=mode, boundary=boundary)
  A2 = np.multiply(in1, in1)
  sumsq = (B2 - AB + A2) / totWeight
  # for some reason this can be negative, try to check
  assert np.min(sumsq) >= 0.0
  return sumsq

class Localizer(Thread):
  def __init__(self):
    self.state = None
    self.left = 0.0
    self.right = 0.0
    self.newLeftSpeed = 0.0
    self.newRightSpeed = 0.0
    self.timeUpdated = time.time()
    self.grid = None
    self.stopstate = False

  def initializeUniformly(self, sentShape):
    self.state = np.ones((sentShape.shape[0],sentShape.shape[1],36))
    self.state.fill(1/np.sum(self.state))

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
    x = np.arange(-32, 33, 1)
    y = np.arange(-32, 33, 1)
    x, y = meshgrid(x, y)
    kernel = np.exp(-1.0 / (20 * dt + 1e-6) * \
        ((y - v) ** 2.0 + np.multiply(x, x)))
    for theta in range(self.state.shape[2]):
      self.state[:,:,theta] = \
          convolve2d(self.state, rotate(kernel, theta * 10), mode="same")

    # apply a gaussian on the angular motion
    D = np.arange(-40, 50, 10.0)
    G = np.exp(-((D - w) ** 2.0) / (2.0 * 5.0))
    G = np.reshape(G, (1, G.shape[0]))
    G /= np.sum(G)

    temp = np.reshape(self.state,
        (self.state.shape[0] * self.state.shape[1], self.state.shape[2]))
    temp = convolve2d(temp, G, mode="same")
    self.state = np.reshape(temp, self.state.shape)

    if np.sum(self.state) == 0.0:
      print "[LOCALIZER] Error: probability for position was 0, resetting"
      self.state = 1.0 / np.prod(self.state.shape)

    self.left = left
    self.right = right

  def observePosition(self, gps, compass, collisions, grid):
    for theta in range(self.state.shape[2]):
      self.state[:,:,theta] = \
          np.multiply(self.state[:,:,theta], gps) * compass[theta]
      self.state[:,:,theta] = np.multiply(self.state[:,:,theta],
          patchSSD2d(grid, rotate(collisions, theta * 10), mode="same"))

  def topPercentPredict(self, percent):
    return np.argwhere(self.state>percent)

  def predict(self):
    return np.copy(self.state)

  def setGrid(self, grid):
    self.grid = grid

  def setSpeeds(self, left, right):
    self.newLeftSpeed = left
    self.newRightSpeed = right

  def setObservation(self, gps, compass, collisions):
    self.gps = gps
    self.compass = compass
    self.collisions = collisions

  def run(self):
    lastTime = time.time()
    while not self.stopstate:
      currTime = time.time()
      if currTime - lastTime < 0.05:
        pass
      lastTime = currTime
      self.updatePosition(self.newLeftSpeed, self.newRightSpeed)
      if type(self.gps) != type(None) and \
         self.gps[0] != None and self.gps[1] != None and \
         type(self.compass) != type(None) and \
         type(self.collisions) != type(None):
        self.observePosition(np.copy(self.gps), np.copy(self.compass), \
            np.copy(self.collisions))
      print "[LOCALIZER] process time:", time.time() - lastTime

  def stop(self):
    self.stopstate = True
