import sys
sys.path.append("../control/")
import control
import math
import random
import numpy as np
import mxnet as mx
import time

def rotateImage(image, angle):
  image_center = tuple(np.array(image.shape)/2)
  rot_mat = cv2.getRotationMatrix2D(image_center,-angle,1.0) # negative due to flipud
  result = cv2.warpAffine(image, rot_mat, image.shape,flags=cv2.INTER_CUBIC)
  return result

class ParticleFilter:
  def __init__(self):
    self.particles = None
    self.timeUpdated = time.time()
    self.shape = None
    self.left = 0.0
    self.right = 0.0
    self.n_particles = 0

  def initializeUniformly(self, n_particles, shape):
    self.n_particles = n_particles
    assert(len(shape) == 3)
    self.particles = np.multiply(np.random.random((n_particles, len(shape))) *
        0.9999999, np.array((shape)))
    self.shape = shape

  def getDistribution(self):
    return self.particles

  def predict(self):
    dist = self.particles.astype(np.int)
    grid = np.zeros(self.shape[:2][::-1])
    grid[dist[:,1], dist[:,0]] = 1.0
    return grid

  def updatePosition(self, left, right):
    # get the update parameters
    currTime = time.time()
    dt = currTime - self.timeUpdated
    self.timeUpdated = currTime
    # use the velocity model (Probabilistic Robotics 5.2)
    vel = 2.0 * math.pi * control.wheel_radius * control.RPM / 60.0
    l = vel * left
    r = vel * right
    v = (l + r) / 2.0
    w = (r - l) / (2.0 * control.robot_radius)

    # update the position
    T = self.particles[:,2:3]
    V = v * np.concatenate((np.cos(T), np.sin(T)), axis=1) + \
        np.random.normal(0, 1.0, (self.particles.shape[0], 2))
    W = np.random.normal(w, 2.0, (self.particles.shape[0], 1))
    VW = np.concatenate((V, W), axis=1)

    self.particles += VW
    self.particles[2] %= 360.0
    for i in range(len(self.shape)):
      self.particles[:,i] = \
          np.clip(self.particles[:,i], 0, self.shape[i] * 0.9999999)

  def observePosition(self, gps, compass, collisions):
    # first assign a probability for each of the particles
    particles = self.particles.astype(np.int)

    G = gps[particles[:,1], particles[:,0]]
    C = compass[(particles[:,2] / 10) % 36]
    print G
    health = np.multiply(G, C)
    # do GPU patchSSD2d later
    if np.sum(health) == 0.0:
      print("Error: sum of healths is 0")
      #self.initializeUniformly(self.n_particles, self.shape)
      return
    
    wheel = np.cumsum(health) / np.sum(health)
    probs = np.random.random((particles.shape[0], )) * 0.9999999
    indeces = [np.min(np.argwhere(probs[i] < wheel)) \
        for i in range(particles.shape[0])]
    self.particles = self.particles[indeces]
