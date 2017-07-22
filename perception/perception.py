import numpy as np
from scipy.signal import convolve2d

def getSonarDistribution(sonarValues):
  # COMPLETE THIS METHOD
  return np.array([[]])

def getLidarDistribution(pts):
  # COMPLETE THIS METHOD
  return np.array([[]])

def getZEDDistribution(columns):
  # COMPLETE THIS METHOD
  return np.array([[]])

def getGPSDistribution(x, y, shape):
  if x <= 0 or x > shape[0] or y < 0 or y >= shape[1]:
    print "Error: distribution out of bounds"
    return np.array([[1]]) # no distribution

  X = np.arange(-9.0, 9.5, 0.5)
  Y = np.arange(9.0, -9.5, -0.5)
  X, Y = np.meshgrid(X, Y)
  G = np.exp(-(X ** 2.0 + Y ** 2.0) / (2.0 * 2.5))
  G /= np.sum(G)

  x_ = x
  y_ = shape[1] - y - 1
  distribution = np.zeros(shape, dtype=np.float32)
  distribution[y_ - 18 : y_ + 19, x_ - 18 : x_ + 19] = G
  return distribution 

def getCompassDistribution(degrees):
  # COMPLETE THIS METHOD
  return np.array([[]])

class Localizer():
  def __init__(self):
    pass

  def initializeUniformly(self, x_meters, y_meters):
    # COMPLETE THIS METHOD
    pass

  def updatePosition(self, vx, vy):
    # COMPLETE THIS METHOD
    pass

  def observePosition(self, gps, sonar, lidar, zed):
    # COMPLETE THIS METHOD
    pass

  def topPercentPredict(self, percent):
    # COMPLETE THIS METHOD
    return []

  def getDistribution(self):
    return np.array([[]])

class Map():
  def __init__(self):
    pass

  def initializeEmpty(self, x_meters, y_meters):
    # COMPLETE THIS METHOD
    pass

  def updateCollisions(self, x, y, collisions):
    # COMPLETE THIS METHOD
    pass

  def predict(self):
    # COMPLETE THIS METHOD
    return np.array([[]])

  def getDistribution(self):
    return np.array([[]])

class ObjectDetector():
  def __init__(self):
    pass

  def setObjects(self, objectNames):
    # COMPLETE THIS METHOD
    pass

  def observe(self, rgbImage):
    # COMPLETE THIS METHOD
    pass

  def detect(self):
    # COMPLETE THIS METHOD
    return []
