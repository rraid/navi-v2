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

def getGPSDistribution(x, y):
  # TODO: account for longitude, latitude offset
  shape = (500, 500)

  if x <= 0 or x > shape[0] or y < 0 or y >= shape[1]:
    print "Error: GPS distribution out of bounds"
    return np.array([[]]) # no distribution

  X = np.arange(-9.0, 9.5, 0.5)
  Y = np.arange(9.0, -9.5, -0.5)
  X, Y = np.meshgrid(X, Y)
  G = np.exp(-(X ** 2.0 + Y ** 2.0) / (2.0 * 2.5))
  G /= np.sum(G)

  x_ = x
  y_ = shape[1] - y - 1
  distribution = np.zeros(shape, dtype=np.float32)
  distribution[max(y_ - 18, 0) : min(y_ + 19, shape[1]),
               max(x_ - 18, 0) : min(x_ + 19, shape[0])] = G
  return distribution 

def getCompassDistribution(degrees):
  if degrees == None:
    print "Error: Compass cannot be None"
    return np.array([[]]) # no distribution

  D = np.arange(-20, 30, 10.0)
  G = np.exp(-(D ** 2.0) / (2.0 * 10))
  G = np.reshape(G, (G.shape[0], 1))
  G /= np.sum(G)

  deg = int(round(degrees / 10.0)) % 36
  distribution = np.zeros((36, 1), dtype=np.float32)
  distribution[deg] = 1.0
  distribution = convolve2d(distribution, G, mode="same", boundary="wrap")
  return distribution

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
