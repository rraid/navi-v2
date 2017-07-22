import sys
sys.path.append("../device/")
import devhub
import numpy as np

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
  # COMPLETE THIS METHOD
  return np.array([[]])

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
