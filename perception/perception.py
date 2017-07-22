import math
import numpy as np
from scipy.signal import convolve2d
from PIL, import Image, ImageDraw
import chilipy
import cv2

#Top four sonars from left to right followed by
#bottom five sonars from left to right
sonarAngle = np.array([-20, -10 , 10, -20, -20, -10, 0, 10, 20])

def getSonarDistribution(sonarValues):
  sonarDistx = np.zeros((9,100))
  sonarDisty = np.zeros((9,100))
  relativeAngle = np.linspace(-7.5,7.5,100)
  for i in range(9):
    for distribution in range(100):
      sonarDistx[i,distribution] =int(math.cos(math.radians(sonarAngle[i] + relativeAngle[distribution]))* sonarValues[i])
      sonarDisty[i,distribution] =int(math.sin(math.radians(sonarAngle[i] + relativeAngle[distribution]))* sonarValues[i])

  sizex = abs(np.amax(sonarDistx)) + abs(np.amin(sonarDistx))
  sizey = abs(np.amax(sonarDisty)) + abs(np.amin(sonarDisty))
  if sizex == 0:
      sizex = 1
  if sizey== 0:
      sizey = 1
  if sizex > 0 and sizey>0:

    sonarArray = np.zeros((sizex*2 + 1,sizey*2 + 1))
  for sonar in range(9):
    for i in range(100):
        sonarArray[sizex + sonarDistx[sonar,i], sizey + sonarDisty[sonar,i]] = 1
  return sonarArray

def getLidarDistribution(pts):
  size = len(pts)
  lidarDist = np.zeros((2,size))
  angleDist = np.linspace(-135,135,size)
  for i in range(size):
    lidarDist[0,i] = int(math.cos(math.radians(angleDist[i])) * pts[i])
    lidarDist[1,i] = int(math.sin(math.radians(angleDist[i])) * pts[i])

  sizex = abs(np.amax(lidarDist[0,:])) + abs(np.amin(lidarDist[0,:]))
  sizey = abs(np.amax(lidarDist[1,:])) + abs(np.amin(lidarDist[1,:]))
  if sizex == 0:
      sizex = 1
  if sizey== 0:
      sizey = 1
  if sizex > 0 and sizey>0:

    lidarArray = np.zeros((sizex*2 + 1,sizey*2 + 1))
  for i in range(size):
    lidarArray[ sizex + lidarDist[0,i],sizey + lidarDist[1,i]] = 1
  return lidarArray
rn np.array([[]])

def getZEDDistribution(columns):
  size = len(columns)
  zedDist = np.zeros((2,size))
  angleDist = np.linspace(-55,55,size)
  for i in range(size):
    zedDist[0,i] = int(math.cos(math.radians(angleDist[i])) * columns[i])
    zedDist[1,i] = int(math.sin(math.radians(angleDist[i])) * columns[i])

  sizex = abs(np.amax(zedDist[0,:])) + abs(np.amin(zedDist[0,:]))
  sizey = abs(np.amax(zedDist[1,:])) + abs(np.amin(zedDist[1,:]))
  if sizex == 0:
      sizex = 1
  if sizey== 0:
      sizey = 1
  if sizex > 0 and sizey>0:

    zedArray = np.zeros((sizex*2 + 1,sizey*2 + 1))
  for i in range(size):
    zedArray[ sizex + zedDist[0,i],sizey + zedDist[1,i]] = 1
  return zedArray

def getGPSDistribution(x, y):
  shape = [500, 500]
  offset = [0, 0] # lat, long
  x -= offset[0]
  y -= offset[1]

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
    self.x_inc = 0.5
    self.x_meters = 0
    self.y_inc = 0.5
    self.y_meters = 0
    self.theta_inc = 10
    self.n_theta = 36
    self.grid = None
    self.timeUpdated = time.time()

  def initializeEmpty(self, x_meters, y_meters):
    self.x_meters = x_meters
    self.y_meters = y_meters
    self.grid = np.zeros((y_meters * 2, x_meters * 2, 36), dtype=np.float32)

  def updateCollisions(self, x, y, collisions):
    assert(type(self.grid) != type(None))
    # just lower the probability of all other collisions in the space
    currTime = time.time()
    dt = currTime - self.timeUpdated
    self.timeUpdated = currTime
    self.grid *= math.exp(0.9 * dt)
    # add collisions
    self.grid = np.clip(self.grid + collisions, 0.0, 1.0)

  def predict(self):
    return self.grid

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
