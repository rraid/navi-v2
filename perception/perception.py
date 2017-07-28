import sys
sys.path.append("../device")
import devhub
import math
import numpy as np
from scipy.signal import convolve2d, correlate2d
from scipy.ndimage.interpolate import rotate
import chilipy
import cv2
import time

#Top four sonars from left to right followed by
#bottom five sonars from left to right
sonarAngle = np.array([-20, -10 , 10, -20, -20, -10, 0, 10, 20])

_X = np.arange(-9.0, 9.5, 0.5)
_Y = np.arange(-9.0, 9.5, 0.5)
_X, _Y = np.meshgrid(_X, _Y)
GPSD = np.exp(-(_X ** 2.0 + _Y ** 2.0) / (2.0 * 2.5))
GPSD /= np.sum(GPSD)

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

def addMatrixFromCenter(matrixA, matrixB):
  convX = (matrixA.shape[0] - matrixB.shape[0])/2
  convy = (matrixA.shape[1] - matrixB.shape[1])/2
  for x in range(matrixB.shape[0]):
    for y in range(matrixB.shape[1]):
      matrixA[x + convX, y + convY] += matrixB[x,y]
  return matrixA

def getCollisionDistribution(sonar, lidar, zed):
  sizex = max(lidar.shape[0],zed.shape[0],sonar.shape[0])
  sizey = max(lidar.shape[1],zed.shape[1],sonar.shape[1])
  sensorSum = np.zeros(sizex,sizey)

  sensorSum = addMatrixFromCenter(sensorSum,lidar)
  sensorSum = addMatrixFromCenter(sensorSum,sonar)
  sensorSum = addMatrixFromCenter(sensorSum,zed)
  return sensorSum

def getGPSDistribution(pos):
  global GPSD
  shape = [500, 500]
  offset = [0, 0] # lat, long
  x = pos[0] - offset[0]
  y = pos[1] - offset[1]

  if x <= 0 or x > shape[0] or y < 0 or y >= shape[1]:
    print "Error: GPS distribution out of bounds"
    return np.array([[]]) # no distribution

  x_ = x
  y_ = shape[1] - y - 1
  distribution = np.zeros(shape, dtype=np.float32)
  distribution[max(y_ - 18, 0) : min(y_ + 19, shape[1]),
               max(x_ - 18, 0) : min(x_ + 19, shape[0])] = GPSD
  return distribution 

def getCompassDistribution(degreesFromNorth):
  if degreesFromNorth == None:
    print "Error: Compass cannot be None"
    return np.array([[]]) # no distribution

  degrees = degreesFromNorth + 90.0 # account for offset
  # create a kernel for convolving - super overkill, but it works
  # TODO: make more efficient later (really inefficient right now)
  D = np.arange(-20, 30, 10.0)
  G = np.exp(-(D ** 2.0) / (2.0 * 10))
  G = np.reshape(G, (G.shape[0], 1))
  G /= np.sum(G)

  deg = int(round(degrees / 10.0)) % 36
  distribution = np.zeros((36, 1), dtype=np.float32)
  distribution[deg] = 1.0
  distribution = convolve2d(distribution, G, mode="same", boundary="wrap")
  return np.reshape(distribution, (distribution.shape[0], ))

class Perception(Thread):
  def __init__(self, pathmap):
    currTime = time.time()
    self.localizer = Localizer()
    self.mapper = GridMap()
    self.detector = ObjectDetector()
    self.pathmap = None
    self.stopstate = False
    self.pathmap = pathmap
    self.collisions = np.zeros(self.pathmap.shape[:2])
    self.localizer.initializeUniformly(self.pathmap.shape[:2])
    self.mapper.initializeEmpty(self.pathmap.shape[:2])
    self.detector.setObjects([]) # no objects for now, we can use them later

  def getCollisions(self):
    return np.copy(self.collisions)

  def run(self):
    lastTime = time.time()
    while not self.stopstate: # spin thread
      self.collisions = getCollisionDsitribution(devhub.getSonarReadings(), \
          devhub.getLidarReadings(), devhub.getZEDDepthColumns())
      if type(devhub.depthImage) != type(None) and devhub.depthImage != [] and \
         type(devhub.rgbImage) != type(None) and devhub.colorImage != []:
        self.detector.setImages(devhub.colorImage, devhub.depthImage)
      self.localizer.setGrid(self.mapper.predict())
      self.mapper.setPositions(self.localizer.predict())
      self.localizer.setSpeed(devhub.leftSpeed, devhub.rightSpeed)
      self.localizer.setObservation( \
        getGPSDistribution(devhub.getGPSLocation()), \
        getCompassDistribution(devhub.getCompassOrientation()), \
        self.collisions)
      self.mapper.setCollisions(self.collisions)
      currTime = time.time()
      print "[PERCEPTION] process time:", currTime - lastTime
      lastTime = currTime

  def stop(self):
    self.stopstate = True
