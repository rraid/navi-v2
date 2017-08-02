import sys
sys.path.append("../device")
import devhub
import math
import numpy as np
from scipy.signal import convolve2d, correlate2d
from scipy.ndimage.interpolation import rotate
import cv2
import time
from threading import Thread

#Top four sonars from left to right followed by
#bottom five sonars from left to right
sonarAngle = np.array([-20, -10 , 10, -20, -20, -10, 0, 10, 20])

mapShape = (480, 640)

_X = np.arange(-9.0, 9.5, 0.5)
_Y = np.arange(-9.0, 9.5, 0.5)
_X, _Y = np.meshgrid(_X, _Y)
GPSD = np.exp(-(np.multiply(_X, _X) + np.multiply(_Y, _Y)) / (2.0 * 2.5))
GPSD /= np.sum(GPSD)

def validDepth(d):
  return d != 0 and d != float("inf") and d != float("-inf") and d != float("nan")

def getSonarDistribution(sonarValues):
  sonarDistx = np.zeros((9,100), dtype = np.int)
  sonarDisty = np.zeros((9,100), dtype = np.int)
  relativeAngle = np.linspace(-7.5,7.5,100)
  sonarValues = [sonarValues[i] / 0.5 if validDepth(sonarValues[i]) else 0.0 for i in range(len(sonarValues))]
  for i in range(9):
    if sonarValues[i] == 0.0:
      continue
    for distribution in range(100):
      sonarDistx[i,distribution] =int(math.cos(math.radians(sonarAngle[i] + relativeAngle[distribution]))* sonarValues[i])
      sonarDisty[i,distribution] =int(math.sin(math.radians(sonarAngle[i] + relativeAngle[distribution]))* sonarValues[i])

  sizex = int(abs(np.amax(sonarDistx)) + abs(np.amin(sonarDistx)))
  sizey = int(abs(np.amax(sonarDisty)) + abs(np.amin(sonarDisty)))
  if sizex == 0:
      sizex = 1
  if sizey== 0:
      sizey = 1
  if sizex > 0 and sizey>0:
    sonarArray = np.zeros((sizex*2 + 1,sizey*2 + 1))
  for sonar in range(9):
    for i in range(100):

        sonarArray[sizex + sonarDistx[sonar,i], sizey + sonarDisty[sonar,i]] += 0.01
  return sonarArray


def getLidarDistribution(pts):
  size = len(pts)
  lidarDist = np.zeros((2,size), dtype = np.int)
  angleDist = np.linspace(135,-135,size)

  pts = [pts[i] / 0.5 if validDepth(pts[i]) else 0.0 for i in range(len(pts))]
  for i in range(size):
    if pts[i] == 0.0:
      continue
    lidarDist[0,i] = int(math.cos(math.radians(angleDist[i])) * pts[i])
    lidarDist[1,i] = int(math.sin(math.radians(angleDist[i])) * pts[i])

  sizex = abs(np.amax(lidarDist[0,:])) + abs(np.amin(lidarDist[0,:]))
  sizey = abs(np.amax(lidarDist[1,:])) + abs(np.amin(lidarDist[1,:]))
  if sizex == 0:
      sizex = 1
  if sizey== 0:
      sizey = 1
  lidarArray = np.zeros((sizex*2 + 1,sizey*2 + 1))

  for i in range(size):
    lidarArray[ sizex + lidarDist[0,i],sizey + lidarDist[1,i]] = 1
  return lidarArray

def getZEDDistribution(columns):
  size = len(columns)
  zedDist = np.zeros((2,size), dtype = np.int)
  angleDist = np.linspace(-55,55,size)
  columns = [columns[i] / 0.5 if validDepth(columns[i]) else 0.0 for i in range(len(columns))]
  for i in range(size):
    if columns[i] == 0.0:
      continue
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
    zedArray[ sizex + zedDist[0,i],sizey + zedDist[1,i]] += 1
  return zedArray

def addMatrixFromCenter(matrixA, matrixB):
  convX = (matrixA.shape[0] - matrixB.shape[0])/2
  convY = (matrixA.shape[1] - matrixB.shape[1])/2
  matrixA[convX:convX+matrixB.shape[0], convY:convY+matrixB.shape[1]] += matrixB
  return matrixA

def getCollisionDistribution(sonar, lidar, zed):
  sizex = max(lidar.shape[0],zed.shape[0],sonar.shape[0])
  sizey = max(lidar.shape[1],zed.shape[1],sonar.shape[1])
  sensorSum = np.zeros((sizex,sizey))

  sensorSum = addMatrixFromCenter(sensorSum,lidar)
  sensorSum = addMatrixFromCenter(sensorSum,sonar)
  sensorSum = addMatrixFromCenter(sensorSum,zed)
  center = (sensorSum.shape[0] / 2, sensorSum.shape[1] / 2)
  sensorSum[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = 0.0
  return np.clip(sensorSum, 0.0, 1.0)

def getGPSDistribution(pos):
  global GPSD
  global mapShape
  shape = mapShape
  offset = [0, 0] # lat, long
  # the 0.5 is because of the map ratio
  x = int(round((pos[0] - offset[0]) / 0.5))
  y = int(round((pos[1] - offset[1]) / 0.5))

  if x <= 0 or x > shape[0] or y < 0 or y >= shape[1]:
    print("Error: GPS distribution out of bounds")
    return np.array([[]]) # no distribution

  distribution = np.zeros(shape, dtype=np.float32)
  distribution[max(y - 18, 0) : min(y + 19, shape[1]),
               max(x - 18, 0) : min(x + 19, shape[0])] = GPSD
  return distribution 

def getCompassDistribution(degreesFromNorth):
  if degreesFromNorth == None:
    print("Error: Compass cannot be None")
    return np.array([[]]) # no distribution

  degrees = (degreesFromNorth + 90.0) % 360.0 # account for offset

  D = np.arange(0, 360, 10.0) - degrees
  D = D + (D > 180) * -360
  D = D + (D > 180) * -360
  D = np.exp(-np.multiply(D, D) / 60.0) # 30 degree standard deviation
  D /= np.sum(D)
  return D

class Perception(Thread):
  def __init__(self, pathmap):
    currTime = time.time()
    self.localizer = Localizer()
    self.mapper = GridMap()
    self.detector = ObjectDetector()
    self.stopstate = False
    self.pathmap = pathmap
    self.collisions = np.zeros(self.pathmap.shape[:2])
    self.localizer.initializeUniformly(self.pathmap.shape[:2])
    self.mapper.initializeEmpty(self.pathmap.shape[:2])
    self.detector.setObjects([]) # no objects for now, we can use them later

    global mapShape
    mapShape = self.pathmap.shape

  def getCollisions(self):
    return np.copy(self.collisions)

  def run(self):
    lastTime = time.time()
    self.localizer.start()
    self.mapper.start()
    self.detector.start()
    while not self.stopstate: # spin thread
      self.collisions = getCollisionDsitribution(
          getSonarDistribution(devhub.getSonarReadings()), \
          getLidarDistribution(devhub.getLidarReadings()), \
          getZEDDistribution(devhub.getZEDDepthColumns()))
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
      print("[PERCEPTION] process time:", currTime - lastTime)
      lastTime = currTime

  def stop(self):
    self.localizer.stop()
    self.mapper.stop()
    self.detector.stop()
    self.stopstate = True
