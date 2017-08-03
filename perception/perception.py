import sys
sys.path.append("../device")
import devhub
import pfilter
import math
import numpy as np
from scipy.signal import convolve2d, correlate2d
from scipy.ndimage.interpolation import rotate
import cv2
import time
from threading import Thread

#Top four sonars from left to right followed by
#bottom five sonars from left to right
sonarAngle = np.array([-30, -5 , 5, 30, -45, -25, 0, 25, 45])

mapShape = (480, 640)

_X = np.arange(-9.0, 9.5, 0.5)
_Y = np.arange(-9.0, 9.5, 0.5)
_X, _Y = np.meshgrid(_X, _Y)
GPSD = np.exp(-(np.multiply(_X, _X) + np.multiply(_Y, _Y)) / (2.0 * 2.5))
GPSD /= np.sum(GPSD)

def validDepth(d):
  return d != 0 and d != float("inf") and d != float("-inf") and not math.isnan(d)

def getSonarDistribution(sonarValues):
  size = len(sonarValues)
  if size == 0:
    return np.array([[]])
  sonarDistx = np.zeros((9,100), dtype = np.int)
  sonarDisty = np.zeros((9,100), dtype = np.int)
  relativeAngle = np.linspace(-7.5,7.5,100)
  sonarValues = [sonarValues[i] / 0.1 if validDepth(sonarValues[i]) else 0.0 for i in range(len(sonarValues))]
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

        sonarArray[sizex + sonarDistx[sonar,i], sizey + sonarDisty[sonar,i]] += 0.1
        #Changed from .01 to .1 because it shows better with agent.py
  return sonarArray


def getLidarDistribution(pts):
  size = len(pts)
  if size == 0:
    return np.array([[]])
  angleDist = np.linspace(135,-135,size) * math.pi / 180
  pts = np.array([pts[i] / 0.1 if validDepth(pts[i]) else 0.0 for i in range(len(pts))])
  c = np.cos(angleDist)
  s = np.sin(angleDist)
  
  y = np.clip(np.multiply(s, pts).astype(int) + 300,0,600)
  x = np.clip(np.multiply(c, pts).astype(int) + 300,0,600)

  lidarArray = np.zeros((600,600))
  
  lidarArray[x,y] = 1.0
  return lidarArray

def getZedDistribution(columns):
  size = len(columns)
  if size == 0:
    return np.array([[]])
  angleDist = np.linspace(-55,55,size) *math.pi / 180
  columns = np.array([columns[i]/0.1 if validDepth(columns[i]) else 0.0 for i in range(len(columns))])
  c = np.cos(angleDist)
  s = np.sin(angleDist)
  
  y = np.clip((np.multiply(s, columns) + 0.3).astype(int) + 300,0,600)
  x = np.clip(np.multiply(c, columns).astype(int) + 300,0,600)
  
  zedArray = np.zeros((600,600))
  zedArray[x,y] = 1.0
  return zedArray

def addMatrixFromCenter(matrixA, matrixB):
  convX = (matrixA.shape[0] - matrixB.shape[0])/2
  convY = (matrixA.shape[1] - matrixB.shape[1])/2
  matrixA[convX:convX+matrixB.shape[0], convY:convY+matrixB.shape[1]] += matrixB
  return matrixA

def getCollisionDistribution(lidar, zed):
  if type(zed) == type(None):
    return []
  if type(lidar) == type(None):
    return []
  #sizex = max(lidar.shape[0],zed.shape[0])
  #sizey = max(lidar.shape[1],zed.shape[1])
  sizex = 600
  sizey = 600
  sensorSum = np.zeros((sizex,sizey))

  sensorSum = addMatrixFromCenter(sensorSum,lidar)
  #sensorSum = addMatrixFromCenter(sensorSum,sonar)
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
    Thread.__init__(self)
    currTime = time.time()
    self.localizer = pfilter.ParticleFilter()
    #self.mapper = GridMap()
    #self.detector = ObjectDetector()
    self.stopstate = False
    self.pathmap = pathmap
    self.collisions = np.zeros(self.pathmap.shape[:2])
    self.localizer.initializeUniformly(5000, list(self.pathmap.shape[:2]) + [360])
    #self.mapper.initializeEmpty(self.pathmap.shape[:2])
    #self.detector.setObjects([]) # no objects for now, we can use them later
    self.left = 0
    self.right = 0

    global mapShape
    mapShape = self.pathmap.shape

  def getCollisions(self):
    return np.copy(self.collisions)
    
  def setSpeed(self, left, right):
    self.left = left
    self.right = right

  def run(self):
    lastTime = time.time()
    #self.mapper.start()
    #self.detector.start()
    while not self.stopstate: # spin thread
      self.collisions = getCollisionDistribution(
          getLidarDistribution(devhub.getLidarReadings()), \
          getZedDistribution(devhub.getZedReadings()))
      self.localizer.updatePosition(self.left,self.right)
      self.localizer.observePosition( \
        getGPSDistribution(devhub.getGPSLocation()), \
        getCompassDistribution(devhub.getCompassOrientation()), \
        self.collisions)
      #if type(devhub.depthImage) != type(None) and devhub.depthImage != [] and \
      #   type(devhub.rgbImage) != type(None) and devhub.colorImage != []:
      #  self.detector.setImages(devhub.colorImage, devhub.depthImage)
      #self.localizer.setGrid(self.mapper.predict())
      #self.mapper.setPositions(self.localizer.predict())
      #self.localizer.setSpeed(devhub.motorVelocity[0], devhub.motorVelocity[1])

      #self.mapper.setCollisions(self.collisions)
      currTime = time.time()
      print("[PERCEPTION] process time:", currTime - lastTime)
      lastTime = currTime

  def stop(self):
#    self.localizer.stop()
#    self.mapper.stop()
#    self.detector.stop()
    self.stopstate = True
