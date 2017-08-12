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

## Y, X
mapShape = [290,317]

_X = np.arange(-10.0, 11.0, 1.0)
_Y = np.arange(-10.0, 11.0, 1.0)
_X, _Y = np.meshgrid(_X, _Y)
GPSD = np.exp(-(np.multiply(_X, _X) + np.multiply(_Y, _Y)) / (2.0 * 2.5))
GPSD /= np.sum(GPSD)

collisions = np.zeros(mapShape)

def validDepth(d):
  return d != 0 and d != float("inf") and d != float("-inf") and not math.isnan(d)

def getZedDistribution(columns):
  size = len(columns)
  if size == 0:
    return np.array([[]])
  angleDist = np.linspace(-55,55,size) *math.pi / 180
  columns = np.array([columns[i]/0.1 if validDepth(columns[i]) else 0.0 for i in range(len(columns))])
  c = np.cos(angleDist)
  s = np.sin(angleDist)
  
  y = np.clip((np.multiply(s, columns) + 0.3).astype(int) + mapShape[1]/2,0,mapShape[1])
  x = np.clip(np.multiply(c, columns).astype(int) + mapShape[0]/2,0,mapShape[0])
  
  zedArray = np.zeros((mapShape[0],mapShape[1]))
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
  sizex = mapShape[0]
  sizey = mapShape[1]
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
  try:
    x = int(round(pos[0]))
    y = int(round(pos[1]))
  except:
    return np.ones(shape) / float(np.prod(shape))
    
  if x < 0 or x >= shape[1] or y < 0 or y >= shape[0]:
    print("Error: GPS distribution out of bounds")
    return np.ones(shape) / float(np.prod(shape))
  distribution = np.zeros(shape[:2], dtype=np.float32)
  miny = max(y - 10, 0)
  maxy = min(y + 11, shape[0])
  minx = max(x - 10, 0)
  maxx = min(x + 11, shape[1])
  distribution[miny : maxy,
               minx : maxx] = GPSD[miny-y+10:maxy-y+10,minx-x+10:maxx-x+10]
  return distribution 

def getCompassDistribution(degreesFromNorth):
  if degreesFromNorth == None:
    print("Error: Compass cannot be None")
    return np.ones((36, )) / 36.0

  degrees = (degreesFromNorth + 90.0) % 360.0 # account for offset

  D = np.arange(0, 360, 10.0) - degrees
  D = D + (D > 180) * -360
  D = D + (D > 180) * -360
  D = D + (D < -180) * 360
  D = D + (D < -180) * 360
  D = np.exp(-np.multiply(D, D) / 80.0) # 40 degree standard deviation
  D /= np.sum(D)
  return D
  
def getCollisions():
    return np.copy(collisions)

class Perception(Thread):
  def __init__(self, pathmap):
    Thread.__init__(self)
    currTime = time.time()
    #self.localizer = pfilter.ParticleFilter()
    #self.mapper = GridMap()
    #self.detector = ObjectDetector()
    self.stopstate = False
    self.pathmap = pathmap
    
    #self.localizer.initializeUniformly(5000, (968,681,360))
    #self.mapper.initializeEmpty(self.pathmap.shape[:2])
    #self.detector.setObjects([]) # no objects for now, we can use them later
    self.left = 0
    self.right = 0

    global mapShape
    mapShape = self.pathmap.shape
    
  def setSpeed(self, left, right):
    self.left = left
    self.right = right

  def run(self):
    global collisions
    currTime = time.time()
    #self.mapper.start()
    #self.detector.start()
    while not self.stopstate: # spin thread
      collisions = getCollisionDistribution(
          getLidarDistribution(devhub.getLidarReadings()), \
          getZedDistribution(devhub.getZedReadings()))
      #self.localizer.updatePosition(self.left,self.right)
      #self.localizer.observePosition( \
        #getGPSDistribution(devhub.getGPSReadings()), \
        #getCompassDistribution(devhub.getCompassReadings()), \
        #self.collisions)
        
        
      #if type(devhub.depthImage) != type(None) and devhub.depthImage != [] and \
      #   type(devhub.rgbImage) != type(None) and devhub.colorImage != []:
      #  self.detector.setImages(devhub.colorImage, devhub.depthImage)
      #self.mapper.setPositions(self.localizer.predict())
      #self.mapper.setCollisions(self.collisions)
      print("[PERCEPTION] process time:", time.time() - currTime)
      currTime = time.time()

  def stop(self):
    #self.localizer.stop()
#    self.mapper.stop()
#    self.detector.stop()
    self.stopstate = True
