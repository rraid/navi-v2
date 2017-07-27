import math
import numpy as np
from scipy.signal import convolve2d, correlate2d
from scipy.ndimage.interpolate import rotate
from PIL, import Image, ImageDraw
import chilipy
import cv2

#Top four sonars from left to right followed by
#bottom five sonars from left to right
sonarAngle = np.array([-20, -10 , 10, -20, -20, -10, 0, 10, 20])

def patchSSD2d(in1, in2, mode="same", boundary="fill"):
  assert len(in1.shape) == 2
  assert len(in2.shape) == 2
  totWeight = float(in2.shape[0] * in2.shape[1])
  B2 = np.sum(np.multiply(in2, in2))
  AB = 2.0 * correlate2d(in1, in2, mode=mode, boundary=boundary)
  A2 = np.multiply(in1, in1)
  sumsq = (B2 - AB + A2) / totWeight
  # for some reason this can be negative, try to check
  assert np.min(sumsq) >= 0.0
  return sumsq

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

def getGPSDistribution(x, y):
  shape = [500, 500]
  offset = [0, 0] # lat, long
  x -= offset[0]
  y -= offset[1]

  if x <= 0 or x > shape[0] or y < 0 or y >= shape[1]:
    print "Error: GPS distribution out of bounds"
    return np.array([[]]) # no distribution

  X = np.arange(-9.0, 9.5, 0.5)
  Y = np.arange(-9.0, 9.5, 0.5)
  X, Y = np.meshgrid(X, Y)
  G = np.exp(-(X ** 2.0 + Y ** 2.0) / (2.0 * 2.5))
  G /= np.sum(G)

  x_ = x
  y_ = shape[1] - y - 1
  distribution = np.zeros(shape, dtype=np.float32)
  distribution[max(y_ - 18, 0) : min(y_ + 19, shape[1]),
               max(x_ - 18, 0) : min(x_ + 19, shape[0])] = G
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
  return distribution

class Localizer():
  def __init__(self):
    self.state = None
    self.left = 0.0
    self.right = 0.0
    self.timeUpdated = time.time()

  def initializeUniformly(self, sentShape):
    self.state = np.ones((sentShape.shape[0],sentShape.shape[1],36))
    self.state.fill(1/np.sum(self.state))

  def updatePosition(self, left, right):
    currTime = time.time()
    dt = currTime - self.timeUpdated
    self.timeUpdated = currTime
    # use the velocity model (Probabilistic Robotics 5.2)
    wheel_radius = 0.14
    robot_radius = 0.343
    rpm = 5700
    vel = lambda k: 2.0 * math.pi * wheel_radius * rpm * k / 60.0
    l = vel(self.left)
    r = vel(self.right)
    v = (l + r) / 2.0
    w = (r - l) / (2.0 * robot_radius)
    # apply a skewed gaussian on the forward motion
    kernel = np.zeros((65,65))
    for i in range(kernel.shape[0]):
      for j in range(kernel.shape[1]):
        kernel[i,j] = math.exp(-1*(1/(20*dt+1e-6))*(((i-32-v)**2) + ((j-32)**2)))
    for theta in self.state.size[2]:
      self.state[:,:,theta] = convolve2d(
          self.state, rotate(kernel, theta * 10), mode="same")

    # apply a gaussian on the angular motion
    D = np.arange(-40, 50, 10.0)
    G = np.exp(-((D - w) ** 2.0) / (2.0 * 5.0))
    G = np.reshape(G, (1, G.shape[0]))
    G /= np.sum(G)

    temp = np.reshape(self.state,
        (self.state.shape[0] * self.state.shape[1], self.state.shape[2]))
    temp = convolve2d(temp, G, mode="same")
    self.state = np.reshape(temp, self.state.shape)

    self.left = left
    self.right = right

  def observePosition(self, gps, compass, collisions):
    for theta in range(self.state.shape[2])
      self.state[:,:,theta] = \
          np.multiply(self.state[:,:,theta], gps) * compass[theta]
      self.state[:,:,theta] = patchSSD2d(
          self.state, rotate(collisions, theta * 10), mode="same")

  def topPercentPredict(self, percent):
    return np.argwhere(self.state>percent)

  def predict(self):
    # return a copy of the distribution
    self.updatePosition(self.left, self.right)
    return self.state[:,:,:]

class GridMap():
  def __init__(self, decay_rate=0.5):
    self.x_inc = 0.5
    self.x_meters = 0
    self.y_inc = 0.5
    self.y_meters = 0
    self.theta_inc = 10
    self.n_theta = 36
    self.decay_rate = decay_rate
    self.grid = None
    self.timeUpdated = time.time()

  def initializeEmpty(self, x_meters, y_meters):
    self.x_meters = x_meters
    self.y_meters = y_meters
    self.grid = np.zeros((int(y_meters / self.y_inc),
                          int(x_meters / self.x_inc)), dtype=np.float32)

  def updateCollisions(self, localizer, collisions):
    assert type(self.grid) != type(None)
    # just lower the probability of all other collisions in the space wrt. time
    currTime = time.time()
    dt = currTime - self.timeUpdated
    self.timeUpdated = currTime
    self.grid *= math.exp(-self.decay_rate * dt)

    # convolve the collisions with the localizer and add it in
    positions = localizer.predict()
    assert(np.sum(positions) == 1.0) # be careful! super slow
    obstacles = np.zeros((positions.shape[0], positions.shape[1]))
    for i in range(positionDistribution.shape[2]):
      obstacles += convolve2d(positions[:,:,i], rotate(collisions, 10 * i))

    # add collisions
    self.grid = np.clip(self.grid + obstacles, 0.0, 1.0)

  def predict(self):
    return self.grid

class ObjectDetector():
  def __init__(self):
    self.tags = dict()

  def setObjects(self, objectNames):
    for obj in objectNames:
      self.tags.update({obj:None})

  def observe(self, rgbImage,depthImage):
    grayImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2GRAY))
    allTags = chili.find(grayImage)
    for tag in self.tags:
      corners = numpy.array(allTags.get(tag))
      midpoint = (corners[2:4] - corners[0:2]) - (corners[6:8] - corners[4:6])
      ##Couldnt think of a good way to calculate the sum of any quadrilatiral,
      ##so i just returned the depth at the midpoint
      depth = depthImage[(midpoint[0],midpoint[1])]
      self.tags[tag] = numpy.array((midpoint,depth))

  def detect(self):
    return self.tags
