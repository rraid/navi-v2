import sys
sys.path.append("../device/")
import perception
import devhub
import pfilter
from threading import Thread

class PerceptBox(Thread):
  """
  This class initializes all the functions of the sensors and distributions that
  we have into one single object instance. This includes threads that have to be
  run, as well as memory allocation, syncronization, and sharing. By managing
  the sensors in this fashion, we are able to speed up the perception process,
  and output a more simple model of our environment.

  Note that this is the first version of the box which uses a online-learned
  calibration technique for correlation between the box and the sensor readings.
  """
  def __init__(self, gridimg, initial_pose=None, sensorport=None):
    Thread.__init__(self)
    """ Constructor
    Arguments:
    - gridimg: numpy array of dimensionality (rows, cols)
    - initial_pose: (x, y, theta) initial pose of the robot
    - sensorport: the port of the arduino that has the sensors
    """
    # shape = (y, x)
    self.gridimg = gridimg
    if type(initial_pose) == type(None):
      # (x, y, theta)
      self.initial_pose = (self.gridimg.shape[1] / 2, \
          self.gridimg.shape[0] / 2, 0)
    else:
      self.initial_pose = initial_pose

    if sensorport:
      devhub.init(megaport=sensorport, control_en=False)
    else: # do no initialization
      pass

    # mapper
    #self.gridmap = gridmap.GridMap()
    #self.gridmap.positionCallback = self.getPose
    #self.gridmap.collisionCallback = devhub.getZedReadings

    # start by defining a buffer to store the GPS and the robot
    self.bufmax = 2000 # max 2000 readings
    self.GPSBuffer = np.zeros((bufmax, 2))
    self.CompassBuffer = np.zeros((bufmax, 1))
    self.ZEDBuffer = np.zeros((bufmax, 3))

    # store the GPS readings until they fill up (note: this will take some time...)
    for i in range(self.bufmax):
      self.lastPos = devhub.getGPSReadings()
      self.lastTheta = devhub.getCompassReadings()
      self.lastPose = zed.getPose()
      self.GPSBuffer[i] = self.lastPos
      self.CompassBUffer[i] = self.lastTheta
      self.ZEDBuffer[i] = self.lastPose

    # learning an initial transformation
    self.transform = np.array([0, 0, 0], dtype=np.float32)
    self.update()

  def update(self):
    # assume that the compass is always right (later on use sin and cos rules)
    theta = np.mean(self.CompassBuffer - self.ZEDBuffer[:,2:])
    # use the transformation to obtain the offset
    s = math.sin(math.radians(theta))
    c = math.cos(math.radians(theta))
    newpos = np.dot(self.ZEDBuffer[:,:2], np.array([[c, s], [-s, c]]))
    # grab the offset from the matrix transform
    offset = np.mean(self.GPSBuffer - newpos)
    self.transform[:2] = offset
    self.transform[2] = theta

  def getPose(self):
    """
    Returns:
    - pose (x, y, theta)
    """
    t = self.transform[2]
    s = math.sin(math.radians(t))
    c = math.cos(math.radians(t))
    R = np.array([[c, s], [-s, c]])
    xy = np.dot(self.lastPose[:2], R) + self.transform[:2]
    return np.array([xy[0], xy[1], t + self.transform[2]])

  def getMap(self, radius=10.0, scaling=0.1):
    """
    Get the distribution of the map, given a radius
    """
    # for now just test the localizer to see if it works
