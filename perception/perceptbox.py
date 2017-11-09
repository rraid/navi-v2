import sys, math
sys.path.append("../device/")
import perception
import devhub
from threading import Thread
import time
import numpy as np

class PerceptBox(Thread):
  """
  Acts as the front end for the perception unit. This unit contains two buffers:
  1) Global Buffer: a buffer which contains (GPS, Compass) readings
  2) Local Buffer: a buffer which contains more accurate stereo readings

  Note that this is the first version of the box which uses a mean-shifted
  correlation technique to connect the local and global frames.
  """
  def __init__(self, gpsCallback=None, compassCallback=None, stereoPoseCallback=None):
    """ Constructor
    Arguments:
    - gpsCallback: a function which returns the gps reading (x, y)
    - compassCallback: a function which returns the compass reading theta
    - stereoPoseCallback: a function which returns (x, y, theta)
    """
    assert(type(gpsCallback) != type(None))
    assert(type(compassCallback) != type(None))
    assert(type(stereoPoseCallback) != type(None))

    Thread.__init__(self)

    # start by defining a buffer to store the GPS and the robot
    self.bufmax = 2000 # max 2000 readings
    self.globalBuffer = np.zeros((self.bufmax, 3))
    self.localBuffer = np.zeros((self.bufmax, 3))

    # create a counter for current frame index
    self.frameid = 0
    self.gpsCallback = gpsCallback
    self.compassCallback = compassCallback
    self.stereoPoseCallback = stereoPoseCallback
    self.globalPose = None
    self.localPose = None

  def update(self):
    """ Update the current pose based on the buffers of stored data
    """
    # combine GPS and compass into [x, y, theta]
    self.globalPose = np.concatenate(
        [self.gpsCallback(), self.compassCallback()], axis=0)
    self.localPose = self.stereoPoseCallback()

    self.globalBuffer[self.frameid, :] = self.globalPose
    self.localBuffer[self.frameid, :] = self.localPose

    cv2.imwrite("image/" + str(self.frameid) + ".png", devhub.depthImage)
    
    self.frameid = (self.frameid + 1) % self.bufmax

  def run(self):
    while True:
      self.update()

  def getPose(self):
    """ 
    Returns:
    - pose (x, y, theta)
    """

    # calculate mean theta offset
    thetaOffset = np.mean(self.globalBuffer[:,2:] - self.localBuffer[:,2:])

    # Use the angle to rotate the local position and obtain the offset
    s = math.sin(math.radians(thetaOffset))
    c = math.cos(math.radians(thetaOffset))
    newpos = np.dot(self.localBuffer[:,:2], np.array([[c, -s], [s, c]]))
    posOffset = [np.mean(self.globalBuffer[:,0] - newpos[:,0]),
                 np.mean(self.globalBuffer[:,1] - newpos[:,1])]

    return np.concatenate((newpos[self.frameid] + posOffset,
      [self.localBuffer[0][2] + thetaOffset]))

  ## Callbacks ##
  def assignGPSCallback(self, fn):
    self.gpsCallback = fn

  def assignCompassCallback(self, fn):
    self.compassCallback = fn

  def assignStereoPoseCallback(self, fn):
    self.stereoPoseCallback = fn
