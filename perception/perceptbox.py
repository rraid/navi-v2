import sys
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
  def __init__(self, gridimg, initial_pose=None,
      gpsCallback=None,
      compassCallback=None,
      stereoPoseCallback=None):
    """ Constructor
    Arguments:
    - gridimg: numpy array of dimensionality (rows, cols)
    - initial_pose: (x, y, theta) initial pose of the robot
    - gpsCallback: a function which returns the gps reading (x, y)
    - compassCallback: a function which returns the compass reading theta
    - stereoPoseCallback: a function which returns (x, y, theta)
    """
    assert(type(gpsCallback) != type(None))
    assert(type(compassCallback) != type(None))
    assert(type(stereoPoseCallback) != type(None))

    Thread.__init__(self)
    # shape = (y, x)
    self.gridimg = gridimg
    if type(initial_pose) == type(None):
      # (x, y, theta)
      self.initial_pose = (self.gridimg.shape[1] / 2, \
          self.gridimg.shape[0] / 2, 0)
    else:
      self.initial_pose = initial_pose

    # start by defining a buffer to store the GPS and the robot
    self.autoCalibrate = True
    self.bufmax = 2000 # max 2000 readings
    self.globalBuffer = np.zeros((0, 3))
    self.localBuffer = np.zeros((0, 3))

    # learning an initial transformation
    self.transform = np.array([0, 0, 0], dtype=np.float32)

    # create a counter for current frame index
    self.frameid = 0
    self.gpsCallback = gpsCallback
    self.compassCallback = compassCallback
    self.stereoPoseCallback = stereoPoseCallback
    self.globalPose = None
    self.localPose = None
    self.calibratedPose = np.array([0, 0, 0], dtype=np.float32)

  def update(self):
    """ Update the current pose based on the buffers of stored data
    """
    # update the values inside the buffers
    self.globalPose = np.concatenate(
        [self.gpsCallback(), self.compassCallback()], axis=1)
    self.localPose = self.stereoPoseCallback()
    if self.globalPose.shape[0] < self.bufmax:
      self.globalBuffer = np.concatenate(self.globalBuffer,
          np.reshape(self.globalPose, (1, 3)))
      self.localBuffer = np.concatenate(self.localBuffer,
          np.reshape(self.localPose, (1, 3)))
    else:
      self.globalBuffer[self.frameid, :] = self.globalPose
      self.localBuffer[self.frameid, :] = self.localPose

    # assume that the compass is always right (later on use sin and cos rules)
    if self.autoCalibrate:
      thetaOffset = np.mean(self.globalBuffer[:,2:] - self.localBuffer[:,2:])
      # use the transformation to obtain the offset
      s = math.sin(math.radians(thetaOffset))
      c = math.cos(math.radians(thetaOffset))
      newpos = np.dot(self.localBuffer[:,:2], np.array([[c, -s], [s, c]]))
      # grab the offset from the matrix transform
      posOffset = np.mean(self.globalBuffer[:,:2] - newpos)
      self.transform[:2] = posOffset
      self.transform[2] = thetaOffset
    else:
      # set the offsets to the old values
      thetaOffset = self.transform[2]
      posOffset = self.transform[:2]
      s = math.sin(math.radians(thetaOffset))
      c = math.cos(math.radians(thetaOffset))
      newpos = np.dot(self.localPose[:2], np.array([[c, -s], [s, c]]))

    # use the transform to estimate the calibrated pose
    self.calibratedPose = np.concatenate(
        [newpos[self.frameid, :], [self.localBuffer[2] + thetaOffset]])
    self.frameid = (self.frameid + 1) % self.bufmax

  def run(self):
    while True:
      self.update()

  def getPose(self):
    """
    Returns:
    - pose (x, y, theta)
    """
    return np.copy(self.calibratedPose)

  def getMap(self, radius=10.0, scaling=0.1):
    """
    Get the distribution of the map, given a radius
    """
    # for now just test the localizer to see if it works
    pass

  def anchor(self):
    """ Turn off automatic calibration and use the current set of readings for
        the offset.
    """
    self.autoCalibrate = False

  ## Callbacks ##
  def assignGPSCallback(self, fn):
    self.gpsCallback = fn

  def assignCompassCallback(self, fn):
    self.compassCallback = fn

  def assignStereoPoseCallback(self, fn):
    self.stereoPoseCallback = fn
