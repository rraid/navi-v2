import numpy as np
import pyserial
import rospy as ros
from std_msgs.msg import Float32MultiArray
from sensor_msgs import LaserScan
from threading import Thread, Event
import time

# Globals for lidar and zed access
lidarReadings = []
depthColumns = []
arduinoDevice = None

def getSonarReadings():
  ##### CODE HERE #####
  raise NotImplementedError
  return {}

def getLidarReadings():
  # Complete code in lidarCallbackHandler()
  global lidarReadings
  return lidarReadings

def getZEDDepthColumns():
  # Complete code in zedCallbackHandler()
  global depthColumns
  return depthColumns

def getGPSLocation():
  ##### CODE HERE #####
  raise NotImplementedError
  return (0.0, 0.0)

def getButtonReadings():
  ##### CODE HERE #####
  raise NotImplementedError
  return {}

def setMotorVelocity(left, right):
  ##### CODE HERE #####
  raise NotImplementedError

def lidarCallbackHandler(scan):
  ##### CODE HERE #####
  global lidarReadings
  raise NotImplementedError

def zedCallbackHandler(frame):
  ##### CODE HERE #####
  global depthColumns
  raise NotImplementedError

class ArduinoListener(Thread):
  ##### Edit this class #####
  def __init__(self, idList):
    Thread.__init__(self)
    self.idList = idList
    self._stop_event = Event()

  def run(self):
    while not self.stopped():
      time.sleep(0.1) # 10 MHz refresh

  def stop(self):
    self._stop_event.set()

  def stopped(self):
    return self._stop_event.is_set()

def init():
  arduinoDevice = ArduinoListener()
  arduinoDevice.start()
  rospy.init_node("DeviceListener", anonymous=True)
  rospy.subscriber("/lidar/scan", \
      LaserScan, lidarCallbackHandler)
  rospy.subscriber("/zed/depth/depth_registered", \
      Float32MultiArray, zedCallbackHandler)
  rospy.spin()

def stop():
  arduinoDevice.stop()
  arduinoDevice.join()
