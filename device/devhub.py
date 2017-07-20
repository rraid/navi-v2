import numpy as np
import serial
import rospy as ros
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from threading import Thread, Event
import time

# Globals
lidarReadings = []
depthColumns = []
sonarReadings = [0,0,0,0,0,0,0,0,0]
latitude = None
longitude = None

motorVelocity = [0,0]

arduinoDevice = None


def getSonarReadings():
  return sonarReadings

def getGPSLocation():
  return (latitude, longitude)

def getLidarReadings():
  global lidarReadings
  
  return lidarReadings

def getZEDDepthColumns():
  global depthColumns
  return depthColumns

def getButtonReadings():
  ##### CODE HERE #####
  raise NotImplementedError
  return {}

def setMotorVelocity(left, right):
  motorVelocity = [left,right]

def lidarCallbackHandler(scan):
  global lidarReadings
  lidarReadings = scan.ranges

def zedCallbackHandler(frame):
  global depthColumns
  depthColumns = frame.data

class ArduinoListener(Thread):

  #def __init__(self, idList):
  def __init__(self):
    print "Thread Started"
    Thread.__init__(self)
    #self.idList = idList
    self._stop_event = Event()
    self.arduino = serial.Serial('/dev/ttyACM0',9600)

  def run(self):
    while not self.stopped():
      self.readSerial()
      #self.writeSerial()
      time.sleep(0.1) # 10 MHz refresh

  def readSerial(self):
    global sonarReadings
    global latitude
    global longitude
    
    if self.arduino.in_waiting > 0:
      buff = None;
      buff = self.arduino.read(self.arduino.in_waiting)
      prev = 1
      count = 0
      if buff[0] == '[':
        for ptr in range(len(buff)):
          if buff[ptr] == ',' or buff[ptr] == ']':
            if count < 9:
              sonarReadings[count] = float(buff[prev:ptr])
              count += 1
              prev = ptr + 1
            elif count == 9:
              latitude = float(buff[prev:ptr])
              count += 1
              prev = ptr + 1
            else:
              longitude = float(buff[prev:ptr])
            
  def writeSerial(self):
    global motorVelocity
    writeBuff = "[" + motorVelocity + "]\n"
    self.arduino.write(writeBuff)
    
  def stop(self):
    self._stop_event.set()

  def stopped(self):
    return self._stop_event.is_set()

def init():
  arduinoDevice = ArduinoListener()
  arduinoDevice.start()
  ros.init_node("DeviceListener", anonymous=True)
  ros.Subscriber("/lidar/scan", 
      LaserScan, lidarCallbackHandler)
  ros.Subscriber("/zed/depth/depth_registered", 
      Float32MultiArray, zedCallbackHandler)
  ros.spin()

def stop():
  arduinoDevice.stop()
  arduinoDevice.join()
