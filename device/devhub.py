import numpy as np
import serial
import rospy as ros
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from threading import Thread, Event
import time
import struct
import cv2
import sys

# Globals
lidarReadings = []
depthColumns = []
sonarReadings = [0,0,0,0,0,0,0,0,0]
latitude = None
longitude = None

motorVelocity = [0,0]

arduinoDevice = None
rosReader = None

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
  print lidarReadings

def zedCallbackHandler(frame):
  global depthColumns
  depthImage = np.fromstring(frame.data, dtype=np.float32)
  cv2.imshow('img',np.reshape(depthImage,(frame.height,frame.width)))
  cv2.waitKey(30)
  #dataMid = len(depthImage)/2
  #print frame.data[0:10]
  #time.sleep(10)
  #subImage = frame.data[dataMid-50:dataMid+50,:]
  #depthColumns = np.minimum(subImage, axis=0)


class ArduinoListener(Thread):

  #def __init__(self, idList):
  def __init__(self):
    print "Ard. thread Started"
    Thread.__init__(self)
    #self.idList = idList
    self.arduino = serial.Serial('/dev/ttyACM0',9600)
    self.stopstate = False

  def run(self):
    while not self.stopstate:
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
    self.stopstate = True

class ROSListener(Thread):

  def __init__(self):
    print "Ros thread Started"
    Thread.__init__(self)
    ros.init_node("DeviceListener", anonymous=True)
    ros.Subscriber("/lidar/scan", 
        LaserScan, lidarCallbackHandler)
    ros.Subscriber("/zed/depth/depth_registered", 
        Image, zedCallbackHandler)

  def run(self):
    while not ros.is_shutdown():
      ros.spin()

def init():
  #arduinoDevice = ArduinoListener()
  #arduinoDevice.start()
  rosReader = ROSListener()
  rosReader.start()

def stop():
  print "STOP SIGNAL RECVD"
  #arduinoDevice.stop()
  #arduinoDevice.join()
  ros.signal_shutdown("Ending Process")
  time.sleep(1)
  sys.exit()
