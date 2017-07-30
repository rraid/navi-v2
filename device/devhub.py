import numpy as np
import serial
import rospy as ros
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from threading import Thread, Event
from cv_bridge import CvBridge
import time
import struct
import cv2
import sys

# Globals
lidarReadings = []
depthColumns = []
colorImage = []
depthImage = []
sonarReadings = [0,0,0,0,0,0,0,0,0]
latitude = None
longitude = None
heading = None

motorVelocity = [0,0]

arduinoMega = None
arduinoUno = None
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

def getCompassOrientation():
  global heading
  return heading

def setMotorVelocity(left, right):
  motorVelocity = [left,right]

def lidarCallbackHandler(scan):
  global lidarReadings
  lidarReadings = scan.intensities


def zedDepthCallbackHandler(frame):
  global depthColumns
  global depthImage
  depthImage = np.reshape(np.fromstring(frame.data, dtype=np.float32),(frame.height,frame.width))
  dataMid = frame.height/2
  subImage = depthImage[dataMid-50:dataMid+50,:]
  depthColumns = np.amin(subImage, axis=0)
  #cv2.imshow('img',depthColumns)
  #cv2.waitKey(30)
  
def zedImageCallbackHandler(frame):
  global colorImage
  colorImage = np.asarray(CvBridge().imgmsg_to_cv2(frame))
  #cv2.imshow('img',colorImage)
  #cv2.waitKey(30)

class ArduinoListener(Thread):

  def __init__(self, idName, control):
    print "Ard. thread Started"
    Thread.__init__(self)
    self.arduino = serial.Serial("/dev/" + idName ,9600)
    self.stopstate = False
    self.control = control

  def run(self):
    while not self.stopstate:
      self.readSerial()
      if(self.control):
        self.writeSerial()
      #time.sleep(0.1) # 10 MHz refresh

  def readSerial(self):
    global sonarReadings
    global latitude
    global longitude
    global heading
    
    if self.arduino.in_waiting > 0:
      buff = None;
      buff = self.arduino.readline()
      prev = 1
      count = 0
      devType = None
      if buff[0] == '[':
        for ptr in range(len(buff)):
          if buff[ptr] == ',' or buff[ptr] == ']':
            if count == 0:
              devType = buff[prev:ptr]
              count += 1
              prev = ptr + 1
            elif devType == "mega":
              if count < 10:
                try:
                  sonarReadings[count-1] = float(buff[prev:ptr])
                except ValueError:
                  print "Serial Read Incorrectly"
                  break
                count += 1
                prev = ptr + 1
              elif count == 10:
                latitude = float(buff[prev:ptr])
                count += 1
                prev = ptr + 1
              elif count ==11:
                longitude = float(buff[prev:ptr])
            elif devType == "uno":
              try:
                heading = float(buff[prev:ptr])
              except ValueError:
                print "Serial Read Incorrectly"
                break
            else:
              print "Couldnt identify device"
            
  def writeSerial(self):
    global motorVelocity
    writeBuff = "[" + str(int(motorVelocity[0]*100)) + "," + str(int(motorVelocity[1]*100)) + "]\n"
    self.arduino.write(writeBuff)
    
  def stop(self):
    self.stopstate = True

class ROSListener(Thread):
  def __init__(self):
    print "Ros thread Started"
    Thread.__init__(self)
    ros.init_node("DeviceListener", anonymous=True)
    ros.Subscriber("/scan", 
        LaserScan, lidarCallbackHandler)
    ros.Subscriber("/zed/depth/depth_registered", 
        Image, zedDepthCallbackHandler)
    ros.Subscriber("/zed/left/image_rect_color", 
        Image, zedImageCallbackHandler)
  def run(self):
    ros.spin() # blocks already

def init():
  global arduinoMega
  global arduinoUno
  #arduinoMega = ArduinoListener("ttyACM3", False)
  #arduinoMega.start()
  #arduinoUno = ArduinoListener("ttyACM1", True)
  #arduinoUno.start()
  rosReader = ROSListener()
  rosReader.start()

def stop():
  global arduinoMega
  global arduinoUno
  arduinoMega.stop()
  arduinoMega.join()
  arduinoUno.stop()
  arduinoUno.join()
  ros.signal_shutdown("Ending Process")
  time.sleep(1)
  sys.exit()
