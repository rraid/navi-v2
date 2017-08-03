import numpy as np
import serial
import rospy as ros
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from threading import Thread, Event, Lock
from cv_bridge import CvBridge
import zed
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
#arduinoUno = None
rosReader = None

def getSonarReadings():
  return sonarReadings

def getGPSReadings():
  return (latitude, longitude)

def getLidarReadings():
  global lidarReadings
  return lidarReadings

def getZedReadings():
  global depthColumns
  depth_image = zed.grabDepthFrame()
  dataMid = depth_image.shape[0]/2
  subImage = depth_image[dataMid-50:dataMid+50,:]
  depthColumns = np.amin(subImage, axis=0)
  return depthColumns

def getCompassReadings():
  global heading
  return heading
  
arduinoWrite = serial.Serial("/dev/ttyACM0" ,9600)
def setMotorVelocity(left, right):
  global motorVelocity
  #motorVelocity = [left,right]
  writeBuff = "["+ str(int(left*100)) + "," + str(int(right*100)) + "]\n"
  arduinoWrite.write(writeBuff)
    

def lidarCallbackHandler(scan):
  global lidarReadings
  lidarReadings = np.array(scan.ranges)
  # multiplying by to to convert to .5 meter

class ArduinoListener(Thread):

  def __init__(self, idName):
    print "Ard. thread Started"
    Thread.__init__(self)
    self.arduino = serial.Serial("/dev/" + idName ,9600)
    self.stopstate = False

  def run(self):
    while not self.stopstate:
      self.readSerial()
      #time.sleep(0.1) # 10 MHz refresh

  def readSerial(self):
    #global sonarReadings
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
      
        try:
          buff = eval(buff.strip())
          (latitude,longitude,heading) = buff
        except:
          print "error"
  def stop(self):
    self.stopstate = True

class ROSListener(Thread):
  def __init__(self):
    print "Ros thread Started"
    Thread.__init__(self)
    ros.init_node("DeviceListener", anonymous=True)
    ros.Subscriber("/scan", 
        LaserScan, lidarCallbackHandler)
  def run(self):
    ros.spin() # blocks already

def init():
  global arduinoMega
  global arduinoUno
  zed.open()
  arduinoMega = ArduinoListener("ttyACM1")
  arduinoMega.start()
  #arduinoUno = ArduinoListener("ttyACM0")
  #arduinoUno.start()
  rosReader = ROSListener()
  rosReader.start()

def stop():
  global arduinoMega
  global arduinoUno
  arduinoMega.stop()
  arduinoMega.join()
  #arduinoUno.stop()
  #arduinoUno.join()
  zed.close()
  ros.signal_shutdown("Ending Process")
  time.sleep(1)
  sys.exit()
