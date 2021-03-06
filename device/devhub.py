import numpy as np
import serial
from threading import Thread, Event, Lock
import zed
import time
import struct
import cv2
import sys
import math


# Globals
depthColumns = []
colorImage = []
depthImage = []
image = []
latitude = None
longitude = None
heading = None

rot = np.array([0,0,0])
tsln = np.array([0,0,0])


## Left, Right
motorVelocity = [0,0]

## Y, X
mapShape = [290,315]
## Y, X
mapBL = [40.52119,-74.462088]
mapTR = [40.523815,-74.458337]

arduinoMega = None
arduinoUno = None
rosReader = None

def getGPSReadings():
  values =  (latitude, longitude)
  if type(values) == type(None) or values[0] == None:
    return None
  x = ((longitude - mapBL[1]) / (mapTR[1] - mapBL[1])) * mapShape[1]
  y = ((latitude - mapBL[0]) / (mapTR[0] - mapBL[0])) * mapShape[0]  
  return x,y

def getZedReadings():
  global depthColumns
  global depthImage
  depthImage = zed.grabDepthFrame()
  if type(depthImage) == type(None):
    return None
  dataMid = depthImage.shape[0]/2
  subImage = depthImage[dataMid-50:dataMid+50,:]
  depthColumns = np.amin(subImage, axis=0)
  return depthImage

def getZedFrame():
  global image
  image = zed.grabFrame()
  return image

def getZedPose():
  return zed.getPose()


def getCompassReadings():
  global heading
  if heading == None:
    return None
  return (heading + 24) % 360
  
def setMotorVelocity(left, right):
  global motorVelocity
  left = np.clip(left,-1.0,1.0)
  right = np.clip(right,-1.0,1.0)
  motorVelocity = [left,right]
  
def getMotorVelocity():
  return motorVelocity

class ArduinoListener(Thread):

  def __init__(self, idName):
    print "Ard. listener Started"
    Thread.__init__(self)
    self.arduino = serial.Serial("/dev/" + idName ,57600)
    self.stopstate = False

  def run(self):
    while not self.stopstate:
      self.readSerial()
      #time.sleep(0.1) # 10 MHz refresh

  def readSerial(self):
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
          print "Failed Serial Read"

  def stop(self):
    self.stopstate = True

class ArduinoPublisher(Thread):
  
  def __init__(self, idName):
    print "Ard. publisher Started"
    Thread.__init__(self)
    self.arduino = serial.Serial("/dev/" + idName ,57600)
    self.stopstate = False

  def run(self):
    while not self.stopstate:
      self.writeSerial()
      time.sleep(0.1) # 10 MHz refresh

  def writeSerial(self):
    global motorVelocity
    left = motorVelocity[0]
    right = motorVelocity[1]
    #print left,right
    writeBuff = "["+ str(int(left*40)) + "," + str(int(right*40)) + "]\n"
    self.arduino.write(writeBuff)

  def stop(self):
    self.stopstate = True

def init():
  global arduinoMega
  global arduinoUno
  zed.open()
  arduinoMega = ArduinoListener("ttyACM0")
  arduinoMega.start()
  #arduinoUno = ArduinoPublisher("ttyACM0")
  #arduinoUno.start()


def stop():
  global arduinoMega
  global arduinoUno
  arduinoMega.stop()
  arduinoMega.join()
  #arduinoUno.stop()
  #arduinoUno.join()
  zed.close()
  time.sleep(1)
  sys.exit()
