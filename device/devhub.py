import numpy as np
import serial
# import rospy as ros
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import LaserScan
from threading import Thread, Event, Lock
import pyzed.camera as zcam
import pyzed.defines as sl
import pyzed.types as tp
import pyzed.core as core
import time
import struct
import cv2
import sys
import math


# Globals
camera_pose = zcam.PyPose()
py_translation = core.PyTranslation()
depthColumns = core.PyMat()
colorImage = core.PyMat()
depthImage = []
depthMat = core.PyMat()
latitude = None
longitude = None
heading = None
rot = np.array([0,0,0])
tsln = np.array([0,0,0])

zed = None
runtime_parameters = None

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
  global zed
  global runtime_parameters
  global depthColumns
  global depthImage
  global depthMat
  global colorImage
  global camera_pose
  global py_translation
  global rot
  global tsln
  
  #Make sure zed is working
  if zed.grab(runtime_parameters) == tp.PyERROR_CODE.PySUCCESS:
    #for zed pose
    tracking_state = zed.get_position(camera_pose)
    #for zed depth
    zed.retrieve_image(colorImage, sl.PyVIEW.PyVIEW_LEFT)
    zed.retrieve_measure(depthMat, sl.PyMEASURE.PyMEASURE_DEPTH)
    
    #get depth stuff
    if type(depthImage) == type(None):
      return None
    #dataMid = depthImage.shape[0]/2
    #subImage = depthImage[dataMid-50:dataMid+50,:]
    #depthColumns = np.amin(subImage, axis=0)
    depthImage = depthMat.get_data()/20000
    
    #get pose stuff
    if tracking_state == sl.PyTRACKING_STATE.PyTRACKING_STATE_OK:
      rotation = camera_pose.get_rotation_vector()
      rot = np.degrees(np.array([round(rotation[0], 2),round(rotation[1], 2),round(rotation[2], 2)]))

      translation = camera_pose.get_translation(py_translation)
      tsln = np.array([round(translation.get()[0], 2),round(translation.get()[1], 2),round(translation.get()[2], 2)])
    else:
      rot = np.array([0,0,0])
      tsln = np.array([0,0,0])
      
    print(type(rot) , type(tsln))
    return depthImage, rot, tsln
    
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
    print("Ard. listener Started")
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
          print("Failed Serial Read")

  def stop(self):
    self.stopstate = True

class ArduinoPublisher(Thread):
  
  def __init__(self, idName):
    print("Ard. publisher Started")
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
    #print(left,right)
    writeBuff = "["+ str(int(left*40)) + "," + str(int(right*40)) + "]\n"
    self.arduino.write(writeBuff)

  def stop(self):
    self.stopstate = True

def init():
  global arduinoMega
  global arduinoUno
  #arduinoMega = ArduinoListener("ttyACM0")
  #arduinoMega.start()
  #arduinoUno = ArduinoPublisher("ttyACM0")
  #arduinoUno.start()
  global zed
  global runtime_parameters
  global camera_pose
  global py_translation
  
  
  
  # Create a PyZEDCamera object
  zed = zcam.PyZEDCamera()

  # Create a PyInitParameters object and set configuration parameters
  init_params = zcam.PyInitParameters()
  init_params.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_PERFORMANCE  # Use PERFORMANCE depth mode
  init_params.coordinate_units = sl.PyUNIT.PyUNIT_MILLIMETER  # Use milliliter units (for depth measurements)
  init_params.coordinate_system = sl.PyCOORDINATE_SYSTEM.PyCOORDINATE_SYSTEM_RIGHT_HANDED_Y_UP
  init_params.camera_fps = 60  # Set fps at 60
  
  err = zed.open(init_params)
  if err != tp.PyERROR_CODE.PySUCCESS:
    exit(1)
    
  runtime_parameters = zcam.PyRuntimeParameters()
  runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_STANDARD  # Use STANDARD sensing mode
  camera_pose = zcam.PyPose()
  py_translation = core.PyTranslation()


def stop():
  global arduinoMega
  global arduinoUno
  #arduinoMega.stop()
  #arduinoMega.join()
  #arduinoUno.stop()
  #arduinoUno.join()
  
  zed.close()

  
  time.sleep(1)
  sys.exit()
