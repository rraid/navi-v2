import sys
sys.path.append("../device/")
import devhub
sys.path.append("../perception/")
import perception
sys.path.append("../control/")
import control
import planning
import signal
import serial
import pygame
import time
import cv2
import numpy as np

perceptor = None
localizer= None
zedOffset = None

## Get Path
path = []
with open("../examples/path.txt", "r") as fp:
  path.append(eval(fp.readline().strip()))
    
stopSig = False
def stopsigHandler(signo, frame):
  stopSig = True
  print("Halting agent")
  devhub.setMotorVelocity(0, 0)
  devhub.stop()
  time.sleep(1)
  sys.exit(0)
  
  
if __name__ == "__main__":
  
  signal.signal(signal.SIGINT, stopsigHandler)
  print("Press Ctrl+C to stop")
  
  ## Initalize Devhub
  devhub.init()
  # stop the motors from moving for now
  devhub.setMotorVelocity(0, 0)
  time.sleep(3)
  ##This is to have the robot map the area around it
  spin = control.spinAround(360,10)
  devhub.setMotorVelocity(spin[0],spin[1])
  time.sleep(10)
  ##Rotate to face north

  while not stopSig:
    
  
  

