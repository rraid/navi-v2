import sys
sys.path.append("../device/")
import zed
import numpy as np
import cv2
import time

zed.open()

while True:
  temp = zed.getMeshData()
  if temp != None:
    print "This is the Data: ", len(temp[0]), len(temp[1])
  time.sleep(.01)
  

zed.close()
