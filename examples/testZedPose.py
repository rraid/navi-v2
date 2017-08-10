import sys
sys.path.append("../device/")
import zed
import numpy as np
import cv2
import time

zed.open()

while True:
  print zed.getPose()
  time.sleep(.01)
  

zed.close()
