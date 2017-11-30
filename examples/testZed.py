#!/usr/bin/env python3
import sys
sys.path.append("../device/")
import devhub
import numpy as np
import cv2
import time
import signal
import pyzed.core as core
import math


mat = None
rot = None
tsln = None
stopTest = False


def stopsigHandler(signal, frame):
  stopTest = True
  devhub.stop()
  sys.exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print("Press Ctrl+C to stop")
  devhub.init()
  cv2.waitKey(100)
  while not stopTest:

    mat,rot,tsln = devhub.getZedReadings()

    if type(mat) == type(None):
      continue
    if type(mat) != np.ndarray:
      time.sleep(0.01)
      continue
    print("Rotation: ", rot)
    print("Translation: ", tsln)

    cv2.imshow("zed depth", mat)
    cv2.waitKey(1)
  
