#!/usr/bin/env python2
import sys
sys.path.append("../device/")
import devhub
import numpy as np
import cv2
import time
import signal

stopTest = False

def stopsigHandler(signo, frame):
  stopTest = True
  devhub.stop()
  sys.exit(0)


if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print "Press Ctrl+C to stop"
  devhub.init()

  while not stopTest:

    frame = devhub.getZedFrame()
    if type(frame) == type(None):
      time.sleep(0.01)
      continue
    
    cv2.imshow("zed depth", frame)  
    
    #cv2.imshow("zed depth", np.clip(frame / 10.0, 0.0, 1.0))
    cv2.waitKey(1)
