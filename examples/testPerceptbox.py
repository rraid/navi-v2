#!/usr/bin/env python2
import sys
sys.path.append("../perception/")
import numpy as np
sys.path.append("../device/")
import devhub
import cv2
import pickle
import signal

mat = None
stopTest = False

buff = []

def stopsigHandler(signal, frame):
  global buff
  
  # save everything
  with open("dataSet/global.pkl", "wb") as fp:
    pickle.dump(buff, fp)
  stopTest = True
  devhub.stop()
  sys.exit(0)

if __name__ == "__main__":

  global buff
  global mat
  global stopTest
  
  
  signal.signal(signal.SIGINT, stopsigHandler)
  print("Press Ctrl+C to stop")
  devhub.init()

  # now that the box has been anchored, we can attempt to test out the localizer
  while not stopTest:

    mat = devhub.getZedReadings()
    if type(mat) == type(None):
      continue
    if type(mat) != np.ndarray:
      time.sleep(0.01)
      continue
    cv2.imshow("zed depth", mat)
    cv2.waitKey(1)
    
    idx = len(buff)
    buff.append("zedFrame" + str(idx))
    np.save("dataSet/zedFrame" + str(idx),mat)
    
    
    #gps
    #compass
    #zedPose
    



