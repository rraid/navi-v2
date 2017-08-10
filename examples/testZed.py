import sys
sys.path.append("../device/")
import zed
import numpy as np
import cv2
import time

zed.open()

while True:

  frame = zed.grabDepthFrame()
  if type(frame) == type(None):
    time.sleep(0.01)
    continue

  cv2.imshow("zed depth", np.clip(frame / 10.0, 0.0, 1.0))
  cv2.waitKey(10)

zed.close()
