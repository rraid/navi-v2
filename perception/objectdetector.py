import chili
import cv2
import numpy as np
import math
import time
from threading import Thread

class ObjectDetector(Thread):
  def __init__(self):
    Thread.__init__(self)
    self.tags = dict()
    self.stopstate = False
    self.loaded = False
    self.rgbImage = None
    self.depthImage = None
    self.processing = False
    self.lastTime = time.time()

  def setObjects(self, objectNames):
    for obj in objectNames:
      self.tags.update({obj:None})

  def observe(self, rgbImage, depthImage):
    grayImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2GRAY)
    allTags = chili.find(grayImage)
    for tag in self.tags:
      corners = np.array(allTags.get(tag))
      midpoint = (((corners[0,:] + corners[1,:]) / 2.0) + \
          ((corners[2,:] + corners[3,:]) / 2.0)) / 2.0
      ##Couldnt think of a good way to calculate the sum of any quadrilatiral,
      ##so i just returned the depth at the midpoint
      depth = depthImage[int(midpoint[1]), int(midpoint[0])]
      self.tags[tag] = (midpoint, depth)

  def predict(self):
    return self.tags, self.lastTime

  def setImages(self, rgbImage, depthImage):
    self.rgbImage = rgbImage
    self.depthImage = depthImage
    self.processing = True

  def run(self):
    while not self.stopstate:
      currTime = time.time()
      if currTime - self.lastTime < 0.1:
        continue
      self.lastTime = currTime
      if type(self.rgbImage) != type(None) and len(self.rgbImage) != 0 and \
         type(self.depthImage) != type(None) and len(self.depthImage) != 0:
        self.observe(np.copy(self.rgbImage), np.copy(self.depthImage))
        print("[OBJDETECT] process time:", time.time() - self.currTime)

  def stop(self):
    self.stopstate = True
