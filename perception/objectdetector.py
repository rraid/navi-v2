import chilipy
import cv2
import numpy as np
import math
from threading import Thread

class ObjectDetector(Thread):
  def __init__(self):
    self.tags = dict()
    self.stopstate = False
    self.loaded = False
    self.rgbImage = None
    self.depthImage = None

  def setObjects(self, objectNames):
    for obj in objectNames:
      self.tags.update({obj:None})

  def observe(self, rgbImage, depthImage):
    grayImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2GRAY))
    allTags = chili.find(grayImage)
    for tag in self.tags:
      corners = numpy.array(allTags.get(tag))
      midpoint = (corners[2:4] - corners[0:2]) - (corners[6:8] - corners[4:6])
      ##Couldnt think of a good way to calculate the sum of any quadrilatiral,
      ##so i just returned the depth at the midpoint
      depth = depthImage[(midpoint[0],midpoint[1])]
      self.tags[tag] = numpy.array((midpoint,depth))

  def detect(self):
    return self.tags

  def setImages(self, rgbImage, depthImage):
    self.rgbImage = rgbImage
    seld.depthImage = depthImage

  def run(self):
    lastTime = time.time()
    while not self.stopstate:
      currTime = time.time()
      if currTime - lastTime < 0.1:
        pass
      lastTime = currTime
      if type(self.rgbImage) != type(None) and len(self.rgbImage) != 0 and \
         type(self.depthImage) != type(None) and len(self.depthImage) != 0:
        self.observe(np.copy(self.rgbImage), np.copy(self.depthImage))
        print "[OBJDETECT] process time:", time.time() - lastTime

  def stop(self):
    self.stopstate = True
