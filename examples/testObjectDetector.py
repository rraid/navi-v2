import sys
sys.path.append("../perception/")
import objectdetector
import cv2
import numpy as np
import time

if __name__ == "__main__":
  colorImage = cv2.imread("sampleImage.png", cv2.IMREAD_COLOR)
  objdetect = objectdetector.ObjectDetector()
  objdetect.setObjects([10, 11, 12]) # id's of the chilitags
  objdetect.start()

  depthImage = np.reshape(np.arange(0, np.prod(colorImage.shape[:2]), 1),
      colorImage.shape[:2])
  objdetect.setImages(colorImage, depthImage)
  time.sleep(0.25) # this will have to be changed one day to a promise with timeout
  tags = objdetect.predict()
  print(str(tags))
  objdetect.stop()
