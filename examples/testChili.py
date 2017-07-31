import sys
sys.path.append("../perception/")
import chili
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

if __name__ == "__main__":
  colorImage = cv2.imread("sampleImage.png", cv2.IMREAD_GRAYSCALE)
  tags = chili.find(colorImage)
  print(str(tags))
