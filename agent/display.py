import numpy as np
from skimage.draw import circle, line

def displayVelocities(left, right, shape=(200, 200)):
  # NOTE: For display purposes only! To use as a distribution, call np.flipud
  # on the output
  r = shape[0]
  c = shape[1]
  # draw a circle representing the robot shape
  rr1, cc1 = circle(r / 2, c / 2, r / 10, shape)
  # draw lines representing the robot left wheel
  rr2, cc2 = line(r / 2, c / 4, r / 2 + int(left * 100), c / 4)
  rr3, cc3 = line(r / 2, c * 3 / 4, r / 2 + int(right * 100), c * 3 / 4)
  diff = (right - left) * 100
  avg = (right + left / 2) * 100
  rr4, cc4 = line(r / 2, c / 2, r / 2 + int(avg), c / 2 + int(diff))
  # push the drawings onto a color image
  img = np.zeros((r, c, 3), dtype=np.uint8)
  img[rr1, cc1, :2] = 255
  img[rr2, cc2, 1:] = 255
  img[rr3, cc3, 1:] = 255
  img[rr4, cc3, 1] = 255
  return np.flipud(img)
