import sys
sys.path.append("../control")
import control
import math
import numpy as np
from scipy.misc import imresize

path = np.array([
  [0, 1],
  [0, 2],
  [0, 3],
  [0, 4],
  [0, 5],
  [0, 6],
  [0, 7],
  [0, 8],
  [0, 9],
  [0, 10],
  [0, 11],
  [0, 12],
  [0, 13],
  [0, 14],
  [0, 15],
  [0, 16],
  [0, 17],
  [0, 18],
  [0, 19],
  [0, 20]])

path = np.concatenate((path, np.fliplr(path)), axis=0)
path += np.array([10, 10])
path[20:,1] += 20

grid = np.zeros((64, 64))
grid[path[:,1], path[:,0]] = 0.5

import cv2

cv2.imshow("grid", np.flipud(grid) * 255)
cv2.waitKey(0)

import time

robotPos = np.array([10.0, 10.0, 90.0])
path = path.tolist()

def getMotion(left, right):
  vel = 2.0 * math.pi * control.wheel_radius * control.RPM / 60.0
  l = vel * left
  r = vel * right
  v = (l + r) / 2.0
  w = (r - l) / (2.0 * control.robot_radius) * 180.0 / math.pi
  return v, w

while True:
  time.sleep(0.1)
  left, right = control.followTraj(robotPos, path, -90.0, 2)
  print "LEFT, RIGHT:", left, right
  v, w = getMotion(left, right)
  print "V, W:", v, w
  dxdy = np.array([v * math.cos(robotPos[2] * math.pi / 180.0),
    v * math.sin(robotPos[2] * math.pi / 180.0), w])
  print "DX, DY:", dxdy
  robotPos += dxdy
  robot = np.copy(grid)
  x = int(round(robotPos[0]))
  y = int(round(robotPos[1]))
  robot[y-2:y+3, x-2:x+3] = 1.0
  cv2.imshow("grid", imresize(np.flipud(robot) * 255, (256, 256), "nearest"))
  cv2.waitKey(1)
  print "POS:", robotPos
  print "=================================================================="
