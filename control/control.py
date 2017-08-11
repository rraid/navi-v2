import math
import numpy as np
import time

wheel_radius = 0.14 # meters
robot_radius = 0.343 # meters
RPM = 1.3


def getTrajectory(pose, k, path): # no more potential field
  if len(path) == 0:
    return []
  # find sse
  dist = np.linalg.norm(pose[:2] - path, axis=1)
  print dist
  #ind = np.argmin(dist)
  #print ind
  minind = 0
  minitem = 100000000000.0
  for i in range(dist.shape[0]):
    if dist[i] < minitem:
      minind = i
      minitem = dist[i]
  ind = minind
  return list(np.array(path[ind:min(len(path), ind + k)]))

def moveForward(distance, dt):
  K = 2.0 * math.pi * wheel_radius * RPM / 60.0
  F = distance / dt
  vel = np.clip(F * 2.0 / K, -1.0, 1.0)
  return vel, vel

def spinAround(dTheta, dt):
  K = 2.0 * math.pi * wheel_radius * RPM / 60.0
  T = dTheta * math.pi / (180.0 * dt)
  vel = np.clip(T * 2.0 * robot_radius / K, -1.0, 1.0)
  return -vel, vel

def followTraj(pose, path, targetAngle, k=3):
  if len(path) == 0:
    return 0.0, 0.0
  if len(path) == 1 or \
      np.linalg.norm(np.array(path[-1]) - np.array(pose[:2])) < 0.5:
    dTheta = targetAngle - pose[2]
    dTheta %= 360.0
    if dTheta > 180.0:
      dTheta -= 360.0
    return spinAround(dTheta, 1.0)
  pose = np.array(pose)
  path = np.array(path)

  # get the distance that is needed to traverse
  dist = np.linalg.norm(pose[:2] - path, axis=1)
  minind = 0
  minitem = 100000000000.0
  for i in range(dist.shape[0]):
    if dist[i] < minitem:
      minind = i
      minitem = dist[i]

  # k steps ahead, find vector
  ind = min(minind + k, len(path)-1)
  target = path[ind,:]
  d = target - pose[:2]

  # get the angle
  dTheta = math.atan2(d[1], d[0]) - (pose[2] * math.pi / 180.0)
  print dTheta
  dTheta %= 360.0
  if dTheta > 180.0:
    dTheta -= 360.0

  # get the distance
  dist = np.linalg.norm(d)

  # grab the speeds
  left1, right1 = moveForward(dist, 1.0)
  left2, right2 = spinAround(dTheta, 1.0)
  left = 1.0 * left1 + 4.0 * left2
  right = 1.0 * right1 + 4.0 * right2

  # renormalize
  normalization = max(abs(left), abs(right))
  if normalization == 0.0:
    return 0.0, 0.0
  return left / normalization, right / normalization
