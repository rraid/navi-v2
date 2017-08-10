import math
import numpy as np

wheel_radius = 0.14 # meters
robot_radius = 0.343 # meters
RPM = 1.3

def getTrajectory(pose, k, planner):
  traj = [(pose[0], pose[1])]
  for i in range(k):
    pose = planner.getNextState(pose)
    traj.append((pose[0], pose[1]))
  return traj

def approximateSlope(pose, traj):
  dists = [((t[1] - float(p[1])) ** 2 + (t[0] - float(p[0])) ** 2) ** 0.5 \
      for p, t in zip(pose, traj)]
  angles = [math.atan2(t[1] - float(p[1]), t[0] - float(p[0])) \
      for p, t in zip(pose, traj)]
  meanDist = sum(dists) / float(len(dists))
  meanAngle = sum(angles) / float(len(angles))
  return meanDist, meanAngle * 180.0 / math.pi

def moveForward(distance, dt):
  K = 2.0 * math.pi * wheel_radius * RPM / 60.0
  F = distance / dt
  vel = np.clip(F * 2.0 / K, -1.0, 1.0)
  return vel, vel

def spinAround(dTheta, dt):
  dTheta = dTheta * math.pi/180
  K = 2.0 * math.pi * wheel_radius * RPM / 60.0
  T = dTheta * math.pi / (180.0 * dt)
  vel = np.clip(T * 2.0 * robot_radius / K, -1.0, 1.0)
  return -vel, vel

def getWheelSpeeds(perception, planner):
  position = perception.localizer.predict()
  pose = list(np.argwhere(position == np.amax(position))[0, :])
  traj = getTrajectory(pose, 7, planner) # 7 step lookahead
  halt = sum([np.array_equal(np.array([0, 0]), t) for t in traj])
  if halt > 0: # there were some commands to halt and not move
    return 0.0, 0.0
  dist, angle = approximateSlope(pose, traj)
  left1, right1 = moveForward(dist, 1.0)
  left2, right2 = spinAround(angle, 1.0)
  left = 0.6 * left1 + 0.4 * left2
  right = 0.6 * right1 + 0.4 * right2
  return left, right
