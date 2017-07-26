import math

wheel_radius = 0.14 # meters
robot_radius = 0.343 # meters
RPM = 5700

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
  vel = F * 2.0 / K
  if vel < -1.0:
    vel = -1.0
  elif vel > 1.0:
    vel = 1.0
  return vel, vel

def spinAround(dTheta, dt):
  K = 2.0 * math.pi * wheel_radius * RPM / 60.0
  T = dTheta * math.pi / (180.0 * dt)
  vel = T * 2.0 * robot_radius / K
  if vel < -1.0:
    vel = -1.0
  elif vel > 1.0:
    vel = 1.0
  return -vel, vel

def wheelSpeed(rpm):
  w = 2 * math.pi * wheel_radius * rpm / 60.0
  return w

def getWheelSpeeds(left, right):
  left = wheelSpeed(left * RPM)
  right = wheelSpeed(right * RPM)
  return left, right

def forwardSpeed(left, right):
  left, right = getWheelSpeeds(left, right)
  speed = (left + right) / 2.0
  return speed

def rotatingSpeed(left, right):
  left, right = getWheelSpeeds(left, right)
  angular_speed = (right - left) / (2.0 * robot_radius)
  return angular_speed
