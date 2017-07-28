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
  vel = np.clip(F * 2.0 / K, -1.0, 1.0)
  return vel, vel

def spinAround(dTheta, dt):
  K = 2.0 * math.pi * wheel_radius * RPM / 60.0
  T = dTheta * math.pi / (180.0 * dt)
  vel = np.clip(T * 2.0 * robot_radius / K, -1.0, 1.0)
  return -vel, vel

def getWheelSpeeds(position, planner):
  traj = getTrajectory(planner)
  return left, right
