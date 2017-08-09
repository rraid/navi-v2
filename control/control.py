import math
import time

wheel_radius = 0.14 # meters
robot_radius = 0.343 # meters
RPM = 1.3
lastwritetime = [time.time()]
target_vel = np.zeros((2, ))
source_vel = np.zeros((2, 3))
pid_coeff = np.zeros((2, 3))
pid_coeff[:,0] = 0.5 # half speed up per second

def PID(target, source, timestamp, pid_coeff):
  dt = min(time.time() - timestamp[0], 1.0) # do not go higher than 1.0
  perr = target - source[:,0]
  ierr = source[:,1] + perr
  derr = perr - source[:,2]
  out = np.sum(np.multiply(\
      np.concatenate((perr, ierr, derr), axis=1),
      pid_coeff), axis=1) * dt
  source[:,0] = out
  source[:,1] = ierr
  source[:,2] = perr
  return out

def getTrajectory(pose, k, path): # no more potential field
  # find sse
  dist = np.linalg.norm(pose[:2] - path, axis=1)
  ind = np.argmin(dist)
  return np.array(path[ind:min(len(path), k)])

def approximateSlope(pose, traj):
  if traj.shape[0] == 0:
    return 0, 0
  relative = traj - pose[:2]
  norm = np.norm(relative, axis=1)
  relative = np.divide(relative, norm)
  angles = np.arctan2(relative[:,0], relative[:,1])
  meanDist = np.mean(norm)
  meanAngle = np.mean(angles) * 180.0 / math.pi
  return meanDist, meanAngle

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

def getWheelSpeeds(pose, path, targetAngle, k=7):
  traj = getTrajectory(pose, k, path)
  halt = sum([np.array_equal(np.array([0, 0]), t) for t in traj])
  if halt > 0: # there were some commands to halt and not move
    return 0.0, 0.0
  dist, angle = approximateSlope(pose, traj)
  if len(path) <= 1: # path should include goal
    angle = targetAngle - pose[2]
  left1, right1 = moveForward(dist, 1.0)
  left2, right2 = spinAround(angle, 1.0)
  left = 0.6 * left1 + 0.4 * left2
  right = 0.6 * right1 + 0.4 * right2
  return np.clip(left, -1.0, 1.0), np.clip(right, -1.0, 1.0)
