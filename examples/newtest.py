import sys
sys.path.append("../perception")
sys.path.append("../device/")
sys.path.append("../control/")
import devhub
import perception
import pfilter
import numpy as np
import cv2
import time
import signal

stopSig = False
def stopsigHandler(signo, frame):
  stopSig = True
  print("Halting agent")
  devhub.setMotorVelocity(0, 0)
  devhub.stop()
  time.sleep(1)
  sys.exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print("Press Ctrl+C to stop")

  # stop the motors from moving for now
  devhub.init()
  devhub.setMotorVelocity(0, 0)
  
  world = np.flipud(cv2.imread("../perception/pathmap.png", \
      cv2.IMREAD_GRAYSCALE) > 128).astype(np.float32)
  localizer = pfilter.ParticleFilter()
  localizer.initializeUniformly(5000, (968,681, 360))

  
  #robotPos = np.array([100.0, 100.0])
  #perception.mapShape = (550, 550)
  perception.mapShape = (681,968)
  starttime = time.time()
  
  vel = [0.0, 0.0]
  
  path = []
  with open("path.txt", "r") as fp:
    path.append(eval(fp.readline().strip()))
  
  
  print path
  
  while stopSig:
    currtime = time.time()
    #robotPos += np.array([0.0, 0.0])
    starttime = currtime

    gps = perception.getGPSDistribution(devhub.getGPSReadings())
    compass = perception.getCompassDistribution(devhub.getCompassReadings())
    collisions = perception.getCollisionDistribution(
      perception.getLidarDistribution(devhub.getLidarReadings()),
      perception.getZedDistribution(devhub.getZedReadings()))

    print "HIII"
    localizer.updatePosition(0.0, 0.0)
    localizer.observePosition(gps, compass, collisions)
    
    print "HIII"
    
    positions = localizer.getDistribution()
    mean = np.mean(positions, axis=0)
    var = np.sqrt(np.var(positions, axis=0))


    print "HIII"
    # stop the robot if the robot's variance is too big
    if var[0] > 2.0:
    
    
      print "HIII, variance too big"
      devhub.setMotorVelocity(0, 0)
    else:
      
      print "HIII, controlling on path"
    
      cy = collisions.shape[0] / 2
      cx = collisions.shape[1] / 2
      if np.sum(collisions[cy:cy+50, cx-10:cx+10]) > 0.01:
        #devhub.setMotorVelocity(0, 0)
        print "CAREFUL! COLLISION DETECTED AHEAD!"
    
      # get closest position
      _path = np.array(path) - mean
      ind = np.argmin(np.sqrt(_path[:,0] ** 2.0 + _path[:,1] ** 2.0))
      k = 5
      traj = path[ind:min(ind+k, len(path))]
      dist, angle = approximateSlope(mean, traj)
      left1, right1 = moveForward(dist, 1.0)
      left2, right2 = spinAround(angle, 1.0)
      left = 0.6 * left1 + 0.4 * left2
      right = 0.6 * right1 + 0.4 * right2
      devhub.setMotorVelocity(left, right)

    gridmap = localizer.predict()
    cv2.imshow("grid", np.flipud(np.clip(pathmap + gridmap, 0.0, 1.0)))
    cv2.waitKey(10)
