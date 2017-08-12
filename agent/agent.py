import sys
sys.path.append("../device/")
import devhub
sys.path.append("../perception/")
import perception
sys.path.append("../control/")
import keyboardControl
import zed
import control
import planning
import signal
import projectedMesh
import time
import cv2
import numpy as np
import math

planner = None
controller = None
zedOffset = None

watchdogTimer = 0

## Get Path
path = []
with open("../examples/path.txt", "r") as fp:
  path.append(eval(fp.readline().strip()))
path = path[::-1]

stopSig = False
def stopsigHandler(signo, frame):
  stopSig = True
  print("Halting agent")
  devhub.setMotorVelocity(0, 0)
  devhub.stop()
  planner.stop()
  controller.stop()
  
  time.sleep(1)
  
  planner.join()
  controller.join()
  zed.close()
  time.sleep(1)
  sys.exit(0)
  
def setup():
  global planner
  global controller
  global zedOffset

  print("Press Ctrl+C to stop")
  
  ## Initalize Devhub
  devhub.init()
  time.sleep(2)
  # stop the motors from moving for now
  devhub.setMotorVelocity(0, 0)
  zedOffset = (math.degrees(zed.getPose()[5])-devhub.getCompassReadings() + 90) % 360
  watchdogTimer = time.time()  
  planner = planning.AStar(np.zeros([200,200]))
  planner.start()
  controller = keyboardControl.Controller()
  controller.start()

  time.sleep(3)
  ##This is to have the robot map the area around it

def loop():
  global planner
  global controller
  global zedOffset
  global watchdogTimer
  ##Get Position
  pose = zed.getPose()
  x = pose[0]
  y = pose[1]
  theta = (pose[5] * 180 / math.pi)
  pos = np.array((x, y))

  ##Get Local Map
  vt = zed.getMeshData()
  if type(vt) == type(None):
    return
  v = vt[0]
  v = v.reshape((v.shape[0] / 3, 3))
  t = vt[1]
  t = t.reshape((t.shape[0] / 3, 3))
  mesh = projectedMesh.project3DMeshTo2DGrid(v, t, (x, y)).T
  
  ##Find Local Map
  gps = devhub.getGPSReadings()
  gps = np.array(gps)
  
  ## Check is zed and gps are consistent
  if time.time() - watchdogTimer > 1:
    if np.linalg.norm(gps-pos) >10:
      devhub.setMotorVelocity(0,0)
      return
    else:
      watchdogTimer = time.time()
  
  pathGoal = None
  for node in path:
    if np.linalg.norm(node - pos) <10.0:
      pathGoal = node
      break
  if pathGoal == None:
    devhub.setMotorVelocities(0,0)
    print "No path found"
    return
    
  planner.setConstraintSpace(mesh)
  localGoal = (pathGoal - pos) * 10
  rotation = np.array([[np.cos(zedOffset), -np.sin(zedOffset)], [np.sin(zedOffset), np.cos(zedOffset)]])
  localGoal = np.dot(rotation, localGoal)
  startingPoint = (mesh.shape[0]/2,mesh.shape[1]/2)
  planner.setNextGoal(localGoal + startingPoint)
  planner.computePath(startingPoint)
  localPath = [startingPoint]
  curr = startingPoint
  while not planner.reachedGoal(curr):
    curr = planner.getNextState(curr)
    localPath.append(curr)
  left,right = control.followTraj(curr,localPath,theta,2)
  #devhub.setMotorVelocities(left,right)
  
if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  setup()
  while not stopSig:
    loop()    
  
      
  
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
    
