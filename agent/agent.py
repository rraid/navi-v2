import sys
sys.path.append("../device/")
import devhub
sys.path.append("../perception/")
import perception
sys.path.append("../control/")
import zed
import control
import planning
import signal
import projectedMesh
import time
import cv2
import numpy as np
import math

perceptor = None
localizer= None
zedOffset = None

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
  time.sleep(1)
  sys.exit(0)
  
  
if __name__ == "__main__":
  
  signal.signal(signal.SIGINT, stopsigHandler)
  print("Press Ctrl+C to stop")
  
  ## Initalize Devhub
  devhub.init()
  # stop the motors from moving for now
  devhub.setMotorVelocity(0, 0)
  time.sleep(3)
  ##This is to have the robot map the area around it
  devhub.setMotorVelocity(-1,1)
  spinTime = time.time()
  zedOffset = None
  while time.time() - spinTime < 5:
    angle = devhub.getCompassReadings()
    if angle > 355 or angle < 5:
      zedOffset = zed.getPose()[0,5] * 180 / math.pi
  print zedOffset
  devhub.setMotorVelocity(0,0)  
  while not stopSig:
    
    ##Get Position
    pose = zed.getPose()[0]
    x = pose[0]
    y = pose[1]
    theta = (pose[5] * 180 / math.pi) - zedOffset + 90
    pos = np.array((x, y))

    ##Get Local Map
    vt = zed.getMeshData()
    if vt == None:
      continue
    v = vt[0]
    v = v.reshape((v.shape[0] / 3, 3))
    t = vt[1]
    t = t.reshape((t.shape[0] / 3, 3))
    mesh = projectedMesh.project3DMeshTo2DGrid(v, t, (x, y)).T
    
    ##Find Local Map
    gps = devhub.getGPSReadings()
    print "HI IM GPS" ,gps
    pathGoal = None

    for node in path:
      if np.linalg.norm(node - gps) <10.0:
        pathGoal = node

        break
    
    planner = planning.AStar(mesh)
    localGoal = (pathGoal - np.array(gps)) * 10
    relativeRadians = -theta * math.pi / 180.0
    rotation = np.array([[np.cos(relativeRadians), -np.sin(relativeRadians)], [np.sin(relativeRadians), np.cos(relativeRadians)]])
    localGoal = np.dot(rotation, localGoal)
    startingPoint = (mesh.shape[0]/2,mesh.shape[1]/2)
    planner.setNextGoal(localGoal + startingPoint)
    planner.setConstraintSpace(np.zeros(mesh.shape))
    planner.computePath(startingPoint)
    localPath = [startingPoint]
    curr = startingPoint
    while not planner.reachedGoal(curr):
      curr = planner.getNextState(curr)
      localPath.append(curr)
    left,right = control.followTraj(curr,localPath,theta,2)
    devhub.setMotorVelocities(left,right)
  
      
  
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
    
