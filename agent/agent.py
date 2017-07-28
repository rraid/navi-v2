import sys
sys.path.append("../device/")
import devhub
sys.path.append("../perception/")
import perception
sys.path.append("../control/")
import control
import planning
import signal
import time
import cv2

stopSig = False
def stopsigHandler(signo, frame):
  stopSig = True

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print("Press Ctrl+C to stop")

  # stop the motors from moving for now
  devhub.init()
  devhub.setMotorVelocity(0, 0)

  # grab the pathmap from a file
  pathmap = cv2.imread("pathmap.png")

  # initialize the perception module
  perceptor = Perception(pathmap)

  # initialize the planner module
  planner = AStar(pathmap)
  BCC = (100, 100)
  planner.setNextGoal(BCC)

  # use the planner to do some control
  while not stopSig:
    planner.setConstraintSpace(perceptor.getCollisions())
    leftSpeed, rightSpeed = control.getWheelSpeeds(perceptor, planner)
    devhub.setMotorVelocity(leftSpeed, rightSpeed)

  print("Halting agent")
  devhub.setMotorVelocity(0, 0)
  planner.stop()
  perceptor.stop()
  time.sleep(1)
  sys.exit(0)
