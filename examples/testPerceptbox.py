import sys
sys.path.append("../perception/")
import perceptbox
import numpy as np

timestep = 0

def getGridImg():
  return np.ones((400, 400))

def getGPS():
  readings = [
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0) ]
  return readings[timestep]

def getCompass():
  readings = [
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0) ]
  return readings[timestep]

def getZed():
  readings = [
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0),
      (0, 0) ]
  return readings[timestep]

if __name__ == "__main__":
  # create a perception box to be used on its own
  box = perceptbox.PerceptBox(getGridImg(),
      gpsCallback=getGPS, compassCallback=getCompass, stereoPoseCallback=getZed)

  # now that all the callbacks are set, let's create an anchor for the robot to
  # anchor the position
  box.anchor()
  box.start()

  # now that the box has been anchored, we can attempt to test out the localizer
  while True:

    print(box.getPose())

    if timestep < 9:
      timestep += 1

