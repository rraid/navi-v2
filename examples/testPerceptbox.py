import sys
sys.path.append("../perception/")
import perceptbox
import numpy as np
sys.path.append("../device/")
import devhub
import cv2

if __name__ == "__main__":
  devhub.init()
  grid = np.flipud(cv2.imread("../perception/pathmap_scaled.png", cv2.IMREAD_GRAYSCALE) / 2)
  # create a perception box to be used on its own
  box = perceptbox.PerceptBox(
      gpsCallback=getGPSReadings,
      compassCallback=getCompassReadings,
      stereoPoseCallback=getZedReadings)

  # now that all the callbacks are set, let's create an anchor for the robot to
  # anchor the position
  box.start()

  # now that the box has been anchored, we can attempt to test out the localizer
  while True:
    pose = box.getPose()
    print(pose)

    position = pose[:2]
    angle = pose[2]
    
    img = perception.getGPSDistribution(position)
    cv2.imshow("Position", np.flipud(img + grid) * 255)
    cv2.waitKey(10)
