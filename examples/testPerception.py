import sys
import cv2
sys.path.append("../perception/")
sys.path.append("../device")
import perception
import devhub

stopTest = False

def stopsigHandler(signo, frame):
  stopTest = True
  devhub.stop()
  sys.exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print "Press Ctrl+C to stop"

  localizer = perception.Localizer()
  grid = perception.Map()
  objDetector = perception.ObjectDetector()

  devhub.init()
  while not stopTest:
    cv2.imshow("Sonar", perception.getSonarDistribution())
    cv2.imshow("Lidar", perception.getLidarDistribution())
    cv2.imshow("ZED", perception.getZEDDistribution())
    cv2.imshow("GPS", perception.getGPSDistribution())
    cv2.imshow("Compass", perception.getCompassDistribution())

    cv2.imshow("Localizer", localizer.getDistribution())
    cv2.imshow("Map", grid.getDistribution())

    print objDetector.detect()

    cv2.waitKey(30)
