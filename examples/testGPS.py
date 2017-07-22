import sys
import cv2
sys.path.append("../perception/")
import perception
import signal

stopTest = False

def stopsigHandler(signo, frame):
  stopTest = True
  sys.exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print "Press Ctrl+C to stop"

  while not stopTest:
    cv2.imshow("GPS", perception.getGPSDistribution(124, 150, (500, 500)) * 255.0)

    cv2.waitKey(30)
