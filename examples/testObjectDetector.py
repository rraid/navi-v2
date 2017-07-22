import sys
sys.path.append("../perception/")
import perception
import cv2
import signal

stopTest = False
def stopsigHandler(signo, frame):
  stopTest = True
  sys.exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print "Press Ctrl+C to stop"

  cam = cv2.VideoCapture(0)
  objdetect = ObjectDetector()
  objdetect.setObjects([0, 1, 2]) # id's of the chilitags

  while not stopTest:
    img = cam.read()
    objdetect.observe(img)
    print objdetect.detect()
