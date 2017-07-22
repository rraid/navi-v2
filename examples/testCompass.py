import sys
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
    print perception.getCompassDistribution(360)
