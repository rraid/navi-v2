import sys
sys.path.append("../device/")
import devhub
import signal
import time

testMotors = False
stopTest = False

def stopsigHandler(signo, frame):
  stopTest = True
  devhub.stop()
  sys.exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print "Press Ctrl+C to stop"
  devhub.init()
  while not stopTest:
    print "Zed:", devhub.getZedReadings()
    print "GPS:", devhub.getGPSReadings()
    print "Heading", devhub.getCompassReadings()
    time.sleep(0.1) # 10 MHz refresh
    #print "Buttons:", devhub.getButtonReadings()
    if testMotors:
      devhub.setMotorVelocity(0, 0)

