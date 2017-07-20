import sys
sys.path.append("../device/")
import devhub
import signal
import time

testMotors = False
stopTest = False

def stopsigHandler(signo, _):
  stopTest = True

if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print "Press Ctrl+C to stop"
  devhub.init()
  while not stopTest:
    print "Sonar:", devhub.getSonarReadings()
    print "Lidar:", devhub.getLidarReadings()
    print "Sonar:", devhub.getZEDDepthColumns()
    print "GPS:", devhub.getGPSLocation()
    time.sleep(0.1) # 10 MHz refresh
    #print "Buttons:", devhub.getButtonReadings()
    if testMotors:
      devhub.setMotorVelocity(0, 0)
  devhub.stop()
