import sys
sys.path.append("../control")
sys.path.append("../device")
import devhub
import control
import time

stop = [0,0]

devhub.init()

move = control.moveForward(.005,2)
print move
start = time.time()
devhub.setMotorVelocity(30,30)
time.sleep(2.5)
devhub.setMotorVelocity(0,0)
time.sleep(1)
devhub.stop()
#print control.spinAround(90,5)
