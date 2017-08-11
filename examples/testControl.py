import sys
sys.path.append("../control")
sys.path.append("../device")
import devhub
import control
import time

stop = [0,0]

move = control.moveForward(.005,2)
print move
start = time.time()
while time.time() < start + 5:
  devhub.setMotorVelocity(30,30)

start = time.time()
while time.time() < start + 5:
  devhub.setMotorVelocity(0,0)

#print control.spinAround(90,5)
