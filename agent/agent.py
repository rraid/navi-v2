import sys
sys.path.append("../device/")
import devhub
sys.path.append("../perception/")
import perception
sys.path.append("../control/")
import control
import planning
import signal
import serial
import pygame
import time
import cv2

# Pygame ish
bkg = (255, 255, 255)
clr = (0, 0, 0)
pygame.init()
size = (200, 200)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Robot Controller")
pygame.key.set_repeat(1, 1)
font = pygame.font.SysFont("Stencil", 20)
clock = pygame.time.Clock()


# Initial value of speed
speed = 0

def keyboardControl():
  global speed
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      pygame.quit()
      sys.exit()
    elif event.type == pygame.KEYUP:
      if event.key == pygame.K_a:
        speed += .05
      elif event.key == pygame.K_s:
        speed -= .05
  if speed < -1:
    speed = -1
  elif speed > 1:
     speed = 1
    
  # Reset info text & motor values
  txt = ""
  motor = [0, 0]

  # Grab keystrokes
  keys_pressed = pygame.key.get_pressed()
  
  if keys_pressed[pygame.K_q]:
    stopSig = True
  elif keys_pressed[pygame.K_UP] and keys_pressed[pygame.K_LEFT]:
    txt = "Left arc turn"
    motor = [0, 1]
  elif keys_pressed[pygame.K_UP] and keys_pressed[pygame.K_RIGHT]:
    txt = "Right arc turn"
    motor = [1, 0]
  elif keys_pressed[pygame.K_DOWN] and keys_pressed[pygame.K_LEFT]:
    txt = "Left arc turn backwards"
    motor = [0, -1]
  elif keys_pressed[pygame.K_DOWN] and keys_pressed[pygame.K_RIGHT]:
    txt = "Right arc turn backwards"
    motor = [-1, 0]
  elif keys_pressed[pygame.K_UP]:
    txt = "Forward"
    motor = [1, 1]
  elif keys_pressed[pygame.K_DOWN]:
    txt = "Backward"
    motor = [-1, -1]
  elif keys_pressed[pygame.K_LEFT]:
    txt = "Left point turn"
    motor = [-1, 1]
  elif keys_pressed[pygame.K_RIGHT]:
    txt = "Right point turn"
    motor = [1, -1]
  screen.fill(bkg)
  speed_text = font.render('Speed: ' + str(speed), True, clr)
  screen.blit(speed_text, [10, 10])
  info_text = font.render(txt, True, clr)
  screen.blit(info_text, [10, 50])
  pygame.display.flip()
  clock.tick(60)
  return [motor[0] * speed, motor[1]*speed]

stopSig = False
def stopsigHandler(signo, frame):
  stopSig = True
  print("Halting agent")
  devhub.setMotorVelocity(0, 0)
  devhub.stop()
  #planner.stop()
  #perceptor.stop()
  time.sleep(1)
  sys.exit(0)
  
  
if __name__ == "__main__":
  signal.signal(signal.SIGINT, stopsigHandler)
  print("Press Ctrl+C to stop")

  # stop the motors from moving for now
  devhub.init()
  devhub.setMotorVelocity(0, 0)

  # grab the pathmap from a file
  pathmap = cv2.imread("pathmap.png")

  # initialize the perception module
  #perceptor = perception(pathmap)

  # initialize the planner module
  #planner = AStar(pathmap)
  BCC = (100, 100)
  #planner.setNextGoal(BCC)

  # use the planner to do some control
  while not stopSig:
    kbdcontrol = keyboardControl()
    devhub.setMotorVelocity(kbdcontrol[0], kbdcontrol[1])
  
  

