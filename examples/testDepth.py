import sys
sys.path.append("../perception/")
import perception
import time
import cv2
import math
import numpy as np

sonarReadings = [10, 10, 10, 10, 10, 10, 10, 15, 15]
lidarReadings = np.linspace(-135 * math.pi / 180.0, 135 * math.pi / 180.0, 1000)
lidarReadings = np.sin(lidarReadings * 16.0) * 4.0 + 50.0
zedReadings = np.linspace(-55 * math.pi / 180.0, 55 * math.pi / 180.0, 1000)
zedReadings = np.cos(zedReadings * 8.0) * 2.0 + 20.0

starttime = time.time()

sonar = perception.getSonarDistribution(sonarReadings)
lidar = perception.getLidarDistribution(lidarReadings)
zed = perception.getZEDDistribution(zedReadings)
collisions = perception.getCollisionDistribution(sonar*1.0, lidar*1.0, zed*1.0)

print("Time taken:", time.time() - starttime)

cv2.imshow("Collisions", np.flipud(collisions * 255.0))
cv2.waitKey(0)
