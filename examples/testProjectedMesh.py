import sys
sys.path.append("../device/")
sys.path.append("../perception/")
import zed
import projectedMesh
import numpy as np
import cv2

while True:
  pose = zed.getPose()
  x = pose[0]
  y = pose[1]
  theta = pose[6]
  pos = np.array((x, y))

  vt = zed.getMeshData()
  v = vt[:,0]
  v.reshape((v.shape[0] / 3, 3))
  t = vt[:,1]
  t.reshape((t.shape[0] / 3, 3))

  mesh = projectedMesh.project3DMeshTo2DGrid(v, t, (x, y))
  cv2.imshow("mesh", np.flipud(mesh) * 255)
  cv2.waitKey(10)
