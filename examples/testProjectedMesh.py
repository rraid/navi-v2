
import sys
sys.path.append("../device/")
sys.path.append("../perception/")
import devhub
import zed
import projectedMesh
import numpy as np
import cv2

devhub.init()

while True:
  pose = zed.getPose()[0]
  x = pose[0]
  y = pose[1]
  theta = pose[5]
  pos = np.array((x, y))

  vt = zed.getMeshData()
  if vt == None:
    continue
  v = vt[0]
  v = v.reshape((v.shape[0] / 3, 3))
  t = vt[1]
  t = t.reshape((t.shape[0] / 3, 3))

  mesh = projectedMesh.project3DMeshTo2DGrid(v, t, (x, y))
  print mesh.shape
  cv2.imshow("mesh", np.flipud(mesh) * 255)
  cv2.waitKey(10)
  """
import sys
sys.path.append("../perception/")
from projectedMesh import project3DMeshTo2DGrid
import numpy as np

triangles = np.array([
  [0, 1, 2],
  [3, 1, 2],
  [3, 0, 2],
  [1, 0, 4]])

vertices = [
    (10, 10, 10),
    (10, 20, 30),
    (20, 10, 5),
    (20, 20, 20),
    (5, 5, 0)]

grid = project3DMeshTo2DGrid(vertices, triangles, (10.0, 10.0))
import cv2
cv2.imshow("grid", np.flipud(grid) * 255)
cv2.waitKey(0)
"""
