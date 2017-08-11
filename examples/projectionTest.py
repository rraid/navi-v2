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
    (100, 100, 100),
    (100, 200, 300),
    (200, 100, 50),
    (200, 200, 200),
    (50, 50, 0)]

project3DMeshTo2DGrid(vertices, triangles, (100.0, 100.0))
