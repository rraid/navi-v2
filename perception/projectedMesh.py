import numpy as np
from skimage.draw import line_aa
import matplotlib.pyplot as plt

def project3DMeshTo2DGrid(vertices, triangles, pose, shape=(400, 400)):
  vertices = (np.array(vertices)[:,:2] - np.array(pose)).astype(np.int)
  vertices *= 10 # scale factor from meters to decimeters
  vertices += np.array([shape[0] / 2, shape[1] / 2])
  grid = np.zeros(shape)
  # remove all the duplicates
  v = np.sort(np.array(triangles)) # sort on last axis, transform to list
  v1 = np.concatenate((v[:,0:1], v[:,2:3]), axis=1)
  edges = tuple(map(tuple, np.concatenate((v[:,0:2], v[:,1:3], v1), axis=0)))
  edges = list(set(edges))
  # draw each edge
  for e in edges:
    pt1 = vertices[e[0]]
    pt2 = vertices[e[1]]
    rr, cc, val = line_aa(pt1[1], pt1[0], pt2[1], pt2[0])
    grid[rr, cc] = val * 1.0

  return grid
