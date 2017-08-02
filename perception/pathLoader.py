import cv2

def load(fname):
  world = np.flipud(cv2.imread(fname, cv2.IMREAD_GRAYSCALE) > 128)\
      .astype(np.float32)
  return world
