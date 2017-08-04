import ctypes, os
import numpy as np
import cv2

path = "/home/nvidia/navi-v2/device/libzed.so"
libzed = ctypes.cdll.LoadLibrary(path)
libzed.zed_open.resType = ctypes.c_bool
libzed.zed_close.resType = None
libzed.grabDepthFrame.resType = ctypes.c_bool
libzed.grabDepthFrame.argTypes = [ctypes.c_void_p]

def open():
  return libzed.zed_open()

def close():
  libzed.zed_close()
  
def grabDepthFrame():
  image_pointer = ctypes.cast((ctypes.c_float * 720 * 1280)(), \
      ctypes.POINTER(ctypes.c_float))
  res = libzed.grabDepthFrame(image_pointer)
  if res == False:
    return None
  else:
    image = np.ctypeslib.as_array(image_pointer, shape=(720, 1280))
    image[np.isnan(image)] = 0.0
    image[np.isinf(image)] = 0.0
    return image
