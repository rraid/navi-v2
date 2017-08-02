import ctypes, os
import numpy as np

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
  image_pointer = ctypes.cast((ctypes.c_ubyte * 720 * 1280 * 4)(),ctypes.POINTER(ctypes.c_ubyte))
  image = np.ctypeslib.ar_array(image_pointer, shape=(720,1280))
  res =  libzed.grabDepthFrame(image_pointer)
  if res==False:
    return None
  else:
    return image
  
