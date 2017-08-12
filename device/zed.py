import ctypes, os
import numpy as np
import cv2

path = "/home/nvidia/navi-v2/device/build/libzed.so"
libzed = ctypes.cdll.LoadLibrary(path)
libzed.zed_open.resType = ctypes.c_bool
libzed.zed_close.resType = None
libzed.zed_run.resType = None
libzed.grabDepthFrame.resType = ctypes.c_bool
libzed.grabDepthFrame.argTypes = [ctypes.c_void_p]
libzed.getPose.resType = ctypes.c_bool
libzed.getPose.argTypes = [ctypes.c_void_p]
libzed.getMeshSizes.resType = ctypes.c_bool
libzed.getMeshSizes.argTypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
libzed.getMeshData.resType = ctypes.c_bool
libzed.getMeshData.argTypes = [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int)]

def open():
  return libzed.zed_open()

def close():
  libzed.zed_close()

def run():
  libzed.zed_run()
  
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
    
def getPose():
  pose_pointer = ctypes.cast((ctypes.c_float * 6)(), ctypes.POINTER(ctypes.c_float))
  res = libzed.getPose(pose_pointer)
  if res == False:
    return None
  else:
    pose = np.ctypeslib.as_array(pose_pointer, shape = (1,6))
    return pose[0,:]
 
def getMeshSizes():
  vSize_pointer = ctypes.cast((ctypes.c_int*1)(), ctypes.POINTER(ctypes.c_int))
  tSize_pointer = ctypes.cast((ctypes.c_int*1)(), ctypes.POINTER(ctypes.c_int))
  res = libzed.getMeshSizes(vSize_pointer,tSize_pointer)
  if res == False:
    return None
  else:
    return [vSize_pointer.contents.value,tSize_pointer.contents.value]
    
def getMeshData():
  vSize, tSize = getMeshSizes()
  if vSize == 0 or tSize == 0:
    return None
  #print "Sizes python: ", ctypes.sizeof(ctypes.c_float) * vSize,    ctypes.sizeof(ctypes.c_int) * tSize
  vertices_pointer = ctypes.cast((ctypes.c_float * vSize)(), ctypes.POINTER(ctypes.c_float))
  triangles_pointer = ctypes.cast((ctypes.c_int * tSize)(), ctypes.POINTER(ctypes.c_int))
  res = libzed.getMeshData(vertices_pointer,triangles_pointer)
  if res == False:
    return None
  else:
    vertices = np.ctypeslib.as_array(vertices_pointer, shape = (1,vSize))
    triangles = np.ctypeslib.as_array(triangles_pointer, shape = (1,tSize))
    return np.array([vertices[0,:],triangles[0,:]])
  
