SET(execName zed)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 OLD)
  cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

SET(SPECIAL_OS_LIBS "")
SET(VERSION_REQ_CUDA "8.0")

find_package(ZED 2 REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" )
  SET(VERSION_REQ_CUDA "")
  link_directories("/usr/lib/aarch64-linux-gnu/tegra") ## on Jetson TX1 64bits, the correct libGL.so is located here (the default one will lead to linking error (undef ref))

  unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
  option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
  ##to prevent from opencv_dep_cudart dependencies error...
  ## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.
endif()

SET(SPECIAL_OS_LIBS "pthread" "X11")
add_definitions(-Wno-write-strings)


find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

SET(HEADER_FOLDER include)
FILE(GLOB_RECURSE HEADER_FILES "${HEADER_FOLDER}/*.hpp" "${HEADER_FOLDER}/*.h")

ADD_LIBRARY(${execName} SHARED ${SRC_FILES} ${HEADER_FILES})
add_definitions(-std=c++0x -g -O3)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${execName}
  ${SPECIAL_OS_LIBS}
  ${ZED_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  ${GLEW_LIBRARY}
  ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
  ${SPECIAL_OS_LIBS}
  )
