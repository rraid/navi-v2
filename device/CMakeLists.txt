SET(execName zed)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#if(COMMAND cmake_policy)
#	cmake_policy(SET CMP0003 OLD)
#	cmake_policy(SET CMP0015 OLD)
#endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH "..")

SET(VERSION_REQ_OCV "3.1")
SET(SPECIAL_OS_LIBS "")
SET(VERSION_REQ_CUDA "8.0")



find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
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
			${OpenCV_LIBRARIES}
			 ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
                        ${SPECIAL_OS_LIBS}
 )
