///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/***************************************************************************************************
 ** This sample demonstrates how to grab images and depth map with the ZED SDK                    **
 ** and apply the result in a 3D view "point cloud style" with OpenGL /freeGLUT                   **
 ** Some of the functions of the ZED SDK are linked with a key press event		                  **
 ***************************************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "libzed.h"

// ZED includes
#include <sl/Camera.hpp>

//// Using std and sl namespaces
using namespace std;
using namespace sl;

//// Create ZED object (camera, callback, images)
sl::Camera zed;
sl::Mat depth_image;
sl::Pose pose;
std::vector<size_t> cl;

sl::TRACKING_STATE tracking_state;

sl::Transform camera_projection;

std::thread zed_callback;

// Spatial Mapping status
bool mapping_is_started = false;
std::chrono::high_resolution_clock::time_point t_last;

bool quit = false;
bool zedGrab = false;

bool zed_open() {

  // Setup configuration parameters for the ZED
  InitParameters initParameters;
  initParameters.camera_resolution = RESOLUTION_HD720;
  initParameters.depth_mode = DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
  initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP; // OpenGL's coordinate system is right_handed
  initParameters.coordinate_units = UNIT_METER; // set meter as the OpenGL world will be in meters
  //printf("Initalized Basic");
  
  // Open the ZED
  ERROR_CODE err = zed.open(initParameters);
  if (err != SUCCESS) {
    printf("[ERROR] Cannot open zed!\n");
    zed.close();
    return false; // Quit if an error occurred
  }
  //printf("Zed opened");
  
  // Set positional tracking parameters
  TrackingParameters trackingParameters;
  trackingParameters.initial_world_transform = sl::Transform::identity();
  trackingParameters.enable_spatial_memory = true;     // Enable Spatial memory
  //printf("Initalized tracking");
  
  zed.enableTracking(trackingParameters);
  //printf("Enabled Tracking");
  
  zed_callback = std::thread(zed_run);
  //printf("Started Thread");
      
  return true;
}

void zed_run(){

  // Get the distance between the center of the camera and the left eye
  float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;
  while (!quit){
    zedGrab = zed.grab();
    if (zedGrab == SUCCESS) {
      // Get the position of the camera in a fixed reference frame (the World Frame)
      TRACKING_STATE tracking_state = zed.getPosition(pose, sl::REFERENCE_FRAME_WORLD);
      if (tracking_state == TRACKING_STATE_OK) {
        // Get the pose at the center of the camera (baseline/2 on X axis)
        //transformPose(pose.pose_data, translation_left_to_center); 
        sl::Transform transform_;
        transform_.setIdentity();
        // Move the tracking frame by tx along the X axis
        transform_.tx = translation_left_to_center;
        // Apply the transformation
        pose.pose_data = Transform::inverse(transform_) * pose.pose_data * transform_;
      }
    }
    else sl::sleep_ms(1);
  }
}




/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void zed_close() {
  printf("Quitting C++\n");
  quit = true;
  zed_callback.join();

  zed.close();
}

bool getPose(void* poseGet){
  if (zedGrab != SUCCESS){
    sl::sleep_ms(1);
    return false;
  }
  // Get quaternion, rotation and translation
  // Only use Euler angles to display absolute angle values. Use quaternions for transforms.
  sl::float3 rotation = pose.getEulerAngles();
  sl::float3 translation = pose.getTranslation();
  ((float*)poseGet)[0] = translation.x;
  ((float*)poseGet)[1] = translation.y;
  ((float*)poseGet)[2] = translation.z;
  ((float*)poseGet)[3] = rotation.x;
  ((float*)poseGet)[4] = rotation.y;
  ((float*)poseGet)[5] = rotation.z;
  return true;
}

bool grabDepthFrame(void* dst) {
  if (zedGrab != SUCCESS){
    sl::sleep_ms(1);
    return false;
  }
  zed.retrieveMeasure(depth_image, sl::MEASURE_DEPTH);
  memcpy(dst, (void *)depth_image.getPtr<sl::float1>(sl::MEM_CPU),
      4 * depth_image.getHeight() * depth_image.getWidth());
  return true;
}
