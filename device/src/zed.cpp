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

sl::Mesh mesh;      // sl::Mesh to hold the mesh generated during spatial mapping
sl::SpatialMappingParameters spatial_mapping_params;
sl::MeshFilterParameters filter_params;
sl::TRACKING_STATE tracking_state;

sl::Transform camera_projection;

std::thread zed_callback;

// Spatial Mapping status
bool mapping_is_started = false;
std::chrono::high_resolution_clock::time_point t_last;

bool saveMesh = false;
bool quit = false;
bool zedGrab = false;

bool zed_open() {

  // Setup configuration parameters for the ZED
  InitParameters initParameters;
  initParameters.camera_resolution = RESOLUTION_HD720;
  initParameters.depth_mode = DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
  initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
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
  
  // Configure Spatial Mapping and filtering parameters
  spatial_mapping_params.range_meter.second = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::RANGE_FAR);
  spatial_mapping_params.resolution_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::RESOLUTION_LOW);
  //printf("Initalized mapping");
  
  filter_params.set(sl::MeshFilterParameters::FILTER_LOW);
  //printf("Initalized filter");
  
  // Start motion tracking
  
  zed.enableTracking(trackingParameters);
  //printf("Enabled Tracking");
  
  //mesh.clear();
  zed.enableSpatialMapping(spatial_mapping_params);
  t_last = std::chrono::high_resolution_clock::now();
  mapping_is_started = true;
  
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
     
      if (mapping_is_started) {
        // Compute elapse time since the last call of sl::Camera::requestMeshAsync()
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_last).count();
        // Ask for a mesh update if 500ms have spend since last request
        if (duration > 500) {
          zed.requestMeshAsync();
          t_last = std::chrono::high_resolution_clock::now();
        }
        if (zed.getMeshRequestStatusAsync() == sl::SUCCESS) {
          // Get the current mesh generated
          zed.retrieveMeshAsync(mesh);
        }
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
  
  mapping_is_started = false;
  
  if(saveMesh){
    sl::Mesh wholeMesh;
    zed.extractWholeMesh(wholeMesh);
    wholeMesh.filter(filter_params, 0);
    //std::string saveName = getDir() + "mesh_gen.obj";
    //wholeMesh.save(saveName.c_str());
  }
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

bool getMeshSizes(int* vertices, int* triangles){
  if (zedGrab != SUCCESS){
    sl::sleep_ms(1);
    return false;
  }
  sl::CameraParameters camLeft = zed.getCameraInformation().calibration_parameters.left_cam;
  camera_projection(0, 0) = 1.0f / tanf(camLeft.h_fov * M_PI / 180.f * 0.5f);
  camera_projection(1, 1) = 1.0f / tanf(camLeft.v_fov * M_PI / 180.f * 0.5f);
  float znear = 0.001f;
  float zfar = 100.f;
  camera_projection(2, 2) = -(zfar + znear) / (zfar - znear);
  camera_projection(2, 3) = -(2.f * zfar * znear) / (zfar - znear);
  camera_projection(3, 2) = -1.f;
  camera_projection(0, 2) = (camLeft.image_size.width - 2.f * camLeft.cx) / camLeft.image_size.width;
  camera_projection(1, 2) = (-1.f * camLeft.image_size.height + 2.f * camLeft.cy) / camLeft.image_size.height;
  camera_projection(3, 3) = 0.f;
  cl = mesh.getSurroundingList(camera_projection,10.0f);
  int vertSize = 0;
  int triSize = 0;
  for(auto &i: cl){
    vertSize += mesh[i].vertices.size() * 3;
    triSize += mesh[i].triangles.size() * 3;
    //Multiply by 3 to account for x, y, z
  }
  *vertices = vertSize;
  *triangles = triSize;
  return true;
}

bool getMeshData(float* vertices, int* triangles){
  if (zedGrab != SUCCESS){
    sl::sleep_ms(1);
    return false;
  }
  int vCount = 0;
  int tCount = 0;
  for(size_t &i: cl ){
      for(size_t j = 0; j < mesh[i].vertices.size(); j++){

        vertices[vCount]   = mesh[i].vertices[j][0];
        vertices[vCount+1] = mesh[i].vertices[j][1];
        vertices[vCount+2] = mesh[i].vertices[j][2];
        vCount +=3;
      }
      for(size_t j = 0; j < mesh[i].triangles.size(); j++){
        triangles[tCount]   = mesh[i].triangles[j][0];
        triangles[tCount+1] = mesh[i].triangles[j][1];
        triangles[tCount+2] = mesh[i].triangles[j][2];
        tCount +=3;
      }
  }
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
