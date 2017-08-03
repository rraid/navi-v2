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
#include "libzed.h"

// ZED includes
#include <sl/Camera.hpp>

//// Using std and sl namespaces
using namespace std;
using namespace sl;

//// Create ZED object (camera, callback, images)
sl::Camera zed;
sl::Mat depth_image;

bool zed_open() {
  // Setup configuration parameters for the ZED
  InitParameters initParameters;
  initParameters.camera_resolution = RESOLUTION_HD720;
  initParameters.depth_mode = DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
  initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
  initParameters.coordinate_units = UNIT_METER; // set meter as the OpenGL world will be in meters

  // Open the ZED
  ERROR_CODE err = zed.open(initParameters);
  if (err != SUCCESS) {
    printf("[ERROR] Cannot open zed!\n");
    zed.close();
    return false; // Quit if an error occurred
  }
  return true;
}

/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void zed_close() {
  zed.close();
}

bool grabDepthFrame(void *dst) {
  if (zed.grab() != SUCCESS){
    sl::sleep_ms(1);
    return false;
  }
  zed.retrieveMeasure(depth_image, sl::MEASURE_DEPTH);
  memcpy(dst, (void *)depth_image.getPtr<sl::float1>(sl::MEM_CPU),
      4 * depth_image.getHeight() * depth_image.getWidth());
  return true;
}
