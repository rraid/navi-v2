#ifndef libzed_h
#define libzed_h

#include <sl/Camera.hpp>
using namespace sl;

extern "C"{
bool zed_open();
void zed_close();
void zed_run();
bool grabDepthFrame(void*);
bool getPose(void*);

}
#endif

