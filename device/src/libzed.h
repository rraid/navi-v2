#ifndef libzed_h
#define libzed_h
extern "C"{
bool zed_open();
void zed_close();
bool grabDepthFrame(void*);
}
#endif

