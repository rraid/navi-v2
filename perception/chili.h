#ifndef __CHILI_H__
#define __CHILI_H__

extern "C" {
  int getNumDetect(void *img, int width, int height);
  void copyAndClean(void *__ids__, void *__buf__);
}

#endif
