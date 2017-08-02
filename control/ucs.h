#ifndef __UCS_H__
#define __UCS_H__

extern "C" {
  void compute(void *xPotential, void *yPotential, void *gridmap,
      int rows, int cols, int x, int y);
}

#endif
