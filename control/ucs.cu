#ifdef __NVCC__

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#define IJ2C(i,j,ld)(((j)*(ld))+(i))
#define checkCudaErrors(val) __check( (val), #val, __FILE__, __LINE__)
template<typename T>
void __check(T err, const char* const func, const char* const file, const int line) {
  if (err != cudaSuccess) {
    std::cerr << "CUDA error at: " << file << ":" << line << std::endl;
    std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
    exit(1);
  }
}

template<typename T>
T *zeros(int rows, int cols) {
  uint8_t *dbuf;
  int n_elem = rows * cols;
  checkCudaErrors(cudaMalloc(&dbuf, sizeof(T) * n_elem));
  checkCudaErrors(cudaMemset(dbuf, 0, sizeof(T) * n_elem));
  return dbuf;
}

void compute(void *xPotential, void *yPotential, void *gridmap,
    int rows, int cols, int x, int y) {
  uint8_t *closed = zeros<uint8_t>(rows, cols);
  uint32_t *opened = gheap();
  uint32_t *openedLength = zeros<uint32_t>(1, 1);
  float *gScore = zeros<float>(rows, cols);
  float *dxPotential = zeros<float>(rows, cols);
  float *dyPotential = zeros<float>(rows, cols);
  float *c_space = zeros<float>(rows, cols);

  uint8_t *action_map = zeros<float>(256, 4);

  uint32_t openlen = 1;
  while (openlen > 0) {
    dim3 blocksize(256, 1, 1);
    dim3 gridSize((openlen-1)/256+1, 1, 1);
    gpu_inplace_sort<<<gridSize, blockSize>>>(opened, openedLength);
    checkCudaErrors(cudaGetLastError());
    
    gpu_close_min<<<gridSize, blockSize>>>(opened, openedLength, closed);
    checkCudaErrors(cudaGetLastError());

    gpu_map_actions<<<gridSize, blockSize>>>(opened, openedLength, action_map);
    checkCudaErrors(cudaGetLastError());

    // TODO: finish when not so tired
  }
}

#else

void compute(void *xPotential, void *yPotential, void *gridmap,
    int rows, int cols, int x, int y) {
  printf("Cannot compute path - no CUDA library found\n");
}

#endif
