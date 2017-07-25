#include <chilitags/chilitags.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "chili.h"

struct float8 {
  int id;
  float data[8];
};

static std::vector<struct float8> data;

int getNumDetect(void *img, int width, int height) {
  data.clear();
  cv::Mat I(height, width, CV_8UC1, img);
  for (const auto &tag : chilitags::Chilitags().find(I)) {
    const cv::Mat_<cv::Point2f> corners(tag.second);
    struct float8 pts;
    pts.id = tag.first;
    for (int i = 0; i < 4; i++) {
      pts.data[i * 2 + 0] = corners(i).x;
      pts.data[i * 2 + 1] = corners(i).y;
    }
    data.push_back(pts);
  }
  return data.size();
}

void copyAndClean(void *__ids__, void *__buf__) {
  int *ibuf = (int *)__ids__;
  float *fbuf = (float *)__buf__;
  for (int i = 0; i < data.size(); i++) {
    for (int j = 0; j < 8; j++) {
      ibuf[i] = data[i].id;
      fbuf[i * 8 + j] = data[i].data[j];
    }
  }
  data.clear();
}
