//
// Created by binbin on 03/05/19.
//

#ifndef RSLOGGER_RAW_H
#define RSLOGGER_RAW_H

#include <opencv2/opencv.hpp>

namespace Raw {

struct uint2 {
  unsigned int x, y;
};

inline uint2 make_uint2(unsigned int x, unsigned int y) {
  uint2 val;
  val.x = x;
  val.y = y;
  return val;
}

struct uchar3 {
  unsigned char x, y, z;
};

inline uchar3 make_uchar3(unsigned char x, unsigned char y, unsigned char z) {
  uchar3 val;
  val.x = x;
  val.y = y;
  val.z = z;
  return val;
}

bool imageToUchar3(const cv::Mat &mat, uchar3 *image);

// unit mm
bool depthToUshort(const cv::Mat mat, ushort *depthMap,
                   const float cx = 320.0f, const float cy = 240.0f,
                   const float fx = 600.0f, const float fy = 600.0f,
                   const std::string input_depth_type="z");

}
#endif //RSLOGGER_RAW_H
