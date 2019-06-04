//
// Created by binbin on 03/05/19.
//

#include "raw.h"

bool Raw::imageToUchar3(const cv::Mat& mat, Raw::uchar3* image){

  unsigned char* pixelPtr = (unsigned char*)mat.data;
  int cn = mat.channels();
  int nRows = mat.rows;
  int nCols = mat.cols;

  for(int i = 0; i < nRows; ++i)
    for(int j = 0; j < nCols; ++j) {
      unsigned char r = pixelPtr[i*mat.cols*cn + j*cn + 0]; // B
      unsigned char g = pixelPtr[i*mat.cols*cn + j*cn + 1]; // G
      unsigned char b = pixelPtr[i*mat.cols*cn + j*cn + 2]; // R
      image[nCols*i+j] = Raw::make_uchar3(r,g,b);
    }
  return true;
}

bool Raw::depthToUshort(const cv::Mat mat, ushort * depthMap,
                        const float cx, const float cy, const float fx, const float fy,
                        const std::string input_depth_type){

  int cn = mat.channels();
  if(cn!=1){
    std::cerr << "Input Error: Not a supported depth image." << std::endl;
    return false;
  }
  int nRows = mat.rows;
  int nCols = mat.cols;

  for(int i = 0; i < nRows; ++i)
    for(int j = 0; j < nCols; ++j) {
      double d = static_cast<double>(mat.at<uint16_t>(i,j));

      if (input_depth_type == "z")
        depthMap[j+nCols*i] = d;
      else if (input_depth_type == "euclidean"){
        double u_u0_by_fx = (j - cx) / fx;
        double v_v0_by_fy = (i - cy) / fy;

        depthMap[j+nCols*i] = d / std::sqrt( u_u0_by_fx * u_u0_by_fx + v_v0_by_fy * v_v0_by_fy + 1);
      }
    }

  return true;
}