//
// Created by binbin on 17/04/19.
//

#ifndef LIBREALSENSE2_RS_LOGGER_H
#define LIBREALSENSE2_RS_LOGGER_H

#include "thirdparty/example.hpp"

enum class direction;
struct ParamConfig;

//helper function
std::string return_current_time_and_date();
void make_new_folder(const std::string& folderName);
void render_slider(rect location, std::vector<rs2::sensor>& sensors, ParamConfig& config);

//every time click, a new recorder will be generated
class RSLogger{
 public:
  RSLogger();
  std::string rosbag;
  std::string rgb_seq;
  std::string depth_seq;
  void record_frames(const rs2::frameset& frameset);

  //imgui caller
  void start(const ParamConfig& config);
  void pause();
  void stop();
  bool is_recording();
  bool is_recording_paused();
  void show_recording_info(const ParamConfig& config, const rs2::device &curr_device);
  void get_intrinsics(const rs2::stream_profile &stream);

 private:

  // Create booleans to control GUI (recorded - allow play button, recording - show 'recording to file' text)
  bool recording = false;
  bool recording_pause = false;

  int width_;
  int height_;
  int curr_frame_num_ = 0;

//  intrinsics
  float resX, resY, fx, fy, cx, cy, d0, d1, d2, d3, d4;

  std::string logFolder_;
  std::string rgbFolder_;
  std::string depthFolder_;

  void init_recording_(const ParamConfig &config);
  void record_depth_(const rs2::depth_frame& depth);
  void record_rgb_(const rs2::video_frame& rgb_frame);
  void record_intrinsics_();
//  void record(rect location, std::shared_ptr<rs2::pipeline> pipe_ptr, rs2::device& curr_device, ParamConfig& config);
};

#endif //LIBREALSENSE2_RS_LOGGER_H
