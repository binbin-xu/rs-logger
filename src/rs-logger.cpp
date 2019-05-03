// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "rs-logger..h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <cstdlib>  // mkdir
#include <chrono> //control time
#include <ctime>
#include <fstream>

/*
 This example introduces the concept of spatial stream alignment.
 For example usecase of alignment, please check out align-advanced and measure demos.
 The need for spatial alignment (from here "align") arises from the fact
 that not all camera streams are captured from a single viewport.
 Align process lets the user translate images from one viewport to another. 
 That said, results of align are synthetic streams, and suffer from several artifacts:
 1. Sampling - mapping stream to a different viewport will modify the resolution of the frame 
               to match the resolution of target viewport. This will either cause downsampling or
               upsampling via interpolation. The interpolation used needs to be of type
               Nearest Neighbor to avoid introducing non-existing values.
 2. Occlussion - Some pixels in the resulting image correspond to 3D coordinates that the original
               sensor did not see, because these 3D points were occluded in the original viewport.
               Such pixels may hold invalid texture values.
*/

// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction
{
  to_depth,
  to_color
};

struct ParamConfig{
  //realsense config
  rs2::config cfg;
  int width = 640;
  int height = 480;

  float       alpha = 0.5f;               // Transparancy coefficient
  direction   dir = direction::to_color;  // Alignment direction

  // create booleans to control auto-exposure and auto white-balance
  bool is_stereo_auto_exposure_on = true;
  bool is_rgb_auto_exposure_on = true;
  bool is_auto_white_balance = true;

  //parameter to control exposture time and gain
  float exposure_time = 166; //default in realsense, unit: ms
  float gain = 64;  //default in realsense

  //output format
  bool output_rosbag = false;
  bool output_sequences = true;
  bool output_raw = false;
  bool output_klg = false;

  ParamConfig();
  ParamConfig(int width, int height);
};

ParamConfig::ParamConfig() = default;

ParamConfig::ParamConfig(int width, int height): width(width), height(height){

}

//common flags for Imgui
static const int flags = ImGuiWindowFlags_NoCollapse
    | ImGuiWindowFlags_NoScrollbar
    | ImGuiWindowFlags_NoSavedSettings
    | ImGuiWindowFlags_NoResize
    | ImGuiWindowFlags_NoMove;

std::string return_current_time_and_date()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  return ss.str();
}

void make_new_folder(const std::string& folderName){
  std::string mkfolder = "mkdir -p " + folderName;
  const int dir_err = std::system(mkfolder.c_str());
  if (-1 == dir_err)
  {
    printf("Error creating directory!n");
    exit(1);
  }
}

int find_argument(int argc, char** argv, const char* argument_name)
{
  for(int i = 1; i < argc; ++i)
  {
    if(strcmp(argv[i], argument_name) == 0)
    {
      return (i);
    }
  }
  return (-1);
}

void parse_argument(int argc, char** argv, const char* str, std::string &val)
{
  int index = find_argument(argc, argv, str) + 1;

  if(index > 0 && index < argc)
  {
    val = argv[index];
  }
}

int parse_argument(int argc, char** argv, const char* str, int &val)
{
  int index = find_argument(argc, argv, str) + 1;

  if(index > 0 && index < argc)
  {
    val = atoi(argv[index]);
  }

  return (index - 1);
}

void RSLogger::init_recording_(const ParamConfig &config){
  //initialize recording folders
  std::string curr_time = return_current_time_and_date();
  char * home = getenv("HOME");
  this->logFolder_ = home;
  this->logFolder_.append("/");

  this->logFolder_.append("rs-logger/" + curr_time  + "/");
  make_new_folder(this->logFolder_);

  if (config.output_rosbag){
    this->rosbag = this->logFolder_ + curr_time + ".bag";
  }

  if (config.output_sequences){
    this->rgbFolder_ = this->logFolder_ + "rgb/";
    this->depthFolder_ = this->logFolder_ + "depth/";

    make_new_folder(this->rgbFolder_);
    make_new_folder(this->depthFolder_);
  }

  this->curr_frame_num_= 0;
}

void render_slider(rect location, std::vector<rs2::sensor>& sensors, ParamConfig& config)
{
//  ImGui::SetNextWindowPos({ location.x, location.y });
//  ImGui::SetNextWindowSize({ location.w, location.h });

  // Render transparency slider:
  ImGui::Begin("Display", nullptr);
  ImGui::PushItemWidth(-1);
  ImGui::SliderFloat("##Slider", &(config.alpha), 0.f, 1.f);
  ImGui::PopItemWidth();
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Texture Transparancy: %.3f", &(config.alpha));

  // Render direction checkboxes:
  bool to_depth = (config.dir == direction::to_depth);
  bool to_color = (config.dir == direction::to_color);

  if (ImGui::Checkbox("Align To Depth", &to_depth))
  {
    config.dir = to_depth ? direction::to_depth : direction::to_color;
  }
//    ImGui::SameLine();
//    ImGui::SetCursorPosX(location.w - 140);
  if (ImGui::Checkbox("Align To Color", &to_color))
  {
    config.dir = to_color ? direction::to_color : direction::to_depth;
  }

  // Tune parameters for sensors:
  auto& stereo_sensor = sensors[0];
  auto& rgb_sensor = sensors[1];

  //set auto exposure:
  if (ImGui::Checkbox("Stereo exposure", &(config.is_stereo_auto_exposure_on))){
    stereo_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.is_stereo_auto_exposure_on);
  }
  if (ImGui::Checkbox("RGB Auto exposure", &(config.is_rgb_auto_exposure_on))){
    rgb_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.is_rgb_auto_exposure_on);
  }

  // Tune exposure time and gain
  if (!config.is_rgb_auto_exposure_on){
    if (rgb_sensor.supports(RS2_OPTION_EXPOSURE)){
      auto exposure_range = rgb_sensor.get_option_range(RS2_OPTION_EXPOSURE);
      ImGui::SliderFloat("RGB exposure time", &(config.exposure_time), 10, 280);
      rgb_sensor.set_option(RS2_OPTION_EXPOSURE, config.exposure_time);
    }
    if (rgb_sensor.supports(RS2_OPTION_GAIN)){
      auto gain_range = rgb_sensor.get_option_range(RS2_OPTION_GAIN);
      ImGui::SliderFloat("RGB gain", &(config.gain), gain_range.min, gain_range.max);
      rgb_sensor.set_option(RS2_OPTION_GAIN, config.gain);
    }
  }

  //set white balance:
  if (ImGui::Checkbox("RGB Auto white balance", &config.is_auto_white_balance)){
    rgb_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.is_auto_white_balance);
  }

  ImGui::End();
}

void RSLogger::record_frames(const rs2::frameset &frameset) {
  // With the aligned frameset we proceed as usual
  auto depth = frameset.get_depth_frame();
  auto color = frameset.get_color_frame();

  record_rgb_(color);
  record_depth_(depth);

  //increament
  this->curr_frame_num_++;
}
void RSLogger::record_rgb_(const rs2::video_frame &color) {
  // Query frame size (width and height)
  const int w = color.as<rs2::video_frame>().get_width();
  const int h = color.as<rs2::video_frame>().get_height();

  // Create OpenCV matrix of size (w,h) from the color data
  cv::Mat colorImage(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
  cv::cvtColor(colorImage, colorImage, CV_BGR2RGB);

  std::stringstream out_sqs;
  out_sqs << this->rgbFolder_;
  out_sqs << std::setfill('0') << std::setw(5) << this->curr_frame_num_ << ".png";
  this->rgb_seq = out_sqs.str();
  cv::imwrite(this->rgb_seq, colorImage);
}

void RSLogger::record_depth_(const rs2::depth_frame& depth) {
  // Query frame size (width and height)
  const int w = depth.as<rs2::depth_frame>().get_width();
  const int h = depth.as<rs2::depth_frame>().get_height();

  // Create OpenCV matrix of size (w,h) from the depth data
  cv::Mat depthImage(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

  std::stringstream out_sqs;
  out_sqs << this->depthFolder_;
  out_sqs << std::setfill('0') << std::setw(5) << this->curr_frame_num_ << ".png";
  this->depth_seq = out_sqs.str();
  cv::imwrite(this->depth_seq, depthImage);
}

void RSLogger::start(const ParamConfig& config) {
  //if re-start, currently recording should be false
  if (!this->recording){
    this->init_recording_(config);
    std::cout<<"initialize folders"<<std::endl;
    if (config.output_sequences)
      this->record_intrinsics_();
  }
  //if resume: currently recording should be true -> no special thing needs to be done
  this->recording = true;
  this->recording_pause = false;
}

void RSLogger::pause() {
  this->recording = true;
  this->recording_pause = true;
}

void RSLogger::stop() {
  this->recording = false;
  this->recording_pause = false;
  std::cout<<"finishing recording ..."<<std::endl;

}
bool RSLogger::is_recording() {
  return this->recording;
}

bool RSLogger::is_recording_paused() {
  return this->recording && !this->recording_pause;
}

void RSLogger::show_recording_info(const ParamConfig& config, const rs2::device &curr_device) {
  if(!this->recording_pause){
    if (config.output_rosbag && curr_device.as<rs2::recorder>()) {
      std::string rosbag_info = "Recording to " + this->rosbag;
      ImGui::TextColored({255 / 255.f, 64 / 255.f, 54 / 255.f, 1}, rosbag_info.c_str());
    }

    //record raw sequences
    if (config.output_sequences) {
      std::string rgb_info = "Recording to " + this->rgb_seq;
      std::string depth_info = "Recording to " + this->depth_seq;
      ImGui::TextColored({255 / 255.f, 64 / 255.f, 54 / 255.f, 1}, rgb_info.c_str());
      ImGui::TextColored({255 / 255.f, 64 / 255.f, 54 / 255.f, 1}, depth_info.c_str());
    }
  }
  else{
    if (config.output_rosbag && curr_device.as<rs2::recorder>()) {
      std::string rosbag_info = "Pausing " + this->rosbag;
      ImGui::TextColored({255 / 255.f, 64 / 255.f, 54 / 255.f, 1}, rosbag_info.c_str());
    }

    if (config.output_sequences) {
      std::string rgb_info = "Pausing " + this->rgb_seq;
      std::string depth_info = "Pausing to " + this->depth_seq;
      ImGui::TextColored({255 / 255.f, 64 / 255.f, 54 / 255.f, 1}, rgb_info.c_str());
      ImGui::TextColored({255 / 255.f, 64 / 255.f, 54 / 255.f, 1}, depth_info.c_str());
    }
  }
}

void RSLogger::record_intrinsics_() {
  std::ofstream camera_info;
  std::string info = this->logFolder_ + "camera.info";
  camera_info.open(info.c_str());
  std::cerr << "[INFO] Logging intrinsics Information to : " << info << std::endl;

  camera_info << "#Image Res.(w,h):" <<std::endl;
  camera_info << this->resX << " " << this->resY <<std::endl;
  camera_info << "#focus lens     :" <<std::endl;
  camera_info << this->fx << " " << this->fy <<std::endl;
  camera_info << "#principal pt   :" <<std::endl;
  camera_info << this->cx << " " << this->cy <<std::endl;
  camera_info << "#Undist. coeff. :" <<std::endl;
  camera_info << this->d0 << " " << this->d1 << " " << this->d2 << " " << this->d3 << " " << this->d4 << std::endl;

  camera_info.close();
}

void RSLogger::get_intrinsics(const rs2::stream_profile &stream) {
  // A sensor's stream (rs2::stream_profile) is in general a stream of data with no specific type.
  // For video streams (streams of images), the sensor that produces the data has a lens and thus has properties such
  //  as a focal point, distortion, and principal point.
  // To get these intrinsics parameters, we need to take a stream and first check if it is a video stream
  if (auto video_stream = stream.as<rs2::video_stream_profile>())
  {
    try
    {
      //If the stream is indeed a video stream, we can now simply call get_intrinsics()
      rs2_intrinsics intrinsics = video_stream.get_intrinsics();
      auto resolution = std::make_pair(intrinsics.width, intrinsics.height);
      auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
      auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
      rs2_distortion model = intrinsics.model;

      this->resX = resolution.first;
      this->resY = resolution.second;
      this->fx = focal_length.first;
      this->fy = focal_length.second;
      this->cx = principal_point.first;
      this->cy = principal_point.second;
      this->d0 = intrinsics.coeffs[0];
      this->d1 = intrinsics.coeffs[1];
      this->d2 = intrinsics.coeffs[2];
      this->d3 = intrinsics.coeffs[3];
      this->d4 = intrinsics.coeffs[4];

      std::cout << "Camera Resolution       : " << this->resX << ", " << this->resY << std::endl;
      std::cout << "Principal Point         : " << this->cx << ", " << this->cy << std::endl;
      std::cout << "Focal Length            : " << this->fx << ", " << this->fy << std::endl;
      std::cout << "Distortion Model        : " << model << std::endl;
      std::cout << "Distortion Coefficients : [" << this->d0 << "," << this->d1 << "," <<
                this->d2 << "," << this->d3 << "," << this->d4 << "]" << std::endl;


    }
    catch (const std::exception& e)
    {
      std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
    }
  }
  else if (auto motion_stream = stream.as<rs2::motion_stream_profile>())
  {
    try
    {
      //If the stream is indeed a motion stream, we can now simply call get_motion_intrinsics()
      rs2_motion_device_intrinsic intrinsics = motion_stream.get_motion_intrinsics();

      std::cout << " Scale X      cross axis      cross axis  Bias X \n";
      std::cout << " cross axis    Scale Y        cross axis  Bias Y  \n";
      std::cout << " cross axis    cross axis     Scale Z     Bias Z  \n";
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          std::cout << intrinsics.data[i][j] << "    ";
        }
        std::cout << "\n";
      }

      std::cout << "Variance of noise for X, Y, Z axis \n";
      for (int i = 0; i < 3; i++)
        std::cout << intrinsics.noise_variances[i] << " ";
      std::cout << "\n";

      std::cout << "Variance of bias for X, Y, Z axis \n";
      for (int i = 0; i < 3; i++)
        std::cout << intrinsics.bias_variances[i] << " ";
      std::cout << "\n";
    }
    catch (const std::exception& e)
    {
      std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
    }
  }
  else
  {
    std::cerr << "Given stream profile has no intrinsics data" << std::endl;
  }
}


RSLogger::RSLogger() = default;

int main(int argc, char * argv[]) try
{
  int record_width = 640;
  int record_height = 480;
  int display_width = 1280;
  int display_height = 720;
//  std::string record_floder = "./";
  parse_argument(argc, argv, "-w", record_width);
  parse_argument(argc, argv, "-h", record_height);
//  parse_argument(argc, argv, "-f", record_floder);
  // Create and initialize GUI related objects
  window app(display_width, display_height, "RealSense Logger"); // Simple window handling
//  ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
  ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
  rs2::colorizer c;                     // Helper to colorize depth images
  texture depth_image, color_image;     // Helpers for renderig images

  //import paramter configurations
  ParamConfig config;
  std::shared_ptr<RSLogger> logger_ptr(new RSLogger());

  // Create a pipeline to easily configure and start the camera
//  rs2::pipeline pipe_ptr;
  // Create a shared pointer to a pipeline
  auto pipe_ptr = std::make_shared<rs2::pipeline>();
  config.cfg.enable_stream(RS2_STREAM_DEPTH, config.width, config.height);
  config.cfg.enable_stream(RS2_STREAM_COLOR, config.width, config.height);
  auto rs_profile = pipe_ptr->start(config.cfg);
  auto const steam = pipe_ptr->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  logger_ptr->get_intrinsics(steam);
  // Initialize a shared pointer to a device with the current device on the pipeline
  auto curr_device = rs_profile.get_device();

  // get information about the auto exposure and auto whitebalance
  auto sensors = curr_device.query_sensors();
//  float scale  = sensors.front().as<rs2::depth_sensor>().get_depth_scale();
//  std::cout<<"depth scale "<<scale<<std::endl;

  auto& stereo_sensor = sensors[0];
//    std::cout<<stereo_sensor.get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
//    std::cout<<stereo_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE)<<std::endl;
  auto& rgb_sensor = sensors[1];

  //set auto exposure:
  stereo_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.is_stereo_auto_exposure_on);
  rgb_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.is_rgb_auto_exposure_on);
  //set white balance:
  rgb_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.is_auto_white_balance);

  // Define two align objects. One will be used to align
  // to depth viewport and the other to color.
  // Creating align object is an expensive operation
  // that should not be performed in the main loop
  rs2::align align_to_depth(RS2_STREAM_DEPTH);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  while (app) // Application still alive?
  {
    // Using the align object, we block the application until a frameset is available
    rs2::frameset frameset = pipe_ptr->wait_for_frames();

    if (config.dir == direction::to_depth)
    {
      // Align all frames to depth viewport
      frameset = align_to_depth.process(frameset);
    }
    else
    {
      // Align all frames to color viewport
      frameset = align_to_color.process(frameset);
    }

    // With the aligned frameset we proceed as usual
    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();
    auto colorized_depth = c.colorize(depth);

    glEnable(GL_BLEND);
    // Use the Alpha channel for blending
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (config.dir == direction::to_depth)
    {
      // When aligning to depth, first render depth image
      // and then overlay color on top with transparancy
      depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });
      color_image.render(color, { 0, 0, app.width(), app.height() },
                         config.alpha);
    }
    else
    {
      // When aligning to color, first render color image
      // and then overlay depth image on top
      color_image.render(color, { 0, 0, app.width(), app.height() });
      depth_image.render(colorized_depth, { 0, 0, app.width(), app
          .height() }, 1 - config.alpha);
    }

    glColor4f(1.f, 1.f, 1.f, 1.f);
    glDisable(GL_BLEND);

    ImGui_ImplGlfw_NewFrame();
    // Render the UI:
    float slider_height = 200;
    render_slider({ 0.f, 0, 180, slider_height}, sensors, config);

//    //record UI
//    record({ 0.f, display_height, 180, display_height}, pipe_ptr, curr_device, config);
//    ImGui::SetNextWindowPos({0.f, slider_height} );
//    ImGui::SetNextWindowSize({180, slider_height + 50});
    ImGui::Begin("record");
    ImGui::Text("Click 'record' to start recording");
    ImGui::Text("%.1f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    //choose which format to use
    ImGui::Checkbox("Rosbag", &config.output_rosbag);
    ImGui::Checkbox("Sequences", &config.output_sequences);
    ImGui::Checkbox("RAW", &config.output_raw);
    ImGui::Checkbox("KLG", &config.output_klg);

    // if record bottom is pressed
    if (ImGui::Button("record")){
      //re-start or resume, depending on states of the recorder
        logger_ptr->start(config);

      //record ros bag
      if(config.output_rosbag){
        // If it is the start of a new recording (device is not a recorder yet)
        if (!curr_device.as<rs2::recorder>())
        {
          pipe_ptr->stop(); // Stop the pipeline with the default configuration
          pipe_ptr = std::make_shared<rs2::pipeline>();
//        rs2::config cfg; // Declare a new configuration
          config.cfg.enable_stream(RS2_STREAM_DEPTH, config.width, config.height);
          config.cfg.enable_stream(RS2_STREAM_COLOR, config.width, config.height);
          config.cfg.enable_record_to_file(logger_ptr->rosbag);
          pipe_ptr->start(config.cfg); //File will be opened at this point
          curr_device = pipe_ptr->get_active_profile().get_device();
        }
        else
        { // If the recording is resumed after a pause, there's no need to reset the shared pointer
          curr_device.as<rs2::recorder>().resume(); // rs2::recorder allows access to 'resume' function
        }
      }

    }

    /*
     When pausing, device still holds the file.
     */
    if (logger_ptr->is_recording()){
      //display status
      logger_ptr->show_recording_info(config, curr_device);

      if (logger_ptr->is_recording_paused())
        logger_ptr->record_frames(frameset);

      // Pause the playback if button is clicked
      if (ImGui::Button("pause\nrecord"))
      {
        if (config.output_rosbag){
          curr_device.as<rs2::recorder>().pause();
        }
        logger_ptr->pause();

      }

      if (ImGui::Button(" stop\nrecord"))
      {
        if (config.output_rosbag){
          pipe_ptr->stop(); // Stop the pipeline that holds the file and the recorder
          pipe_ptr = std::make_shared<rs2::pipeline>(); //Reset the shared pointer with a new pipeline
          pipe_ptr->start(); // Resume streaming with default configuration
          curr_device = pipe_ptr->get_active_profile().get_device();
        }

        logger_ptr->stop();
      }

    }

    ImGui::End();
    ImGui::Render();
  }

  return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception & e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
