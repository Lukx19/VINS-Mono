#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;

extern std::string IMAGE_TOPIC;
extern std::string DEPTH_IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern int MAX_CNT_PER_BLOCK;
extern int BLOCK_PER_ROW;
extern int BLOCK_PER_COL;
extern int FAST_THRESHOLD;
extern std::string ALG;
extern bool ADAPTIVE_THRESHOLD;
extern bool EDGE_PREFERENCE;
extern bool ENABLE_F_REJECTION;
extern bool RGBD_CAM;

template <typename T>
T readROSParam(ros::NodeHandle& n, const std::string& name) {
  T ans;
  if (n.getParam(name, ans)) {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  } else {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

template <typename T>
T readParam(const cv::FileNode& fn, const T& default_value) {
  if (fn.empty()) {
    return default_value;
  } else {
    return fn;
  }
}

void readParameters(ros::NodeHandle& n);
