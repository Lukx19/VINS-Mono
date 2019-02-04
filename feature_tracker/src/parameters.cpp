#include "parameters.h"

std::string IMAGE_TOPIC;
std::string DEPTH_IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
int MAX_CNT_PER_BLOCK;
int BLOCK_PER_ROW;
int BLOCK_PER_COL;
int FAST_THRESHOLD;
bool ADAPTIVE_THRESHOLD;
bool EDGE_PREFERENCE;
bool ENABLE_F_REJECTION;
bool RGBD_CAM;
std::string ALG;

void readParameters(ros::NodeHandle& n) {
  std::string config_file;
  config_file = readROSParam<std::string>(n, "config_file");
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  std::string VINS_FOLDER_PATH = readROSParam<std::string>(n, "vins_folder");

  fsSettings["image_topic"] >> IMAGE_TOPIC;
  fsSettings["depth_topic"] >> DEPTH_IMAGE_TOPIC;
  fsSettings["imu_topic"] >> IMU_TOPIC;
  MAX_CNT = fsSettings["max_cnt"];
  std::cout << MAX_CNT << std::endl;
  MIN_DIST = fsSettings["min_dist"];
  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  FREQ = fsSettings["freq"];
  F_THRESHOLD = fsSettings["F_threshold"];
  SHOW_TRACK = fsSettings["show_track"];
  EQUALIZE = fsSettings["equalize"];
  FISHEYE = fsSettings["fisheye"];
  if (FISHEYE == 1)
    FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
  CAM_NAMES.push_back(config_file);

  MAX_CNT_PER_BLOCK = readParam<int>(fsSettings["max_features_per_block"], 5);
  BLOCK_PER_ROW = readParam<int>(fsSettings["blocks_per_row"], 5);
  BLOCK_PER_COL = readParam<int>(fsSettings["blocks_per_col"], 5);
  ALG = readParam<std::string>(fsSettings["algorithm"], "GoodFeatures");
  std::cout << "0000000000000000000000" << BLOCK_PER_ROW << "   " << BLOCK_PER_COL << std::endl;
  WINDOW_SIZE = 20;
  STEREO_TRACK = false;
  FOCAL_LENGTH = 460;
  FAST_THRESHOLD = readParam<int>(fsSettings["fast_threshold"], 20);
  ADAPTIVE_THRESHOLD = readParam<int>(fsSettings["adaptive_threshold"], 0);
  EDGE_PREFERENCE = readParam<int>(fsSettings["strong_edge_keypoint_preference"], 0);
  ENABLE_F_REJECTION = readParam<int>(fsSettings["fundamental_matrix_rejection"], 1);
  RGBD_CAM = readParam<int>(fsSettings["rgbd_camera"], 0);
  if (FREQ == 0)
    FREQ = 100;

  fsSettings.release();
}
