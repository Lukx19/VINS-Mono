#pragma once

#include <execinfo.h>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <queue>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"
#include "sequence_analysis.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f& pt);

template<typename T>
void reduceVector(std::vector<T>& v, std::vector<uchar> status) {
  size_t j = 0;
  for (size_t i = 0; i < v.size(); ++i) {
    if (status[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

template<typename T>
std::string printVector(const std::vector<T> vec){
  std::stringstream ss;
  ss << "[";
  for (auto&& el : vec) {
    ss<<el<<",";
  }
  ss<<"]\n";
  return ss.str();
}

void reduceVector(vector<cv::Point2f>& v, vector<uchar> status);
void reduceVector(vector<int>& v, vector<uchar> status);

class FeatureTracker {
public:
  FeatureTracker();

  void useInitTracking();
  void useStandardTracking();

  void printStatistics() const;

  void prepareForPublish();

  void readImage(const cv::Mat& _img, double _cur_time, bool pub_this_frame);

  void setMask();

  bool updateID(unsigned int i);

  void readIntrinsicParameter(const string& calib_file);

  void showUndistortion(const string& name);

  void rejectWithF();

  void undistortedPoints();


  cv::Mat fisheye_mask;
  cv::Mat cur_img;
  vector<cv::Point2f> cur_un_pts, cur_pts;
  vector<cv::Point2f> pts_velocity;
  vector<int> ids;
  vector<int> track_cnt;
  camodocal::CameraPtr m_camera;

private:
  cv::Mat mask;

  cv::Mat prev_img, forw_img;
  vector<cv::Point2f> n_pts;
  vector<cv::Point2f> prev_pts, forw_pts;
  vector<cv::Point2f> prev_un_pts;

  // assign region id to each forward features
  std::vector<int> region_ids;
  // count number of points per region/block
  std::vector<int> region_pts;
  map<int, cv::Point2f> cur_un_pts_map;
  map<int, cv::Point2f> prev_un_pts_map;
  double cur_time;
  double prev_time;
  std::vector<int> region_threshold_;
  static int n_id;

  SequenceAnalysis feature_length_;
  std::vector<std::vector<cv::KeyPoint>> regional_kp_;

  int max_num_features_;
  int max_num_features_region_;
  bool init_tracking_;
  std::string feature_type_;

  void addPoints(const std::vector<cv::Point2f>& points, int region_id = 0);
  void addPoints(const std::vector<cv::KeyPoint>& points, int region_id = 0);
  void addPoints(const std::vector<CvPoint>& points, int region_id = 0);
  void reducePoints(const std::vector<uchar>& mask);
};
