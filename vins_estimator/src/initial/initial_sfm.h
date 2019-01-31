#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cstdlib>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>


struct SFMFeature {
  bool state;
  int id;

  using Observation = std::vector<std::pair<int, Eigen::Vector3d>,
                                  Eigen::aligned_allocator<std::pair<int, Eigen::Vector3d>>>;
  // x,y coordinates in camera frame and depth information. If depth not available than -1
  Observation observation;
  // x,y,z coordinates in [m] in world frame
  double position[3];
  // not used
  double depth;
};

struct ReprojectionError3D {
  ReprojectionError3D(double observed_u, double observed_v)
    : observed_u(observed_u), observed_v(observed_v) {
  }

  template <typename T>
  bool operator()(const T* const camera_R, const T* const camera_T, const T* point,
                  T* residuals) const {
    T p[3];
    ceres::QuaternionRotatePoint(camera_R, point, p);
    p[0] += camera_T[0];
    p[1] += camera_T[1];
    p[2] += camera_T[2];
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    residuals[0] = xp - T(observed_u);
    residuals[1] = yp - T(observed_v);
    return true;
  }

  static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
        new ReprojectionError3D(observed_x, observed_y)));
  }

  double observed_u;
  double observed_v;
};

void triangulatePoint(const Eigen::Matrix<double, 3, 4>& Pose0,
                      const Eigen::Matrix<double, 3, 4>& Pose1, const Eigen::Vector2d& point0,
                      const Eigen::Vector2d& point1, Eigen::Vector3d& point_3d);

bool solveFrameByPnP(Eigen::Matrix3d& R_initial, Eigen::Vector3d& P_initial, int i,
                     const std::vector<SFMFeature>& sfm_f);

void triangulateTwoFrames(int frame0, const Eigen::Matrix<double, 3, 4>& Pose0, int frame1,
                          const Eigen::Matrix<double, 3, 4>& Pose1, std::vector<SFMFeature>& sfm_f);

// convert from camera coordinates into world coordinates in camera frame with known depth.
Eigen::Vector3d toImage3DCoords(const Eigen::Vector3d& point);

void triangulatePointWithDepth(const Eigen::Matrix<double, 3, 4>& pose0,
                               const Eigen::Matrix<double, 3, 4>& pose1, const Eigen::Vector3d& point0,
                               const Eigen::Vector3d& point1, Eigen::Vector3d& point_3d);

void triangulateWithDepth(int frame0, const Eigen::Matrix<double, 3, 4>& Pose0, int frame1,
                          const Eigen::Matrix<double, 3, 4>& Pose1, std::vector<SFMFeature>& sfm_f);

bool globalSFM(int frame_num, Eigen::Quaterniond* q, Eigen::Vector3d* T, int l,
               const Eigen::Matrix3d& relative_R, const Eigen::Vector3d& relative_T,
               std::vector<SFMFeature>& sfm_f, std::map<int, Eigen::Vector3d>& sfm_tracked_points);
// class GlobalSFM {
// public:
//   GlobalSFM();

//   int feature_num;
// };