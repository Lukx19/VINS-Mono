#pragma once

#include <ceres/ceres.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include "../parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
public:
  ProjectionFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j);
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;
  void check(double** parameters);

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};

class ProjectionFactorAuto {
public:
  ProjectionFactorAuto(const Eigen::Vector3d& pts_i,
                       const Eigen::Vector3d& pts_j, const Eigen::Matrix2d& sqrt_info)
    : sqrt_info_(sqrt_info),tangent_base_(),pts_i_(pts_i), pts_j_(pts_j) {
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3 b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp)
      tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
  }

  template <typename T>
  bool operator()(const T* const pose_i, const T* const pose_j, const T* const ex_pose,
                  const T* const feature, T* residuals) const {
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector2 = Eigen::Matrix<T, 2, 1>;

    Vector3 Pi(pose_i[0], pose_i[1], pose_i[2]);
    Eigen::Quaternion<T> Qi(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);

    Vector3 Pj(pose_j[0], pose_j[1], pose_j[2]);
    Eigen::Quaternion<T> Qj(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);

    Vector3 tic(ex_pose[0], ex_pose[1], ex_pose[2]);
    Eigen::Quaternion<T> qic(ex_pose[6], ex_pose[3], ex_pose[4], ex_pose[5]);

    T depth_i = feature[0];

    Vector3 pts_j_t = pts_j_.cast<T>();
    Vector3 pts_i_t = pts_i_.cast<T>();
    Eigen::Matrix<T, 2, 2> sqrt_info_t = sqrt_info_.cast<T>();
    Eigen::Matrix<T, 2, 3> tangent_base_t = tangent_base_.cast<T>();
    Vector3 pts_camera_i = pts_i_t / depth_i;
    Vector3 pts_imu_i = qic * pts_camera_i + tic;
    Vector3 pts_w = Qi * pts_imu_i + Pi;
    Vector3 pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Vector3 pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Map<Vector2> residual(residuals);
#ifdef UNIT_SPHERE_ERROR
    residual = tangent_base_t * (pts_camera_j.normalized() - pts_j_t.normalized());
#else
    T dep_j = pts_camera_j.z();
    residual(0) = (pts_camera_j(0) / dep_j) - pts_j_t(0);
    residual(1) = (pts_camera_j(1) / dep_j) - pts_j_t(1);
    // residual = (pts_camera_j / dep_j).head<2>() - pts_j_t.head<2>();
#endif
    residual = sqrt_info_t * residual;
    return true;
  }

private:
  Eigen::Matrix2d sqrt_info_;
  Eigen::Matrix<double, 2, 3> tangent_base_;
  Eigen::Vector3d pts_i_, pts_j_;
};

// struct ReprojectionError {
//   ReprojectionError(const Eigen::Vector3d observed, const Eigen::Vector3d worldPoint) {
//     observed_x = observed.x() / observed.z();
//     observed_y = observed.y() / observed.z();

//     X = worldPoint.x();
//     Y = worldPoint.y();
//     Z = worldPoint.z();
//   }

//   // Factory to hide the construction of the CostFunction object from the client code.
//   static ceres::CostFunction* Create(const Eigen::Vector3d observed,
//                                      const Eigen::Vector3d worldPoint) {
//     return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>(
//         new ReprojectionError(observed, worldPoint)));
//   }

//   template <typename T>
//   bool operator()(const T* const camera_rotation, const T* const camera_translation,
//                   T* residuals) const {
//     // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
//     Eigen::Matrix<T, 3, 1> point;
//     point << T(X), T(Y), T(Z);

//     // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
//     Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(camera_rotation);

//     // Rotate the point using Eigen rotations
//     Eigen::Matrix<T, 3, 1> p = q * point;

//     // Map T* to Eigen Vector3 with correct Scalar type
//     Eigen::Matrix<T, 3, 1> t = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(camera_translation);
//     p += t;

//     // Cast CCS 3D point to plane at z = 1
//     T xp = p[0] / p[2];
//     T yp = p[1] / p[2];

//     // The error is the difference between the predicted and observed position.
//     residuals[0] = xp - T(observed_x);
//     residuals[1] = yp - T(observed_y);

//     return true;
//   }

//   double observed_x;
//   double observed_y;
//   double X;
//   double Y;
//   double Z;
// };

// ceres::Problem problem;

// // Use my LocalParameterization
// ceres::LocalParameterization* quaternion_parameterization =
//     new ceres_ext::EigenQuaternionParameterization;

// for (auto cam = cameras.begin(); cam != cameras.end(); cam++) {
//   for (auto fpt = cam->features.begin(); fpt != cam->features.end(); fpt++) {
//     ceres::CostFunction* cost_function =
//         ReprojectionError::Create(fpt->ImgPoint->X, fpt->worldPoint->X);
//     problem.AddResidualBlock(cost_function, NULL, cam->q.coeffs().data(), cam->t.data());
//   }

//   // Apply the parameterization
//   problem.SetParameterization(cam->q.coeffs().data(), quaternion_parameterization);
// }

// // Set a few options
// ceres::Solver::Options options;
// options.use_nonmonotonic_steps = true;
// options.preconditioner_type = ceres::SCHUR_JACOBI;
// options.linear_solver_type = ceres::DENSE_SCHUR;
// options.max_num_iterations = 100;

// ceres::Solver::Summary summary;
// ceres::Solve(options, &problem, &summary);

// std::cout << "Final report:\n" << summary.FullReport();