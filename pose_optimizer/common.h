#ifndef POSE_OPTIMIZER_COMMON_H_
#define POSE_OPTIMIZER_COMMON_H_

#include "pose_optimizer/types.h"

namespace pose_optimizer {

double ComputeQuaternion(const double& w) { return w; }

Orientation ComputeQuaternion(const Eigen::Vector3d& w) {
  Orientation orientation{Orientation::Identity()};
  const double theta = w.norm();
  if (theta < 1e-6) {
    orientation.w() = 1.0;
    orientation.vec() = 0.5 * w;
  } else {
    const double half_theta = theta * 0.5;
    const double sin_half_theta_divided_theta = std::sin(half_theta) / theta;
    orientation.w() = std::cos(half_theta);
    orientation.vec() = sin_half_theta_divided_theta * w;
  }
  return orientation;
}

void ApplyDeltaRotation(const Eigen::Matrix<double, 1, 1>& delta_R,
                        Eigen::Isometry2d* pose) {
  pose->rotate(delta_R(0));  // 1D 회전 (예: 2D 평면 회전)
}

void ApplyDeltaRotation(const Eigen::Matrix<double, 3, 1>& delta_R,
                        Eigen::Isometry3d* pose) {
  pose->rotate(ComputeQuaternion(delta_R));
}

}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_COMMON_H_