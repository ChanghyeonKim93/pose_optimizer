#ifndef POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_H_
#define POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_H_

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>

#include "Eigen/Dense"

#include "pose_optimizer/options.h"
#include "pose_optimizer/single_pose_optimizer/problem.h"
#include "pose_optimizer/summary.h"
#include "pose_optimizer/types.h"

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimPose>
class SinglePoseOptimizer {
  static const int kDimTranslation = kDimPose;
  static const int kDimRotation = kDimPose;
  static const int kDimRotationParam = kDimPose == 2 ? 1 : 3;
  static const int kDimPoseParam = kDimPose == 2 ? 3 : 6;
  using Pose = Eigen::Transform<double, kDimPose, 1>;
  using HessianMatrix = Eigen::Matrix<double, kDimPoseParam, kDimPoseParam>;
  using GradientVector = Eigen::Matrix<double, kDimPoseParam, 1>;
  static_assert(kDimPose == 2 || kDimPose == 3,
                "SinglePoseOptimizer only supports 2D and 3D poses.");

 public:
  SinglePoseOptimizer();

  bool Solve(const Problem<kDimPose>& problem, const Options& options,
             Pose* pose, Summary* summary = nullptr);

 private:
  void AddLocalHessianOnlyUpperTriangle(const HessianMatrix& local_hessian,
                                        HessianMatrix* hessian);
  void ReflectHessian(HessianMatrix* hessian);
};

// 명시적 인스턴스화를 선언
extern template class SinglePoseOptimizer<2>;
extern template class SinglePoseOptimizer<3>;

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer

#endif  // NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_POSE_OPTIMIZER_H_