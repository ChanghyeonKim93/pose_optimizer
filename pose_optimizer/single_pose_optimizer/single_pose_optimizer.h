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

template <int kDimTranslation, int kDimRotation>
class SinglePoseOptimizer {
  static constexpr int kDimPose = kDimTranslation + kDimRotation;
  using HessianMatrix = Eigen::Matrix<double, kDimPose, kDimPose>;
  using GradientVector = Eigen::Matrix<double, kDimPose, 1>;
  using Pose = Eigen::Transform<double, kDimTranslation, 1>;

 public:
  SinglePoseOptimizer();

  bool Solve(const Problem<kDimTranslation, kDimRotation>& problem,
             const Options& options, Pose* pose, Summary* summary = nullptr);

 private:
  void AddLocalHessianOnlyUpperTriangle(const HessianMatrix& local_hessian,
                                        HessianMatrix* hessian);
  void ReflectHessian(HessianMatrix* hessian);
};

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer

#endif  // NONLINEAR_OPTIMIZER_POSE_OPTIMIZER_POSE_OPTIMIZER_H_