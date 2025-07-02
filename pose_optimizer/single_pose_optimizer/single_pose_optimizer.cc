#include "pose_optimizer/single_pose_optimizer/single_pose_optimizer.h"

#include "pose_optimizer/common.h"

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimTranslation, int kDimRotation>
SinglePoseOptimizer<kDimTranslation, kDimRotation>::SinglePoseOptimizer() {}

template <int kDimTranslation, int kDimRotation>
bool SinglePoseOptimizer<kDimTranslation, kDimRotation>::Solve(
    const Problem<kDimTranslation, kDimRotation>& problem,
    const Options& options, Pose* pose, Summary* summary) {
  bool success = true;

  Pose optimized_pose = *pose;

  double lambda = options.damping_handle.initial_lambda;
  double previous_cost = std::numeric_limits<double>::max();
  int iteration = 0;
  for (; iteration < options.max_iterations; ++iteration) {
    HessianMatrix hessian{HessianMatrix::Zero()};
    GradientVector gradient{GradientVector::Zero()};

    double cost = 0.0;
    for (const auto& residual_block : problem.GetResidualBlocks()) {
      HessianMatrix local_hessian{HessianMatrix::Zero()};
      GradientVector local_gradient{GradientVector::Zero()};
      double local_cost = 0.0;
      residual_block->Evaluate(optimized_pose.rotation(),
                               optimized_pose.translation(), &local_hessian,
                               &local_gradient, &local_cost);
      AddLocalHessianOnlyUpperTriangle(local_hessian, &hessian);
      *gradient += local_gradient;
      cost += local_cost;
    }
    ReflectHessian(&hessian);

    // Damping hessian
    for (int k = 0; k < kDimPose; k++) hessian(k, k) *= 1.0 + lambda;

    // Compute the step
    const Eigen::Matrix<double, kDimPose, 1> update_step =
        hessian.ldlt().solve(-gradient);

    // Update the pose
    const Eigen::Matrix<double, kDimTranslation, 1> delta_t =
        update_step.template block<kDimTranslation, 1>(0, 0);
    const Eigen::Matrix<double, kDimRotation, 1> delta_R =
        update_step.template block<kDimRotation, 1>(kDimTranslation, 0);

    // Check convergence
    if (update_step.norm() < options.convergence_handle.parameter_tolerance) {
      break;
    }
    if (gradient.norm() < options.convergence_handle.gradient_tolerance) {
      break;
    }
    if (std::abs(cost - previous_cost) <
        options.convergence_handle.function_tolerance) {
      break;
    }

    const auto& damping_param = options.damping_handle;
    lambda *= (cost > previous_cost) ? damping_param.lambda_increasing_factor
                                     : damping_param.lambda_decreasing_factor;
    lambda =
        std::clamp(lambda, damping_param.min_lambda, damping_param.max_lambda);

    if (kDimRotation == 1)
      optimized_pose.rotate(delta_R);
    else if (kDimRotation == 3)
      optimized_pose.rotate(ComputeQuaternion(delta_R));
    optimized_pose.translate(delta_t);
    // Rotation part should be projected to the SO(3) manifold

    previous_cost = cost;
  }

  *pose = optimized_pose;

  return success;
}

template <int kDimTranslation, int kDimRotation>
void SinglePoseOptimizer<kDimTranslation, kDimRotation>::
    AddLocalHessianOnlyUpperTriangle(const HessianMatrix& local_hessian,
                                     HessianMatrix* hessian) {
  for (int i = 0; i < kDimPose; ++i)
    for (int j = i; j < kDimPose; ++j) (*hessian)(i, j) += local_hessian(i, j);
}

template <int kDimTranslation, int kDimRotation>
void SinglePoseOptimizer<kDimTranslation, kDimRotation>::ReflectHessian(
    HessianMatrix* hessian) {
  for (int i = 0; i < kDimPose; ++i)
    for (int j = i + 1; j < kDimPose; ++j) (*hessian)(j, i) = (*hessian)(i, j);
};

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer
