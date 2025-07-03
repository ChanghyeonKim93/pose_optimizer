#include "pose_optimizer/single_pose_optimizer/single_pose_optimizer.h"

#include "pose_optimizer/common.h"
#include "pose_optimizer/summary.h"
#include "pose_optimizer/time_monitor.h"

namespace pose_optimizer {
namespace single_pose_optimizer {

template <int kDimPose>
SinglePoseOptimizer<kDimPose>::SinglePoseOptimizer() {}

template <int kDimPose>
bool SinglePoseOptimizer<kDimPose>::Solve(const Problem<kDimPose>& problem,
                                          const Options& options, Pose* pose,
                                          Summary* summary) {
  bool success = true;

  const auto& residual_blocks = problem.GetResidualBlocks();

  OverallSummary overall_summary;
  overall_summary.max_iterations = options.max_iterations;
  overall_summary.num_residual_blocks = residual_blocks.size();
  overall_summary.num_parameter_blocks = 1;

  Pose optimized_pose = *pose;

  double lambda = options.damping_handle.initial_lambda;
  double previous_cost = std::numeric_limits<double>::max();
  int iteration = 0;
  for (; iteration < options.max_iterations; ++iteration) {
    StopWatchTimer::Start();

    HessianMatrix hessian{HessianMatrix::Zero()};
    GradientVector gradient{GradientVector::Zero()};

    double cost = 0.0;
    for (const auto& residual_block : residual_blocks) {
      HessianMatrix local_hessian{HessianMatrix::Zero()};
      GradientVector local_gradient{GradientVector::Zero()};
      double local_cost = 0.0;
      residual_block->Evaluate(optimized_pose.rotation(),
                               optimized_pose.translation(), &local_hessian,
                               &local_gradient, &local_cost);
      AddLocalHessianOnlyUpperTriangle(local_hessian, &hessian);
      gradient += local_gradient;
      cost += local_cost;
    }
    ReflectHessian(&hessian);

    // Damping hessian
    for (int k = 0; k < kDimPoseParam; k++) hessian(k, k) *= 1.0 + lambda;

    // Compute the step
    const Eigen::Matrix<double, kDimPoseParam, 1> update_step =
        hessian.ldlt().solve(-gradient);

    // Update the pose
    const Eigen::Matrix<double, kDimTranslation, 1> delta_t =
        update_step.template block<kDimTranslation, 1>(0, 0);
    const Eigen::Matrix<double, kDimRotationParam, 1> delta_R =
        update_step.template block<kDimRotationParam, 1>(kDimTranslation, 0);

    const auto& damping_param = options.damping_handle;
    lambda *= (cost > previous_cost) ? damping_param.lambda_increasing_factor
                                     : damping_param.lambda_decreasing_factor;
    lambda =
        std::clamp(lambda, damping_param.min_lambda, damping_param.max_lambda);

    StepSummary step_summary;
    step_summary.iteration = iteration;
    step_summary.iteration_time_in_seconds = StopWatchTimer::Stop();
    step_summary.cumulative_time_in_seconds +=
        step_summary.iteration_time_in_seconds;
    // step_summary.step_solver_time_in_seconds;
    step_summary.cost = cost;
    step_summary.cost_change = cost - previous_cost;
    step_summary.gradient_norm = gradient.norm();
    step_summary.step_norm = update_step.norm();
    step_summary.status = StepSummary::Status::UPDATE;
    // step_summary.step_size;
    // step_summary.step_solver_time_in_seconds;
    // step_summary.trust_region_radius;
    if (summary != nullptr) summary->SetStepSummary(step_summary);

    // Check convergence
    const auto& convergence_handle = options.convergence_handle;
    if (step_summary.step_norm < convergence_handle.parameter_tolerance) {
      break;
    }
    if (step_summary.gradient_norm < convergence_handle.gradient_tolerance) {
      break;
    }
    if (std::abs(step_summary.cost_change) <
        convergence_handle.function_tolerance) {
      break;
    }

    ApplyDeltaRotation(delta_R, &optimized_pose);
    optimized_pose.translate(delta_t);
    // Rotation part should be projected to the SO(3) manifold

    previous_cost = cost;
  }

  if (summary != nullptr) {
    overall_summary.is_converged = (iteration < options.max_iterations);
    summary->SetOverallSummary(overall_summary);
  }

  *pose = optimized_pose;

  return success;
}

template <int kDimPose>
void SinglePoseOptimizer<kDimPose>::AddLocalHessianOnlyUpperTriangle(
    const HessianMatrix& local_hessian, HessianMatrix* hessian) {
  for (int i = 0; i < kDimPoseParam; ++i)
    for (int j = i; j < kDimPoseParam; ++j)
      (*hessian)(i, j) += local_hessian(i, j);
}

template <int kDimPose>
void SinglePoseOptimizer<kDimPose>::ReflectHessian(HessianMatrix* hessian) {
  for (int i = 0; i < kDimPoseParam; ++i)
    for (int j = i + 1; j < kDimPoseParam; ++j)
      (*hessian)(j, i) = (*hessian)(i, j);
};

template class SinglePoseOptimizer<2>;
template class SinglePoseOptimizer<3>;

}  // namespace single_pose_optimizer
}  // namespace pose_optimizer
