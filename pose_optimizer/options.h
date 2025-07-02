#ifndef POSE_OPTIMIZER_OPTIONS_H_
#define POSE_OPTIMIZER_OPTIONS_H_

namespace pose_optimizer {

struct Options {
  int max_iterations = 100;
  struct {
    double parameter_tolerance = 1e-8;
    double gradient_tolerance = 1e-10;
    double function_tolerance = 1e-6;
  } convergence_handle;
  struct {
    double min_step_size = 1e-6;
    double max_step_size = 1.0;
  } step_size_handle;
  struct {
    double min_lambda = 1e-6;
    double max_lambda = 1e-2;
    double initial_lambda = 0.001;
    double lambda_increasing_factor = 2.0;
    double lambda_decreasing_factor = 0.6;
  } damping_handle;
};

}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_OPTIONS_H_