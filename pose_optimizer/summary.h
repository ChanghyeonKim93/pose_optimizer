#ifndef POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_SUMMARY_H_
#define POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_SUMMARY_H_

#include <string>
#include <vector>

namespace pose_optimizer {

enum class SolverType {
  LEVENBERG_MARQUARDT = 0,
  GRADIENT_DESCENT = 1,
  GAUSS_NEWTON = 2
};

enum class IterationStatus {
  UNDEFINED = -1,
  UPDATE = 0,
  UPDATE_TRUST_MORE = 1,
  SKIPPED = 2
};

struct OptimizationInfo {
  double cost{-1.0};
  double cost_change{-1.0};
  double average_reprojection_error{-1.0};
  double abs_gradient{-1.0};
  double abs_step{-1.0};
  double damping_term{-1.0};
  double iter_time{-1.0};
  IterationStatus iteration_status{IterationStatus::UNDEFINED};
};

class Summary {
 public:
  Summary();

  ~Summary();

  std::string BriefReport();

  std::string FullReport();

  const double GetTotalTimeInSecond() const;

  void AddOptimizationInfo(const OptimizationInfo& optimization_info) {
    optimization_infos_.push_back(optimization_info);
  }

 protected:
  std::vector<OptimizationInfo> optimization_infos_;
  int max_iteration_;
  double total_time_in_millisecond_;
  bool convergence_status_;
};

}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_SINGLE_POSE_OPTIMIZER_SUMMARY_H_