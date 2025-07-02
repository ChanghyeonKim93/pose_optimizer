#ifndef POSE_OPTIMIZER_SUMMARY_H_
#define POSE_OPTIMIZER_SUMMARY_H_

#include <string>
#include <vector>

namespace pose_optimizer {

struct OverallSummary {
  int num_residual_blocks{-1};
  int num_parameter_blocks{-1};
  int max_iterations{-1};
  bool is_converged{true};
};

struct StepSummary {
  enum class Status {
    UNDEFINED = -1,
    UPDATE = 0,
    UPDATE_TRUST_MORE = 1,
    SKIPPED = 2
  };

  int iteration{-1};
  double iteration_time_in_seconds{0.0};
  double cumulative_time_in_seconds{0.0};
  double step_solver_time_in_seconds{0.0};
  double cost{0.0};
  double cost_change{0.0};
  double gradient_norm{0.0};
  double step_norm{0.0};
  double step_size{0.0};
  double trust_region_radius{0.0};
  Status status{Status::UNDEFINED};
};

class Summary {
 public:
  Summary();

  ~Summary();

  std::string GetBriefReport() const;

  double GetTotalTimeInSeconds() const;

  void SetStepSummary(const StepSummary& step_summary);

  void SetOverallSummary(const OverallSummary& overall_summary);

 private:
  bool is_step_summary_set_{false};
  bool is_overall_summary_set_{false};
  std::vector<StepSummary> step_summary_list_;
  OverallSummary overall_summary_;
};

}  // namespace pose_optimizer

#endif  // POSE_OPTIMIZER_SUMMARY_H_