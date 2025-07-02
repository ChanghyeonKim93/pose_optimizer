#include "pose_optimizer/summary.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include "pose_optimizer/text_colorizer.h"

namespace pose_optimizer {

Summary::Summary() {}

Summary::~Summary() {}

std::string Summary::GetBriefReport() const {
  const std::streamsize default_precision{std::cout.precision()};
  std::stringstream ss;
  ss << "itr ";            // 5
  ss << "  total_cost  ";  // 14
  ss << " cost_change ";   // 13
  ss << " |step|  ";       // 10
  ss << " |gradient| ";    // 12
  ss << " damp_term ";     // 12
  ss << " itr_time[ms] ";  // 11
  ss << "itr_stat\n";
  const size_t num_iterations = step_summary_list_.size();
  for (size_t iteration = 0; iteration < num_iterations; ++iteration) {
    const StepSummary& step_summary = step_summary_list_[iteration];

    ss << std::setw(3) << iteration << " ";
    ss << " " << std::scientific << step_summary.cost;
    ss << "    " << std::setprecision(2) << std::scientific
       << step_summary.cost_change;
    ss << "   " << std::setprecision(2) << std::scientific
       << step_summary.step_norm;
    ss << "   " << std::setprecision(2) << std::scientific
       << step_summary.gradient_norm;
    ss << "   " << std::setprecision(2) << std::scientific
       << step_summary.trust_region_radius;
    ss << "   " << std::setprecision(2) << std::scientific
       << step_summary.iteration_time_in_seconds;

    switch (step_summary.status) {
      case StepSummary::Status::UPDATE:
        ss << "     "
           << "UPDATE";
        break;
      case StepSummary::Status::SKIPPED:
        ss << "     " << TEXT_YELLOW(" SKIP ");
        break;
      case StepSummary::Status::UPDATE_TRUST_MORE:
        ss << "     " << TEXT_GREEN("UPDATE");
        break;
      default:
        ss << "     ";
    }
    ss << "\n";
    ss << std::setprecision(default_precision);  // restore defaults
  }

  ss << std::setprecision(5);
  ss << "Analytic Solver Report:\n";
  ss << "  Iterations      : " << num_iterations << "\n";
  ss << "  Total time      : "
     << step_summary_list_.back().cumulative_time_in_seconds * 0.001
     << " [second]\n";
  ss << "  Initial cost    : " << step_summary_list_.front().cost << "\n";
  ss << "  Final cost      : " << step_summary_list_.back().cost << "\n";
  ss << ", Termination     : "
     << (overall_summary_.is_converged ? TEXT_GREEN("CONVERGENCE")
                                       : TEXT_YELLOW("NO_CONVERGENCE"))
     << "\n";
  if (overall_summary_.max_iterations == num_iterations) {
    ss << TEXT_YELLOW(
        " WARNIING: MAX ITERATION is reached ! The solution could be local "
        "minima.\n");
  }
  ss << std::setprecision(default_precision);  // restore defaults
  return ss.str();
}

double Summary::GetTotalTimeInSeconds() const {
  if (is_step_summary_set_)
    throw std::runtime_error("step_summary is not set.");
  const double total_time_in_seconds =
      step_summary_list_.back().cumulative_time_in_seconds;
  return total_time_in_seconds;
}

void Summary::SetStepSummary(const StepSummary& step_summary) {
  step_summary_list_.push_back(step_summary);
  is_step_summary_set_ = true;
}

void Summary::SetOverallSummary(const OverallSummary& overall_summary) {
  overall_summary_ = overall_summary;
  is_overall_summary_set_ = true;
}

}  // namespace pose_optimizer