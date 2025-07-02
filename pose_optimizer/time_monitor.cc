#include "pose_optimizer/time_monitor.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace pose_optimizer {

double TimeMonitorManager::min_time_threshold_{1e-7};
std::string TimeMonitorManager::save_file_name_{""};

TimeMonitorManager* TimeMonitorManager::GetSingletonInstance() {
  static TimeMonitorManager time_monitor_manager;
  return &time_monitor_manager;
}

TimeMonitor::TimeMonitor(const std::string& file_name,
                         const std::string& function_name)
    : name_(file_name + "/" + function_name),
      start_time_(std::chrono::high_resolution_clock::now()) {}

TimeMonitor::~TimeMonitor() {
  std::chrono::high_resolution_clock::time_point end_time =
      std::chrono::high_resolution_clock::now();
  const double time_in_msec = (end_time - start_time_).count() * 1e-6;
  TimeMonitorManager::GetSingletonInstance()->RegisterTime(name_, time_in_msec);
}

TimeMonitorManager::TimeMonitorManager() {}

void TimeMonitorManager::RegisterTime(const std::string& time_monitor_name,
                                      const double time) {
  if (time_list_of_functions_.find(time_monitor_name) ==
      time_list_of_functions_.end()) {
    time_list_of_functions_.insert({time_monitor_name, std::vector<double>()});
    time_list_of_functions_.at(time_monitor_name).reserve(1000000);
  }
  time_list_of_functions_.at(time_monitor_name).push_back(time);
}

void TimeMonitorManager::SetMinTimeThreshold(const double min_time_threshold) {
  min_time_threshold_ = min_time_threshold;
}

void TimeMonitorManager::SaveFile(const std::string& file_name) {
  save_file_name_ = file_name;
}

TimeMonitorManager::~TimeMonitorManager() {
  std::stringstream ss;
  ss << "------------ Time Analysis ------------" << std::endl;
  ss.precision(6);
  ss.setf(std::ios::fixed);
  for (auto& [time_monitor_name, time_list] : time_list_of_functions_) {
    SortTimeList(&time_list);
    ss << time_monitor_name << "\n";
    ss << std::setw(18) << std::setfill(' ') << "calls: " << time_list.size()
       << "\n";
    if (time_list.empty()) continue;
    ss << std::setw(18) << std::setfill(' ');
    ss << "min: ";
    ss << std::setw(15) << std::setfill(' ') << GetMin(time_list) << " [ms]\n";
    ss << std::setw(18) << std::setfill(' ');
    ss << "max: ";
    ss << std::setw(15) << std::setfill(' ') << GetMax(time_list) << " [ms]\n";
    ss << std::setw(18) << std::setfill(' ');
    ss << "avg: ";
    ss << std::setw(15) << std::setfill(' ') << GetAverage(time_list)
       << " [ms]\n";
    ss << std::setw(18) << std::setfill(' ');
    ss << "std: ";
    ss << std::setw(15) << std::setfill(' ') << GetStd(time_list) << " [ms]\n";
    ss << std::setw(18) << std::setfill(' ');
    ss << "total occupancy: ";
    ss << std::setw(15) << std::setfill(' ') << GetSum(time_list) << " [ms]\n";
    ss << " -----------------------------------------------\n";
  }
  std::cerr << ss.str();

  if (save_file_name_.size() > 0) {
    std::cerr << "Save file!\n";
  }
}

void TimeMonitorManager::SortTimeList(std::vector<double>* time_list) {
  std::sort(time_list->begin(), time_list->end());
}

double TimeMonitorManager::GetMin(const std::vector<double>& time_list) {
  return time_list.front();
}

double TimeMonitorManager::GetMax(const std::vector<double>& time_list) {
  return time_list.back();
}

double TimeMonitorManager::GetSum(const std::vector<double>& time_list) {
  double time_sum = 0.0;
  for (const auto& time : time_list) time_sum += time;
  return time_sum;
}

double TimeMonitorManager::GetAverage(const std::vector<double>& time_list) {
  if (time_list.empty()) return 0.0;
  return (GetSum(time_list) / static_cast<double>(time_list.size()));
}

double TimeMonitorManager::GetStd(const std::vector<double>& time_list) {
  if (time_list.size() <= 1) return 0.0;
  double squared_time_sum = 0.0;
  for (const auto& time : time_list) squared_time_sum += time * time;
  const double avg_time = GetAverage(time_list);
  return std::sqrt(squared_time_sum / static_cast<double>(time_list.size()) -
                   avg_time * avg_time);
}

}  // namespace pose_optimizer