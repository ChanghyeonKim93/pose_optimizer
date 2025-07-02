#ifndef POSE_OPTIMIZER_TIME_MONITOR_H_
#define POSE_OPTIMIZER_TIME_MONITOR_H_

#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace pose_optimizer {

class StopWatchTimer {
 public:
  inline static void Start() {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  inline static double Stop() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = end_time - start_time_;
    return elapsed_time.count() * 1e-9;  // 초 단위로 반환
  }

 private:
  inline static std::chrono::high_resolution_clock::time_point start_time_;
};

#define CHECK_EXEC_TIME_FROM_HERE \
  pose_optimizer::TimeMonitor time_monitor(__FILE__, __func__);

class TimeMonitor {
 public:
  TimeMonitor(const std::string& file_name, const std::string& function_name);
  ~TimeMonitor();

 private:
  const std::string name_;
  const std::chrono::high_resolution_clock::time_point start_time_;
};

class TimeMonitorManager {
  friend class TimeMonitor;

 public:
  TimeMonitorManager(const TimeMonitorManager& time_monitor_manager) = delete;

  static void SetMinTimeThreshold(const double min_time_threshold);
  static void SaveFile(const std::string& file_name);
  static TimeMonitorManager* GetSingletonInstance();

 protected:
  void RegisterTime(const std::string& time_monitor_name, const double time);

 private:
  TimeMonitorManager();
  ~TimeMonitorManager();

  void SortTimeList(std::vector<double>* time_list);
  double GetMin(const std::vector<double>& time_list);
  double GetMax(const std::vector<double>& time_list);
  double GetSum(const std::vector<double>& time_list);
  double GetAverage(const std::vector<double>& time_list);
  double GetStd(const std::vector<double>& time_list);

  std::map<std::string, std::vector<double>> time_list_of_functions_;

  static double min_time_threshold_;
  static std::string save_file_name_;
};

}  // namespace pose_optimizer

// #define log(fmt, ...)
//   printf("[%s: %d][%s] " fmt "\t\t\t (%s, %s)\n", __FILE__, __LINE__,
//          __func__, __DATE__, __TIME__);

#endif  // POSE_OPTIMIZER_TIME_MONITOR_H_
