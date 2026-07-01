#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_mpc/SystemObservation.h>

#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class AmpDataLogger {
 public:
  struct Config {
    std::string logDir = "amp_data";
    std::string logPrefix = "motion";
    bool logToFile = true;
    bool logToRos = true;
    bool startRecording = false;
    double logFrequency = 50.0;
    int numJoints = 12;
    int numFeet = 4;
  };

  AmpDataLogger(const Config& config, ros::NodeHandle& nh);
  ~AmpDataLogger();

  void log(double time,
           const vector_t& optimizedState,
           const vector_t& optimizedInput,
           const vector_t& wbcOutput,
           const vector_t& measuredRbdState,
           size_t plannedMode,
           const CentroidalModelInfo& info);

  void startNewSequence(const std::string& gaitName);
  void stopLogging();
  void flush();

 private:
  void openFile();
  void closeFile();
  void writeHeader();
  void publishRosData(double time,
                      const vector_t& optimizedState,
                      const vector_t& optimizedInput,
                      const vector_t& wbcOutput,
                      size_t plannedMode,
                      const CentroidalModelInfo& info);

  Config config_;
  std::ofstream logFile_;
  std::mutex logMutex_;
  bool isLogging_{false};
  int sequenceCount_{0};
  int frameCount_{0};
  double lastLogTime_{0.0};
  double firstRecordedTime_{0.0};
  double lastRecordedTime_{0.0};
  double sequenceStartWallTime_{0.0};
  bool hasRecordedFrame_{false};
  std::string currentGaitName_{"unknown"};
  std::string currentLogFile_;

  ros::Publisher ampDataPub_;
  ros::Publisher ampContactPub_;
  ros::Subscriber loggingCmdSub_;
  ros::Subscriber gaitNameSub_;

  void loggingCmdCallback(const std_msgs::Bool::ConstPtr& msg);
  void gaitNameCallback(const std_msgs::String::ConstPtr& msg);
};

}  // namespace legged
