#include "legged_controllers/AmpDataLogger.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <algorithm>
#include <cerrno>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace legged {
namespace {

bool ensureDirectory(const std::string& path) {
  if (path.empty()) {
    return false;
  }

  std::string current;
  size_t index = 0;
  if (path[0] == '/') {
    current = "/";
    index = 1;
  }

  while (index <= path.size()) {
    const size_t next = path.find('/', index);
    const std::string part = path.substr(index, next == std::string::npos ? std::string::npos : next - index);
    if (!part.empty()) {
      if (!current.empty() && current.back() != '/') {
        current += "/";
      }
      current += part;
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
      }
    }

    if (next == std::string::npos) {
      break;
    }
    index = next + 1;
  }

  return true;
}

}  // namespace

AmpDataLogger::AmpDataLogger(const Config& config, ros::NodeHandle& nh) : config_(config) {
  if (config_.logToRos) {
    ampDataPub_ = nh.advertise<std_msgs::Float64MultiArray>("/amp/motion_data", 1);
    ampContactPub_ = nh.advertise<std_msgs::Float64MultiArray>("/amp/contact_data", 1);
    loggingCmdSub_ = nh.subscribe<std_msgs::Bool>("/amp/enable_logging", 1,
                                                   &AmpDataLogger::loggingCmdCallback, this);
    gaitNameSub_ = nh.subscribe<std_msgs::String>("/amp/gait_name", 1,
                                                  &AmpDataLogger::gaitNameCallback, this);
  }

  if (config_.startRecording && config_.logToFile) {
    openFile();
  } else if (config_.startRecording) {
    isLogging_ = true;
  }

  ROS_INFO("[AmpDataLogger] Initialized. Log dir: %s, Log to file: %d, Log to ROS: %d, Recording: %d",
           config_.logDir.c_str(), config_.logToFile, config_.logToRos, isLogging_);
  if (!isLogging_) {
    ROS_INFO("[AmpDataLogger] Recording is idle. Publish true to /amp/enable_logging, or press 'l' in keyboard control, to start.");
  }
}

AmpDataLogger::~AmpDataLogger() {
  closeFile();
}

void AmpDataLogger::openFile() {
  std::lock_guard<std::mutex> lock(logMutex_);
  if (!ensureDirectory(config_.logDir)) {
    ROS_ERROR("[AmpDataLogger] Failed to create log directory: %s", config_.logDir.c_str());
    isLogging_ = false;
    return;
  }

  std::stringstream ss;
  ss << config_.logDir << "/" << config_.logPrefix << "_"
     << std::setfill('0') << std::setw(4) << sequenceCount_ << ".csv";

  currentLogFile_ = ss.str();
  logFile_.open(currentLogFile_);
  if (logFile_.is_open()) {
    writeHeader();
    isLogging_ = true;
    frameCount_ = 0;
    firstRecordedTime_ = 0.0;
    lastRecordedTime_ = 0.0;
    lastLogTime_ = 0.0;
    hasRecordedFrame_ = false;
    sequenceStartWallTime_ = ros::WallTime::now().toSec();
    sequenceCount_++;
    ROS_INFO("[AmpDataLogger] Recording started. File: %s", currentLogFile_.c_str());
  } else {
    isLogging_ = false;
    ROS_ERROR("[AmpDataLogger] Failed to open log file: %s", currentLogFile_.c_str());
  }
}

void AmpDataLogger::closeFile() {
  std::lock_guard<std::mutex> lock(logMutex_);
  if (logFile_.is_open()) {
    logFile_.flush();
    logFile_.close();
    isLogging_ = false;
    const double motionDuration = hasRecordedFrame_ ? std::max(0.0, lastRecordedTime_ - firstRecordedTime_) : 0.0;
    const double wallDuration = std::max(0.0, ros::WallTime::now().toSec() - sequenceStartWallTime_);
    const double rateDuration = motionDuration > 1e-6 ? motionDuration : wallDuration;
    const double averageRate = rateDuration > 1e-6 ? static_cast<double>(frameCount_) / rateDuration : 0.0;
    ROS_INFO("[AmpDataLogger] Recording stopped. File: %s, frames: %d, motion duration: %.3f s, wall duration: %.3f s, average rate: %.2f Hz",
             currentLogFile_.c_str(), frameCount_, motionDuration, wallDuration, averageRate);
  } else if (isLogging_) {
    isLogging_ = false;
    ROS_INFO("[AmpDataLogger] Recording stopped. ROS-only recording, frames: %d", frameCount_);
  }
}

void AmpDataLogger::writeHeader() {
  logFile_ << "time";

  logFile_ << ",base_px,base_py,base_pz";
  logFile_ << ",base_yaw,base_pitch,base_roll";
  logFile_ << ",root_lin_vel_bx,root_lin_vel_by,root_lin_vel_bz";
  logFile_ << ",root_ang_vel_bx,root_ang_vel_by,root_ang_vel_bz";

  for (int i = 0; i < config_.numJoints; ++i) {
    logFile_ << ",q" << i;
  }

  for (int i = 0; i < config_.numJoints; ++i) {
    logFile_ << ",dq" << i;
  }

  for (int i = 0; i < config_.numFeet; ++i) {
    logFile_ << ",contact" << i;
  }

  logFile_ << ",gait_name";
  logFile_ << "\n";
}

void AmpDataLogger::log(double time,
                         const vector_t& optimizedState,
                         const vector_t& optimizedInput,
                         const vector_t& wbcOutput,
                         const vector_t& measuredRbdState,
                         size_t plannedMode,
                         const CentroidalModelInfo& info) {
  if (!isLogging_) {
    return;
  }

  double dt = 1.0 / config_.logFrequency;
  if (time - lastLogTime_ < dt && lastLogTime_ > 0.0) {
    return;
  }
  lastLogTime_ = time;

  if (config_.logToRos) {
    publishRosData(time, optimizedState, optimizedInput, wbcOutput, plannedMode, info);
  }

  if (config_.logToFile && isLogging_) {
    std::lock_guard<std::mutex> lock(logMutex_);
    if (!logFile_.is_open()) return;

    logFile_ << std::fixed << std::setprecision(6) << time;

    for (int i = 6; i < 12; ++i) {
      if (i < optimizedState.size()) {
        logFile_ << "," << optimizedState(i);
      } else {
        logFile_ << ",0.0";
      }
    }

    vector3_t rootLinVelBody = vector3_t::Zero();
    vector3_t rootAngVelBody = vector3_t::Zero();
    if (measuredRbdState.size() >= 2 * info.generalizedCoordinatesNum) {
      const vector3_t baseZyx = measuredRbdState.segment<3>(0);
      const matrix3_t worldToBody = getRotationMatrixFromZyxEulerAngles(baseZyx).transpose();
      rootAngVelBody.noalias() = worldToBody * measuredRbdState.segment<3>(info.generalizedCoordinatesNum);
      rootLinVelBody.noalias() = worldToBody * measuredRbdState.segment<3>(info.generalizedCoordinatesNum + 3);
    }
    for (int i = 0; i < 3; ++i) {
      logFile_ << "," << rootLinVelBody(i);
    }
    for (int i = 0; i < 3; ++i) {
      logFile_ << "," << rootAngVelBody(i);
    }

    vector_t jointAngles = centroidal_model::getJointAngles(optimizedState, info);
    for (int i = 0; i < jointAngles.size(); ++i) {
      logFile_ << "," << jointAngles(i);
    }

    vector_t jointVels = centroidal_model::getJointVelocities(optimizedInput, info);
    for (int i = 0; i < jointVels.size(); ++i) {
      logFile_ << "," << jointVels(i);
    }

    contact_flag_t contacts = modeNumber2StanceLeg(plannedMode);
    for (int i = 0; i < config_.numFeet; ++i) {
      logFile_ << "," << (contacts[i] ? 1.0 : 0.0);
    }

    logFile_ << "," << currentGaitName_;
    logFile_ << "\n";

    if (!hasRecordedFrame_) {
      firstRecordedTime_ = time;
      hasRecordedFrame_ = true;
    }
    lastRecordedTime_ = time;
    frameCount_++;

    if (frameCount_ % 100 == 0) {
      logFile_.flush();
    }
  }
}

void AmpDataLogger::publishRosData(double time,
                                    const vector_t& optimizedState,
                                    const vector_t& optimizedInput,
                                    const vector_t& wbcOutput,
                                    size_t plannedMode,
                                    const CentroidalModelInfo& info) {
  std_msgs::Float64MultiArray motionMsg;
  motionMsg.data.push_back(time);

  for (int i = 0; i < optimizedState.size(); ++i) {
    motionMsg.data.push_back(optimizedState(i));
  }
  for (int i = 0; i < optimizedInput.size(); ++i) {
    motionMsg.data.push_back(optimizedInput(i));
  }
  for (int i = 0; i < wbcOutput.size(); ++i) {
    motionMsg.data.push_back(wbcOutput(i));
  }
  ampDataPub_.publish(motionMsg);

  std_msgs::Float64MultiArray contactMsg;
  contact_flag_t contacts = modeNumber2StanceLeg(plannedMode);
  for (int i = 0; i < config_.numFeet; ++i) {
    contactMsg.data.push_back(contacts[i] ? 1.0 : 0.0);
  }
  ampContactPub_.publish(contactMsg);
}

void AmpDataLogger::startNewSequence(const std::string& gaitName) {
  currentGaitName_ = gaitName;
  if (config_.logToFile && isLogging_) {
    closeFile();
    openFile();
  }
  ROS_INFO("[AmpDataLogger] Starting new sequence for gait: %s", gaitName.c_str());
}

void AmpDataLogger::stopLogging() {
  closeFile();
}

void AmpDataLogger::flush() {
  std::lock_guard<std::mutex> lock(logMutex_);
  if (logFile_.is_open()) {
    logFile_.flush();
  }
}

void AmpDataLogger::loggingCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data && !isLogging_) {
    if (config_.logToFile) {
      openFile();
    } else {
      frameCount_ = 0;
      firstRecordedTime_ = 0.0;
      lastRecordedTime_ = 0.0;
      lastLogTime_ = 0.0;
      hasRecordedFrame_ = false;
      sequenceStartWallTime_ = ros::WallTime::now().toSec();
      isLogging_ = true;
    }
    ROS_INFO("[AmpDataLogger] Recording enabled via ROS topic");
  } else if (!msg->data && isLogging_) {
    stopLogging();
    ROS_INFO("[AmpDataLogger] Recording disabled via ROS topic");
  } else if (msg->data && isLogging_) {
    ROS_INFO("[AmpDataLogger] Recording is already active. File: %s", currentLogFile_.c_str());
  } else {
    ROS_INFO("[AmpDataLogger] Recording is already idle.");
  }
}

void AmpDataLogger::gaitNameCallback(const std_msgs::String::ConstPtr& msg) {
  if (!msg->data.empty()) {
    currentGaitName_ = msg->data;
    ROS_INFO("[AmpDataLogger] Gait name set to: %s", currentGaitName_.c_str());
  }
}

}  // namespace legged
