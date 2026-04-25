#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include "serviceinterface.h"

namespace aubo_ros2_control
{
struct AuboConnectionOptions
{
  std::string ip{"192.168.1.107"};
  int port{8899};
  std::string username{"aubo"};
  std::string password{"123456"};
  uint8_t collision_class{6};
  bool startup_read_pose{true};
  bool shutdown_on_disconnect{false};
  double max_joint_acc_rad{50.0 * M_PI / 180.0};
  double max_joint_vel_rad{50.0 * M_PI / 180.0};
};

class AuboCommander
{
public:
  explicit AuboCommander(AuboConnectionOptions options = {});
  ~AuboCommander();

  bool connect(std::string * error_message = nullptr);
  void disconnect();

  bool isConnected() const;

  bool readJointPositions(std::array<double, 6> & joints);
  bool sendJointMove(const std::array<double, 6> & joints, bool blocking);
  bool sendFollowModeJointMove(const std::array<double, 6> & joints);

private:
  static void jointStatusCallback(
    const aubo_robot_namespace::JointStatus * joint_status,
    int size,
    void * arg);

  bool startupRobot(std::string * error_message);
  bool configureMoveProfile(std::string * error_message);
  bool queryJointStatus(std::array<double, 6> & joints);
  void handleJointStatus(const aubo_robot_namespace::JointStatus * joint_status, int size);

  AuboConnectionOptions options_;
  ServiceInterface robot_service_;

  mutable std::mutex mutex_;
  std::array<double, 6> latest_joint_positions_{};
  bool have_joint_status_{false};
  std::chrono::steady_clock::time_point last_joint_status_time_{};
  bool started_up_{false};
  std::atomic<bool> connected_{false};
};
}  // namespace aubo_ros2_control
