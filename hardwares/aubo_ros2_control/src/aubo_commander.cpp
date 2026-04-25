#include "aubo_ros2_control/aubo_commander.h"

#include <algorithm>
#include <cstring>

namespace aubo_ros2_control
{
namespace
{
bool isSuccess(int ret)
{
  return ret == aubo_robot_namespace::InterfaceCallSuccCode;
}
}  // namespace

AuboCommander::AuboCommander(AuboConnectionOptions options)
: options_(std::move(options))
{
}

AuboCommander::~AuboCommander()
{
  disconnect();
}

bool AuboCommander::connect(std::string * error_message)
{
  if (connected_.load(std::memory_order_acquire)) {
    return true;
  }

  const int login_ret = robot_service_.robotServiceLogin(
    options_.ip.c_str(), options_.port, options_.username.c_str(), options_.password.c_str());
  if (!isSuccess(login_ret)) {
    if (error_message) {
      *error_message = "AUBO login failed, ret=" + std::to_string(login_ret);
    }
    return false;
  }

  if (!startupRobot(error_message)) {
    robot_service_.robotServiceLogout();
    return false;
  }

  if (!configureMoveProfile(error_message)) {
    disconnect();
    return false;
  }

  const int callback_ret = robot_service_.robotServiceRegisterRealTimeJointStatusCallback(
    &AuboCommander::jointStatusCallback, this);
  if (!isSuccess(callback_ret)) {
    if (error_message) {
      *error_message = "Failed to register real-time joint callback, ret=" +
        std::to_string(callback_ret);
    }
    disconnect();
    return false;
  }

  const int push_ret = robot_service_.robotServiceSetRealTimeJointStatusPush(true);
  if (!isSuccess(push_ret)) {
    if (error_message) {
      *error_message = "Failed to enable real-time joint push, ret=" + std::to_string(push_ret);
    }
    disconnect();
    return false;
  }

  std::array<double, 6> joints{};
  if (!queryJointStatus(joints)) {
    if (error_message) {
      *error_message = "Failed to query initial joint status from AUBO SDK";
    }
    disconnect();
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_joint_positions_ = joints;
    have_joint_status_ = true;
    last_joint_status_time_ = std::chrono::steady_clock::now();
  }

  connected_.store(true, std::memory_order_release);
  return true;
}

void AuboCommander::disconnect()
{
  const bool was_connected = connected_.exchange(false, std::memory_order_acq_rel);
  if (!was_connected && !started_up_) {
    return;
  }

  robot_service_.robotServiceSetRealTimeJointStatusPush(false);
  robot_service_.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);

  if (started_up_ && options_.shutdown_on_disconnect) {
    robot_service_.robotServiceRobotShutdown();
  }
  started_up_ = false;

  robot_service_.robotServiceLogout();

  std::lock_guard<std::mutex> lock(mutex_);
  have_joint_status_ = false;
}

bool AuboCommander::isConnected() const
{
  return connected_.load(std::memory_order_acquire);
}

bool AuboCommander::readJointPositions(std::array<double, 6> & joints)
{
  constexpr auto kRealtimeJointStatusTimeout = std::chrono::milliseconds(200);
  const auto now = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (
      have_joint_status_ &&
      (last_joint_status_time_.time_since_epoch().count() > 0) &&
      ((now - last_joint_status_time_) <= kRealtimeJointStatusTimeout))
    {
      joints = latest_joint_positions_;
      return true;
    }
  }

  if (queryJointStatus(joints)) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_joint_positions_ = joints;
    have_joint_status_ = true;
    last_joint_status_time_ = std::chrono::steady_clock::now();
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_joint_status_) {
      joints = latest_joint_positions_;
      return true;
    }
  }

  return false;
}

bool AuboCommander::sendJointMove(const std::array<double, 6> & joints, bool blocking)
{
  if (!isConnected()) {
    return false;
  }

  double joint_angle[aubo_robot_namespace::ARM_DOF] = {0.0};
  std::copy(joints.begin(), joints.end(), joint_angle);

  const int ret = robot_service_.robotServiceJointMove(joint_angle, blocking);
  return isSuccess(ret);
}

bool AuboCommander::sendFollowModeJointMove(const std::array<double, 6> & joints)
{
  if (!isConnected()) {
    return false;
  }

  double joint_angle[aubo_robot_namespace::ARM_DOF] = {0.0};
  std::copy(joints.begin(), joints.end(), joint_angle);

  const int ret = robot_service_.robotServiceFollowModeJointMove(joint_angle);
  return isSuccess(ret);
}

void AuboCommander::jointStatusCallback(
  const aubo_robot_namespace::JointStatus * joint_status,
  int size,
  void * arg)
{
  if (arg == nullptr || joint_status == nullptr) {
    return;
  }
  static_cast<AuboCommander *>(arg)->handleJointStatus(joint_status, size);
}

bool AuboCommander::startupRobot(std::string * error_message)
{
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  aubo_robot_namespace::ToolDynamicsParam tool_dynamics_param;
  std::memset(&tool_dynamics_param, 0, sizeof(tool_dynamics_param));

  const int ret = robot_service_.rootServiceRobotStartup(
    tool_dynamics_param,
    static_cast<uint8>(options_.collision_class),
    options_.startup_read_pose,
    true,
    1000,
    result);

  if (!isSuccess(ret)) {
    if (error_message) {
      *error_message = "AUBO startup failed, ret=" + std::to_string(ret);
    }
    return false;
  }

  started_up_ = true;
  return true;
}

bool AuboCommander::configureMoveProfile(std::string * error_message)
{
  const int init_ret = robot_service_.robotServiceInitGlobalMoveProfile();
  if (!isSuccess(init_ret)) {
    if (error_message) {
      *error_message = "Failed to initialize AUBO move profile, ret=" + std::to_string(init_ret);
    }
    return false;
  }

  aubo_robot_namespace::JointVelcAccParam max_acc;
  aubo_robot_namespace::JointVelcAccParam max_vel;
  for (int i = 0; i < aubo_robot_namespace::ARM_DOF; ++i) {
    max_acc.jointPara[i] = options_.max_joint_acc_rad;
    max_vel.jointPara[i] = options_.max_joint_vel_rad;
  }

  const int acc_ret = robot_service_.robotServiceSetGlobalMoveJointMaxAcc(max_acc);
  if (!isSuccess(acc_ret)) {
    if (error_message) {
      *error_message = "Failed to set AUBO max joint acceleration, ret=" +
        std::to_string(acc_ret);
    }
    return false;
  }

  const int vel_ret = robot_service_.robotServiceSetGlobalMoveJointMaxVelc(max_vel);
  if (!isSuccess(vel_ret)) {
    if (error_message) {
      *error_message = "Failed to set AUBO max joint velocity, ret=" + std::to_string(vel_ret);
    }
    return false;
  }

  return true;
}

bool AuboCommander::queryJointStatus(std::array<double, 6> & joints)
{
  aubo_robot_namespace::JointStatus joint_status[aubo_robot_namespace::ARM_DOF];
  const int ret = robot_service_.robotServiceGetRobotJointStatus(
    joint_status, aubo_robot_namespace::ARM_DOF);
  if (!isSuccess(ret)) {
    return false;
  }

  for (int i = 0; i < aubo_robot_namespace::ARM_DOF; ++i) {
    joints[static_cast<size_t>(i)] = joint_status[i].jointPosJ;
  }
  return true;
}

void AuboCommander::handleJointStatus(
  const aubo_robot_namespace::JointStatus * joint_status,
  int size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int count = std::min(size, static_cast<int>(aubo_robot_namespace::ARM_DOF));
  for (int i = 0; i < count; ++i) {
    latest_joint_positions_[static_cast<size_t>(i)] = joint_status[i].jointPosJ;
  }
  have_joint_status_ = true;
  last_joint_status_time_ = std::chrono::steady_clock::now();
}
}  // namespace aubo_ros2_control
