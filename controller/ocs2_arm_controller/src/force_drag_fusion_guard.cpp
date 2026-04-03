#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "dobot_force_msgs/srv/force_drive_mode.hpp"
#include "dobot_force_msgs/srv/force_drive_speed.hpp"
#include "dobot_force_msgs/srv/stop_drag.hpp"
#include "dobot_force_msgs/srv/enable_ft_sensor.hpp"
#include "dobot_force_msgs/srv/six_force_home.hpp"

using namespace std::chrono_literals;

namespace {

class ForceDragFusionGuard : public rclcpp::Node {
public:
  ForceDragFusionGuard() : Node("force_drag_fusion_guard") {
    wrench_topic_ = declare_parameter<std::string>("wrench_topic", "/force_torque_sensor_broadcaster/wrench_filtered");
    // Default behavior: single-threshold immediate switch.
    force_enter_threshold_ = declare_parameter<double>("force_enter_threshold", 25.0);
    force_exit_threshold_ = declare_parameter<double>("force_exit_threshold", 25.0);
    torque_enter_threshold_ = declare_parameter<double>("torque_enter_threshold", 3.0);
    torque_exit_threshold_ = declare_parameter<double>("torque_exit_threshold", 3.0);
    required_samples_ = declare_parameter<int>("required_samples", 1);
    min_switch_interval_sec_ = declare_parameter<double>("min_switch_interval_sec", 0.0);
    attempt_interval_sec_ = declare_parameter<double>("attempt_interval_sec", 0.8);
    hold_settle_ms_ = declare_parameter<int>("hold_settle_ms", 80);
    ocs2_resume_settle_ms_ = declare_parameter<int>("ocs2_resume_settle_ms", 120);
    service_wait_timeout_ms_ = declare_parameter<int>("service_wait_timeout_ms", 800);
    service_call_timeout_ms_ = declare_parameter<int>("service_call_timeout_ms", 5000);

    service_prefix_ = declare_parameter<std::string>("force_service_prefix", "/dobot_force_control/srv");
    drag_speed_ = declare_parameter<int>("drag_speed", 20);
    drag_x_ = declare_parameter<int>("drag_x", 1);
    drag_y_ = declare_parameter<int>("drag_y", 1);
    drag_z_ = declare_parameter<int>("drag_z", 1);
    drag_rx_ = declare_parameter<int>("drag_rx", 1);
    drag_ry_ = declare_parameter<int>("drag_ry", 1);
    drag_rz_ = declare_parameter<int>("drag_rz", 1);
    // Some firmware variants reject omitted user; default to base/user frame index 0.
    drag_user_ = declare_parameter<int>("drag_user", 0);
    auto_prepare_on_demand_ = declare_parameter<bool>("auto_prepare_on_demand", false);
    prepare_settle_ms_ = declare_parameter<int>("prepare_settle_ms", 500);

    if (!service_prefix_.empty() && service_prefix_.back() == '/') {
      service_prefix_.pop_back();
    }

    // Use separate callback groups to avoid service-response starvation while in subscription callbacks.
    sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    fsm_pub_ = create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);

    speed_client_ = create_client<dobot_force_msgs::srv::ForceDriveSpeed>(
        service_prefix_ + "/ForceDriveSpeed", rmw_qos_profile_services_default, client_cb_group_);
    mode_client_ = create_client<dobot_force_msgs::srv::ForceDriveMode>(
        service_prefix_ + "/ForceDriveMode", rmw_qos_profile_services_default, client_cb_group_);
    stop_drag_client_ = create_client<dobot_force_msgs::srv::StopDrag>(
        service_prefix_ + "/StopDrag", rmw_qos_profile_services_default, client_cb_group_);
    enable_ft_client_ = create_client<dobot_force_msgs::srv::EnableFTSensor>(
        service_prefix_ + "/EnableFTSensor", rmw_qos_profile_services_default, client_cb_group_);
    six_force_home_client_ = create_client<dobot_force_msgs::srv::SixForceHome>(
        service_prefix_ + "/SixForceHome", rmw_qos_profile_services_default, client_cb_group_);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_cb_group_;

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        wrench_topic_, 20,
        std::bind(&ForceDragFusionGuard::onWrench, this, std::placeholders::_1),
        sub_options);

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          bool updated = false;
          for (const auto& p : params)
          {
            const auto& n = p.get_name();
            if (n == "force_enter_threshold") { force_enter_threshold_ = p.as_double(); updated = true; }
            else if (n == "force_exit_threshold") { force_exit_threshold_ = p.as_double(); updated = true; }
            else if (n == "torque_enter_threshold") { torque_enter_threshold_ = p.as_double(); updated = true; }
            else if (n == "torque_exit_threshold") { torque_exit_threshold_ = p.as_double(); updated = true; }
            else if (n == "required_samples") { required_samples_ = p.as_int(); updated = true; }
            else if (n == "min_switch_interval_sec") { min_switch_interval_sec_ = p.as_double(); updated = true; }
            else if (n == "attempt_interval_sec") { attempt_interval_sec_ = p.as_double(); updated = true; }
            else if (n == "hold_settle_ms") { hold_settle_ms_ = p.as_int(); updated = true; }
            else if (n == "ocs2_resume_settle_ms") { ocs2_resume_settle_ms_ = p.as_int(); updated = true; }
            else if (n == "prepare_settle_ms") { prepare_settle_ms_ = p.as_int(); updated = true; }
            else if (n == "service_wait_timeout_ms") { service_wait_timeout_ms_ = p.as_int(); updated = true; }
            else if (n == "service_call_timeout_ms") { service_call_timeout_ms_ = p.as_int(); updated = true; }
          }
          if (updated)
          {
            RCLCPP_INFO(
                get_logger(),
                "Updated params: enter(F/T)=%.3f/%.3f, exit(F/T)=%.3f/%.3f, samples=%d, min_interval=%.3fs, attempt_interval=%.3fs, hold_settle=%dms, prepare_settle=%dms, wait_timeout=%dms, call_timeout=%dms",
                force_enter_threshold_, torque_enter_threshold_,
                force_exit_threshold_, torque_exit_threshold_,
                required_samples_, min_switch_interval_sec_, attempt_interval_sec_, hold_settle_ms_, prepare_settle_ms_,
                service_wait_timeout_ms_, service_call_timeout_ms_);
          }
          return result;
        });

    last_switch_time_ = now();

    RCLCPP_INFO(get_logger(),
                "force_drag_fusion_guard started. wrench_topic=%s, enter(F/T)=%.3f/%.3f, exit(F/T)=%.3f/%.3f",
                wrench_topic_.c_str(), force_enter_threshold_, torque_enter_threshold_,
                force_exit_threshold_, torque_exit_threshold_);
  }

private:
  enum class Mode { OCS2, DRAG };

  static double norm3(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
  }

  std::chrono::milliseconds service_wait_timeout() const {
    return std::chrono::milliseconds(service_wait_timeout_ms_ > 0 ? service_wait_timeout_ms_ : 1);
  }

  std::chrono::milliseconds service_call_timeout() const {
    return std::chrono::milliseconds(service_call_timeout_ms_ > 0 ? service_call_timeout_ms_ : 1);
  }

  bool call_force_drive_speed() {
    if (!speed_client_->wait_for_service(service_wait_timeout())) {
      RCLCPP_WARN(get_logger(), "ForceDriveSpeed service unavailable");
      return false;
    }
    auto req = std::make_shared<dobot_force_msgs::srv::ForceDriveSpeed::Request>();
    req->speed = drag_speed_;
    auto fut = speed_client_->async_send_request(req);
    if (fut.wait_for(service_call_timeout()) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "ForceDriveSpeed call timeout");
      return false;
    }
    const auto res = fut.get();
    if (!res) {
      RCLCPP_WARN(get_logger(), "ForceDriveSpeed empty response");
      return false;
    }
    if (res->res != 0) {
      RCLCPP_WARN(get_logger(), "ForceDriveSpeed failed, res=%d", res->res);
      return false;
    }
    return true;
  }

  bool call_force_drive_mode(int x, int y, int z, int rx, int ry, int rz, int user) {
    if (!mode_client_->wait_for_service(service_wait_timeout())) {
      RCLCPP_WARN(get_logger(), "ForceDriveMode service unavailable");
      return false;
    }
    auto req = std::make_shared<dobot_force_msgs::srv::ForceDriveMode::Request>();
    req->x = x;
    req->y = y;
    req->z = z;
    req->rx = rx;
    req->ry = ry;
    req->rz = rz;
    req->user = user;
    auto fut = mode_client_->async_send_request(req);
    if (fut.wait_for(service_call_timeout()) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "ForceDriveMode call timeout");
      return false;
    }
    const auto res = fut.get();
    if (!res) {
      RCLCPP_WARN(get_logger(), "ForceDriveMode empty response");
      return false;
    }
    if (res->res != 0) {
      RCLCPP_WARN(get_logger(), "ForceDriveMode failed, res=%d", res->res);
      return false;
    }
    return true;
  }

  bool call_stop_drag() {
    if (!stop_drag_client_->wait_for_service(service_wait_timeout())) {
      RCLCPP_WARN(get_logger(), "StopDrag service unavailable");
      return false;
    }
    auto req = std::make_shared<dobot_force_msgs::srv::StopDrag::Request>();
    auto fut = stop_drag_client_->async_send_request(req);
    if (fut.wait_for(service_call_timeout()) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "StopDrag call timeout");
      return false;
    }
    const auto res = fut.get();
    if (!res) {
      RCLCPP_WARN(get_logger(), "StopDrag empty response");
      return false;
    }
    if (res->res != 0) {
      RCLCPP_WARN(get_logger(), "StopDrag failed, res=%d", res->res);
      return false;
    }
    return true;
  }

  bool call_enable_ft_sensor() {
    if (!enable_ft_client_->wait_for_service(service_wait_timeout())) {
      RCLCPP_WARN(get_logger(), "EnableFTSensor service unavailable");
      return false;
    }
    auto req = std::make_shared<dobot_force_msgs::srv::EnableFTSensor::Request>();
    req->status = 1;
    auto fut = enable_ft_client_->async_send_request(req);
    if (fut.wait_for(service_call_timeout()) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "EnableFTSensor call timeout");
      return false;
    }
    const auto res = fut.get();
    if (!res) {
      RCLCPP_WARN(get_logger(), "EnableFTSensor empty response");
      return false;
    }
    if (res->res == -2) {
      // On some firmware -2 means already enabled / no state change needed.
      RCLCPP_INFO(get_logger(), "EnableFTSensor returned -2, treat as already enabled");
      return true;
    }
    if (res->res != 0) {
      RCLCPP_WARN(get_logger(), "EnableFTSensor failed, res=%d", res->res);
      return false;
    }
    return true;
  }

  bool call_six_force_home() {
    if (!six_force_home_client_->wait_for_service(service_wait_timeout())) {
      RCLCPP_WARN(get_logger(), "SixForceHome service unavailable");
      return false;
    }
    auto req = std::make_shared<dobot_force_msgs::srv::SixForceHome::Request>();
    auto fut = six_force_home_client_->async_send_request(req);
    if (fut.wait_for(service_call_timeout()) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "SixForceHome call timeout");
      return false;
    }
    const auto res = fut.get();
    if (!res) {
      RCLCPP_WARN(get_logger(), "SixForceHome empty response");
      return false;
    }
    if (res->res != 0) {
      RCLCPP_WARN(get_logger(), "SixForceHome failed, res=%d", res->res);
      return false;
    }
    return true;
  }

  bool prepare_force_if_needed() {
    if (!auto_prepare_on_demand_ || force_prepared_) {
      return true;
    }

    const bool ft_ok = call_enable_ft_sensor();
    if (!ft_ok) {
      return false;
    }

    if (prepare_settle_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(prepare_settle_ms_));
    }
    const bool home_ok = call_six_force_home();
    if (!home_ok) {
      return false;
    }

    force_prepared_ = true;
    RCLCPP_INFO(get_logger(), "On-demand force prepare succeeded (EnableFTSensor + SixForceHome)");
    return true;
  }

  void publish_fsm(int cmd) {
    std_msgs::msg::Int32 msg;
    msg.data = cmd;
    fsm_pub_->publish(msg);
  }

  void switch_to_drag(double f_norm, double t_norm) {
    if (!prepare_force_if_needed()) {
      RCLCPP_WARN(get_logger(), "Skip drag entry: on-demand force prepare failed");
      last_switch_time_ = now();
      return;
    }

    publish_fsm(2);  // HOLD first, avoid conflicting with OCS2 writes
    if (hold_settle_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(hold_settle_ms_));
    }
    // Prefer entering mode first, then set speed. Some firmwares reject speed updates before mode.
    bool mode_ok = call_force_drive_mode(drag_x_, drag_y_, drag_z_, drag_rx_, drag_ry_, drag_rz_, drag_user_);
    // Fallback for firmware that requires explicit user index even when omitted.
    if (!mode_ok && drag_user_ == -1) {
      mode_ok = call_force_drive_mode(drag_x_, drag_y_, drag_z_, drag_rx_, drag_ry_, drag_rz_, 0);
    }
    const bool speed_ok = mode_ok ? call_force_drive_speed() : false;
    if (speed_ok && mode_ok) {
      mode_ = Mode::DRAG;
      last_switch_time_ = now();
      RCLCPP_WARN(get_logger(),
                  "Force threshold exceeded (|F|=%.3fN, |T|=%.3fNm). Entered DRAG protection mode.",
                  f_norm, t_norm);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Force threshold exceeded but failed to enter DRAG (speed_ok=%s, mode_ok=%s)",
                  speed_ok ? "true" : "false", mode_ok ? "true" : "false");
      // Avoid getting stuck in HOLD when drag service calls fail.
      publish_fsm(3);
      // Entry failed: slow down retry to avoid HOLD<->OCS2 thrashing on unstable dashboard.
      last_enter_attempt_time_ = now();
      last_switch_time_ = now();
    }
  }

  void switch_to_ocs2(double f_norm, double t_norm) {
    const bool stop_ok = call_stop_drag();
    if (stop_ok) {
      if (ocs2_resume_settle_ms_ > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ocs2_resume_settle_ms_));
      }
      publish_fsm(3);  // back to OCS2
      mode_ = Mode::OCS2;
      last_switch_time_ = now();
      RCLCPP_INFO(get_logger(),
                  "Force dropped below exit threshold (|F|=%.3fN, |T|=%.3fNm). Returned to OCS2.",
                  f_norm, t_norm);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Failed to stop drag, keep DRAG mode and retry later.");
      mode_ = Mode::DRAG;
      last_switch_time_ = now();
    }
  }

  void onWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    if (switching_.load()) {
      return;
    }

    const auto now_t = now();
    if ((now_t - last_switch_time_).seconds() < min_switch_interval_sec_) {
      return;
    }

    const double f_norm = norm3(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    const double t_norm = norm3(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);

    const bool over_enter = (f_norm >= force_enter_threshold_) || (t_norm >= torque_enter_threshold_);
    const bool below_exit = (f_norm <= force_exit_threshold_) && (t_norm <= torque_exit_threshold_);

    if (mode_ == Mode::OCS2) {
      enter_counter_ = over_enter ? (enter_counter_ + 1) : 0;
      if (enter_counter_ >= required_samples_) {
        if ((now_t - last_enter_attempt_time_).seconds() < attempt_interval_sec_) {
          return;
        }
        last_enter_attempt_time_ = now_t;
        enter_counter_ = 0;
        if (!switching_.exchange(true)) {
          switch_to_drag(f_norm, t_norm);
          switching_.store(false);
        }
      }
      exit_counter_ = 0;
    } else {
      exit_counter_ = below_exit ? (exit_counter_ + 1) : 0;
      if (exit_counter_ >= required_samples_) {
        if ((now_t - last_exit_attempt_time_).seconds() < attempt_interval_sec_) {
          return;
        }
        last_exit_attempt_time_ = now_t;
        exit_counter_ = 0;
        if (!switching_.exchange(true)) {
          switch_to_ocs2(f_norm, t_norm);
          switching_.store(false);
        }
      }
      enter_counter_ = 0;
    }
  }

  std::string wrench_topic_;
  std::string service_prefix_;

  double force_enter_threshold_{15.0};
  double force_exit_threshold_{15.0};
  double torque_enter_threshold_{3.0};
  double torque_exit_threshold_{3.0};
  int required_samples_{1};
  double min_switch_interval_sec_{0.0};
  double attempt_interval_sec_{0.8};
  int hold_settle_ms_{80};
  int ocs2_resume_settle_ms_{120};
  int service_wait_timeout_ms_{800};
  int service_call_timeout_ms_{5000};

  int drag_speed_{20};
  int drag_x_{1};
  int drag_y_{1};
  int drag_z_{1};
  int drag_rx_{1};
  int drag_ry_{1};
  int drag_rz_{1};
  int drag_user_{-1};
  bool auto_prepare_on_demand_{true};
  int prepare_settle_ms_{500};
  bool force_prepared_{false};

  Mode mode_{Mode::OCS2};
  int enter_counter_{0};
  int exit_counter_{0};
  rclcpp::Time last_switch_time_;
  rclcpp::Time last_enter_attempt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_exit_attempt_time_{0, 0, RCL_ROS_TIME};
  std::atomic_bool switching_{false};

  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

  rclcpp::Client<dobot_force_msgs::srv::ForceDriveSpeed>::SharedPtr speed_client_;
  rclcpp::Client<dobot_force_msgs::srv::ForceDriveMode>::SharedPtr mode_client_;
  rclcpp::Client<dobot_force_msgs::srv::StopDrag>::SharedPtr stop_drag_client_;
  rclcpp::Client<dobot_force_msgs::srv::EnableFTSensor>::SharedPtr enable_ft_client_;
  rclcpp::Client<dobot_force_msgs::srv::SixForceHome>::SharedPtr six_force_home_client_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForceDragFusionGuard>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
