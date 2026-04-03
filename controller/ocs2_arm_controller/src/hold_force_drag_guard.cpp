#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "dobot_force_msgs/srv/enable_ft_sensor.hpp"
#include "dobot_force_msgs/srv/force_drive_mode.hpp"
#include "dobot_force_msgs/srv/force_drive_speed.hpp"
#include "dobot_force_msgs/srv/six_force_home.hpp"
#include "dobot_force_msgs/srv/stop_drag.hpp"

namespace
{
class HoldForceDragGuard : public rclcpp::Node
{
public:
    HoldForceDragGuard()
        : Node("hold_force_drag_guard")
    {
        wrench_topic_ = declare_parameter<std::string>(
            "wrench_topic", "/force_torque_sensor_broadcaster/wrench_filtered");
        force_enter_threshold_ = declare_parameter<double>("force_enter_threshold", 10.0);
        force_exit_threshold_ = declare_parameter<double>("force_exit_threshold", 7.0);
        torque_enter_threshold_ = declare_parameter<double>("torque_enter_threshold", 1.0);
        torque_exit_threshold_ = declare_parameter<double>("torque_exit_threshold", 0.7);
        required_samples_ = declare_parameter<int>("required_samples", 2);
        min_switch_interval_sec_ = declare_parameter<double>("min_switch_interval_sec", 0.4);
        attempt_interval_sec_ = declare_parameter<double>("attempt_interval_sec", 0.8);
        hold_resume_settle_ms_ = declare_parameter<int>("hold_resume_settle_ms", 120);
        service_wait_timeout_ms_ = declare_parameter<int>("service_wait_timeout_ms", 800);
        service_call_timeout_ms_ = declare_parameter<int>("service_call_timeout_ms", 5000);

        service_prefix_ = declare_parameter<std::string>("force_service_prefix", "/dobot_force_control/srv");
        drag_speed_ = declare_parameter<int>("drag_speed", 8);
        drag_x_ = declare_parameter<int>("drag_x", 1);
        drag_y_ = declare_parameter<int>("drag_y", 1);
        drag_z_ = declare_parameter<int>("drag_z", 1);
        drag_rx_ = declare_parameter<int>("drag_rx", 1);
        drag_ry_ = declare_parameter<int>("drag_ry", 1);
        drag_rz_ = declare_parameter<int>("drag_rz", 1);
        drag_user_ = declare_parameter<int>("drag_user", 0);
        auto_prepare_on_demand_ = declare_parameter<bool>("auto_prepare_on_demand", false);
        prepare_settle_ms_ = declare_parameter<int>("prepare_settle_ms", 500);

        current_fsm_state_.store(declare_parameter<int>("initial_fsm_state", 2));

        if (!service_prefix_.empty() && service_prefix_.back() == '/')
        {
            service_prefix_.pop_back();
        }

        sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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
            std::bind(&HoldForceDragGuard::onWrench, this, std::placeholders::_1),
            sub_options);
        fsm_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/fsm_command", 20,
            std::bind(&HoldForceDragGuard::onFsmCommand, this, std::placeholders::_1),
            sub_options);

        param_cb_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& p : params)
                {
                    const auto& name = p.get_name();
                    if (name == "force_enter_threshold") { force_enter_threshold_ = p.as_double(); }
                    else if (name == "force_exit_threshold") { force_exit_threshold_ = p.as_double(); }
                    else if (name == "torque_enter_threshold") { torque_enter_threshold_ = p.as_double(); }
                    else if (name == "torque_exit_threshold") { torque_exit_threshold_ = p.as_double(); }
                    else if (name == "required_samples") { required_samples_ = p.as_int(); }
                    else if (name == "min_switch_interval_sec") { min_switch_interval_sec_ = p.as_double(); }
                    else if (name == "attempt_interval_sec") { attempt_interval_sec_ = p.as_double(); }
                    else if (name == "hold_resume_settle_ms") { hold_resume_settle_ms_ = p.as_int(); }
                    else if (name == "service_wait_timeout_ms") { service_wait_timeout_ms_ = p.as_int(); }
                    else if (name == "service_call_timeout_ms") { service_call_timeout_ms_ = p.as_int(); }
                    else if (name == "prepare_settle_ms") { prepare_settle_ms_ = p.as_int(); }
                }
                return result;
            });

        last_switch_time_ = now();

        RCLCPP_INFO(
            get_logger(),
            "hold_force_drag_guard started. wrench_topic=%s, enter(F/T)=%.3f/%.3f, exit(F/T)=%.3f/%.3f, initial_state=%d",
            wrench_topic_.c_str(),
            force_enter_threshold_,
            torque_enter_threshold_,
            force_exit_threshold_,
            torque_exit_threshold_,
            current_fsm_state_.load());
    }

private:
    enum class DragMode
    {
        IDLE,
        DRAG,
    };

    static double norm3(double x, double y, double z)
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    bool isHoldState() const
    {
        return current_fsm_state_.load() == 2;
    }

    std::chrono::milliseconds serviceWaitTimeout() const
    {
        return std::chrono::milliseconds(service_wait_timeout_ms_ > 0 ? service_wait_timeout_ms_ : 1);
    }

    std::chrono::milliseconds serviceCallTimeout() const
    {
        return std::chrono::milliseconds(service_call_timeout_ms_ > 0 ? service_call_timeout_ms_ : 1);
    }

    template<typename ServiceT>
    bool waitForClient(const typename rclcpp::Client<ServiceT>::SharedPtr& client, const char* name)
    {
        if (!client->wait_for_service(serviceWaitTimeout()))
        {
            RCLCPP_WARN(get_logger(), "%s service unavailable", name);
            return false;
        }
        return true;
    }

    bool callEnableFtSensor()
    {
        if (!waitForClient<dobot_force_msgs::srv::EnableFTSensor>(enable_ft_client_, "EnableFTSensor"))
        {
            return false;
        }

        auto req = std::make_shared<dobot_force_msgs::srv::EnableFTSensor::Request>();
        req->status = 1;
        auto future = enable_ft_client_->async_send_request(req);
        if (future.wait_for(serviceCallTimeout()) != std::future_status::ready)
        {
            RCLCPP_WARN(get_logger(), "EnableFTSensor call timeout");
            return false;
        }

        const auto response = future.get();
        if (!response)
        {
            RCLCPP_WARN(get_logger(), "EnableFTSensor empty response");
            return false;
        }

        if (response->res == -2)
        {
            RCLCPP_INFO(get_logger(), "EnableFTSensor returned -2, treat as already enabled");
            return true;
        }

        if (response->res != 0)
        {
            RCLCPP_WARN(get_logger(), "EnableFTSensor failed, res=%d", response->res);
            return false;
        }

        return true;
    }

    bool callSixForceHome()
    {
        if (!waitForClient<dobot_force_msgs::srv::SixForceHome>(six_force_home_client_, "SixForceHome"))
        {
            return false;
        }

        auto req = std::make_shared<dobot_force_msgs::srv::SixForceHome::Request>();
        auto future = six_force_home_client_->async_send_request(req);
        if (future.wait_for(serviceCallTimeout()) != std::future_status::ready)
        {
            RCLCPP_WARN(get_logger(), "SixForceHome call timeout");
            return false;
        }

        const auto response = future.get();
        if (!response)
        {
            RCLCPP_WARN(get_logger(), "SixForceHome empty response");
            return false;
        }

        if (response->res != 0)
        {
            RCLCPP_WARN(get_logger(), "SixForceHome failed, res=%d", response->res);
            return false;
        }

        return true;
    }

    bool callForceDriveMode(int x, int y, int z, int rx, int ry, int rz, int user)
    {
        if (!waitForClient<dobot_force_msgs::srv::ForceDriveMode>(mode_client_, "ForceDriveMode"))
        {
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
        auto future = mode_client_->async_send_request(req);
        if (future.wait_for(serviceCallTimeout()) != std::future_status::ready)
        {
            RCLCPP_WARN(get_logger(), "ForceDriveMode call timeout");
            return false;
        }

        const auto response = future.get();
        if (!response)
        {
            RCLCPP_WARN(get_logger(), "ForceDriveMode empty response");
            return false;
        }

        if (response->res != 0)
        {
            RCLCPP_WARN(get_logger(), "ForceDriveMode failed, res=%d", response->res);
            return false;
        }

        return true;
    }

    bool callForceDriveSpeed()
    {
        if (!waitForClient<dobot_force_msgs::srv::ForceDriveSpeed>(speed_client_, "ForceDriveSpeed"))
        {
            return false;
        }

        auto req = std::make_shared<dobot_force_msgs::srv::ForceDriveSpeed::Request>();
        req->speed = drag_speed_;
        auto future = speed_client_->async_send_request(req);
        if (future.wait_for(serviceCallTimeout()) != std::future_status::ready)
        {
            RCLCPP_WARN(get_logger(), "ForceDriveSpeed call timeout");
            return false;
        }

        const auto response = future.get();
        if (!response)
        {
            RCLCPP_WARN(get_logger(), "ForceDriveSpeed empty response");
            return false;
        }

        if (response->res != 0)
        {
            RCLCPP_WARN(get_logger(), "ForceDriveSpeed failed, res=%d", response->res);
            return false;
        }

        return true;
    }

    bool callStopDrag()
    {
        if (!waitForClient<dobot_force_msgs::srv::StopDrag>(stop_drag_client_, "StopDrag"))
        {
            return false;
        }

        auto req = std::make_shared<dobot_force_msgs::srv::StopDrag::Request>();
        auto future = stop_drag_client_->async_send_request(req);
        if (future.wait_for(serviceCallTimeout()) != std::future_status::ready)
        {
            RCLCPP_WARN(get_logger(), "StopDrag call timeout");
            return false;
        }

        const auto response = future.get();
        if (!response)
        {
            RCLCPP_WARN(get_logger(), "StopDrag empty response");
            return false;
        }

        if (response->res != 0)
        {
            RCLCPP_WARN(get_logger(), "StopDrag failed, res=%d", response->res);
            return false;
        }

        return true;
    }

    bool prepareForceIfNeeded()
    {
        if (!auto_prepare_on_demand_ || force_prepared_)
        {
            return true;
        }

        if (!callEnableFtSensor())
        {
            return false;
        }

        if (prepare_settle_ms_ > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(prepare_settle_ms_));
        }

        if (!callSixForceHome())
        {
            return false;
        }

        force_prepared_ = true;
        RCLCPP_INFO(get_logger(), "On-demand force prepare succeeded (EnableFTSensor + SixForceHome)");
        return true;
    }

    void enterDrag(double f_norm, double t_norm)
    {
        if (!prepareForceIfNeeded())
        {
            last_switch_time_ = now();
            return;
        }

        bool mode_ok = callForceDriveMode(drag_x_, drag_y_, drag_z_, drag_rx_, drag_ry_, drag_rz_, drag_user_);
        if (!mode_ok && drag_user_ == -1)
        {
            mode_ok = callForceDriveMode(drag_x_, drag_y_, drag_z_, drag_rx_, drag_ry_, drag_rz_, 0);
        }

        const bool speed_ok = mode_ok ? callForceDriveSpeed() : false;
        if (mode_ok && speed_ok)
        {
            drag_mode_ = DragMode::DRAG;
            last_switch_time_ = now();
            RCLCPP_WARN(
                get_logger(),
                "Entered HOLD drag mode (|F|=%.3fN, |T|=%.3fNm). Robot is now in native ForceDrive.",
                f_norm, t_norm);
            return;
        }

        last_switch_time_ = now();
        RCLCPP_WARN(
            get_logger(),
            "Failed to enter HOLD drag mode (mode_ok=%s, speed_ok=%s).",
            mode_ok ? "true" : "false",
            speed_ok ? "true" : "false");
    }

    void exitDrag(double f_norm, double t_norm)
    {
        if (!callStopDrag())
        {
            last_switch_time_ = now();
            RCLCPP_WARN(get_logger(), "Failed to stop HOLD drag mode, will retry.");
            return;
        }

        if (hold_resume_settle_ms_ > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(hold_resume_settle_ms_));
        }

        drag_mode_ = DragMode::IDLE;
        last_switch_time_ = now();
        RCLCPP_INFO(
            get_logger(),
            "Exited HOLD drag mode (|F|=%.3fN, |T|=%.3fNm). Controller returns to locked HOLD target.",
            f_norm, t_norm);
    }

    void stopDragForStateChange(int next_state)
    {
        if (drag_mode_ != DragMode::DRAG)
        {
            return;
        }

        if (!callStopDrag())
        {
            RCLCPP_WARN(get_logger(), "Failed to stop HOLD drag while leaving HOLD state %d.", next_state);
            return;
        }

        drag_mode_ = DragMode::IDLE;
        last_switch_time_ = now();
        RCLCPP_INFO(get_logger(), "Stopped HOLD drag because FSM left HOLD (next_state=%d).", next_state);
    }

    void onFsmCommand(const std_msgs::msg::Int32::SharedPtr msg)
    {
        const int command = msg->data;
        if (command < 1 || command > 4)
        {
            return;
        }

        current_fsm_state_.store(command);
        enter_counter_ = 0;
        exit_counter_ = 0;

        if (command != 2 && !switching_.exchange(true))
        {
            stopDragForStateChange(command);
            switching_.store(false);
        }
    }

    void onWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        if (switching_.load())
        {
            return;
        }

        if (!isHoldState())
        {
            enter_counter_ = 0;
            exit_counter_ = 0;
            return;
        }

        const auto now_t = now();
        if ((now_t - last_switch_time_).seconds() < min_switch_interval_sec_)
        {
            return;
        }

        const double f_norm = norm3(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
        const double t_norm = norm3(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);

        const bool over_enter = (f_norm >= force_enter_threshold_) || (t_norm >= torque_enter_threshold_);
        const bool below_exit = (f_norm <= force_exit_threshold_) && (t_norm <= torque_exit_threshold_);

        if (drag_mode_ == DragMode::IDLE)
        {
            enter_counter_ = over_enter ? (enter_counter_ + 1) : 0;
            exit_counter_ = 0;
            if (enter_counter_ >= required_samples_)
            {
                if ((now_t - last_enter_attempt_time_).seconds() < attempt_interval_sec_)
                {
                    return;
                }

                last_enter_attempt_time_ = now_t;
                enter_counter_ = 0;
                if (!switching_.exchange(true))
                {
                    enterDrag(f_norm, t_norm);
                    switching_.store(false);
                }
            }
            return;
        }

        exit_counter_ = below_exit ? (exit_counter_ + 1) : 0;
        enter_counter_ = 0;
        if (exit_counter_ >= required_samples_)
        {
            if ((now_t - last_exit_attempt_time_).seconds() < attempt_interval_sec_)
            {
                return;
            }

            last_exit_attempt_time_ = now_t;
            exit_counter_ = 0;
            if (!switching_.exchange(true))
            {
                exitDrag(f_norm, t_norm);
                switching_.store(false);
            }
        }
    }

    std::string wrench_topic_;
    std::string service_prefix_;

    double force_enter_threshold_{10.0};
    double force_exit_threshold_{7.0};
    double torque_enter_threshold_{1.0};
    double torque_exit_threshold_{0.7};
    int required_samples_{2};
    double min_switch_interval_sec_{0.4};
    double attempt_interval_sec_{0.8};
    int hold_resume_settle_ms_{120};
    int service_wait_timeout_ms_{800};
    int service_call_timeout_ms_{5000};

    int drag_speed_{8};
    int drag_x_{1};
    int drag_y_{1};
    int drag_z_{1};
    int drag_rx_{1};
    int drag_ry_{1};
    int drag_rz_{1};
    int drag_user_{0};
    bool auto_prepare_on_demand_{false};
    int prepare_settle_ms_{500};
    bool force_prepared_{false};

    DragMode drag_mode_{DragMode::IDLE};
    int enter_counter_{0};
    int exit_counter_{0};
    std::atomic<int> current_fsm_state_{2};
    rclcpp::Time last_switch_time_;
    rclcpp::Time last_enter_attempt_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_exit_attempt_time_{0, 0, RCL_ROS_TIME};
    std::atomic_bool switching_{false};

    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_sub_;

    rclcpp::Client<dobot_force_msgs::srv::ForceDriveSpeed>::SharedPtr speed_client_;
    rclcpp::Client<dobot_force_msgs::srv::ForceDriveMode>::SharedPtr mode_client_;
    rclcpp::Client<dobot_force_msgs::srv::StopDrag>::SharedPtr stop_drag_client_;
    rclcpp::Client<dobot_force_msgs::srv::EnableFTSensor>::SharedPtr enable_ft_client_;
    rclcpp::Client<dobot_force_msgs::srv::SixForceHome>::SharedPtr six_force_home_client_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};
} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HoldForceDragGuard>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
