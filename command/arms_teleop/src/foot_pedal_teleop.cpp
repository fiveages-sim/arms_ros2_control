// USB keyboard foot-pedal teleop. Reads one Linux evdev device and controls the WBC FSM.

#include <fcntl.h>
#include <glob.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace {

constexpr std::int32_t kFsmHome = 1;
constexpr std::int32_t kFsmHold = 2;
constexpr std::int32_t kFsmOcs2 = 3;
constexpr std::int32_t kFsmMoveJ = 4;
constexpr std::int32_t kHomePoseSwitch = 100;
constexpr std::int32_t kXrStart = 50;
constexpr std::int32_t kXrEnd = 51;
constexpr std::int32_t kXrManualIntervention = 52;
constexpr std::int32_t kXrDelete = 53;

const char * fsmName(const std::int32_t state)
{
    switch (state) {
    case kFsmHome:
        return "HOME";
    case kFsmHold:
        return "HOLD";
    case kFsmOcs2:
        return "OCS2";
    case kFsmMoveJ:
        return "MOVEJ";
    case 0:
        return "UNKNOWN";
    default:
        return "INVALID";
    }
}

}  // namespace

class FootPedalTeleop final : public rclcpp::Node {
public:
    FootPedalTeleop()
        : Node("foot_pedal_teleop")
    {
        device_path_ = declare_parameter<std::string>("device", "auto");
        device_serial_ = declare_parameter<std::string>("device_serial", "");
        grab_device_ = declare_parameter<bool>("grab_device", true);
        hold_on_disconnect_ = declare_parameter<bool>("hold_on_disconnect", true);
        double_click_ms_ = static_cast<int>(declare_parameter<std::int64_t>(
            "gesture.double_click_ms", 300));
        long_press_ms_ = static_cast<int>(
            declare_parameter<std::int64_t>("gesture.long_press_ms", 700));
        xr_publish_rate_hz_ = declare_parameter<double>("xr.publish_rate_hz", 30.0);
        mode_command_topic_ = declare_parameter<std::string>("mode_command_topic", "mode_command");
        click_profile_ = declare_parameter<std::string>("click_profile", "dexcap");
        cartesian_calibrate_service_ = declare_parameter<std::string>(
            "cartesian_teleop.calibrate_service", "/dexcap_cartesian_teleop/calibrate");
        cartesian_enable_service_ = declare_parameter<std::string>(
            "cartesian_teleop.enable_service", "/dexcap_cartesian_teleop/enable");
        key_hold_ = static_cast<int>(declare_parameter<std::int64_t>("keys.hold", KEY_F13));
        key_home_ = static_cast<int>(declare_parameter<std::int64_t>("keys.home", KEY_F14));
        key_ocs2_ = static_cast<int>(declare_parameter<std::int64_t>("keys.ocs2", KEY_F15));
        key_movej_ = static_cast<int>(declare_parameter<std::int64_t>("keys.movej", KEY_F16));

        validateParameters();
        device_path_ = resolveDevicePath();

        fsm_command_publisher_ =
            create_publisher<std_msgs::msg::Int32>("/fsm_command", rclcpp::QoS(10).reliable());
        mode_command_publisher_ =
            create_publisher<std_msgs::msg::String>(mode_command_topic_, rclcpp::QoS(10).reliable());
        xr_controller_state_publisher_ = create_publisher<std_msgs::msg::Int32>(
            "/xr/controller_state", rclcpp::QoS(10).reliable());
        cartesian_calibrate_client_ = create_client<std_srvs::srv::Trigger>(
            cartesian_calibrate_service_);
        cartesian_enable_client_ = create_client<std_srvs::srv::SetBool>(
            cartesian_enable_service_);

        rclcpp::QoS state_qos(1);
        state_qos.reliable().transient_local();
        fsm_state_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/fsm_state", state_qos,
            [this](const std_msgs::msg::Int32::SharedPtr message) {
                receiveFsmState(message->data);
            });

        rclcpp::QoS wbc_state_qos(1);
        wbc_state_qos.reliable().transient_local();
        wbc_state_subscription_ =
            create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
                "/ocs2_wbc_controller/current_state", wbc_state_qos,
                [this](const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr message) {
                    receiveWbcState(*message);
                });

        openDevice();

        RCLCPP_INFO(get_logger(),
                    "FSM mapping: F13=immediate HOLD; long F14/F15/F16=HOME/OCS2/MOVEJ");
        if (click_profile_ == "dexcap") {
            RCLCPP_INFO(
                get_logger(),
                "DexCap gestures: F14 single/double=calibrate; F16 single=disable, "
                "double=enable; long F14/F15/F16=HOME/OCS2/MOVEJ (%d/%d ms)",
                double_click_ms_, long_press_ms_);
        } else {
            RCLCPP_INFO(
                get_logger(),
                "XR gestures: F14 single/double=XR 50/51, F16 single/double=XR 52/53; "
                "long F14/F15/F16=HOME/OCS2/MOVEJ (%d/%d ms)",
                double_click_ms_, long_press_ms_);
        }
        RCLCPP_INFO(get_logger(),
                    "In OCS2 only: F15 single/double toggles left/right arm");
        RCLCPP_INFO(get_logger(), "Waiting for /fsm_state; HOLD remains available at all times");

        input_timer_ = create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&FootPedalTeleop::readEvents, this));
        const auto xr_period = std::chrono::duration<double>(1.0 / xr_publish_rate_hz_);
        xr_publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(xr_period),
            std::bind(&FootPedalTeleop::publishXrControllerState, this));
    }

    ~FootPedalTeleop() override
    {
        closeDevice();
    }

    FootPedalTeleop(const FootPedalTeleop &) = delete;
    FootPedalTeleop & operator=(const FootPedalTeleop &) = delete;

private:
    void validateParameters() const
    {
        if (device_path_.empty()) {
            throw std::runtime_error("parameter 'device' must be 'auto' or an evdev device path");
        }

        const std::array<int, 4> keys{key_hold_, key_home_, key_ocs2_, key_movej_};
        for (const int key : keys) {
            if (key < 0 || key > KEY_MAX) {
                throw std::runtime_error("configured pedal key code is outside the Linux input range");
            }
        }

        auto sorted_keys = keys;
        std::sort(sorted_keys.begin(), sorted_keys.end());
        if (std::adjacent_find(sorted_keys.begin(), sorted_keys.end()) != sorted_keys.end()) {
            throw std::runtime_error("keys.hold/home/ocs2/movej must use four distinct key codes");
        }
        if (double_click_ms_ < 150 || double_click_ms_ > 1000) {
            throw std::runtime_error(
                "gesture.double_click_ms must be between 150 and 1000 ms");
        }
        if (long_press_ms_ < 400 || long_press_ms_ > 2000) {
            throw std::runtime_error(
                "gesture.long_press_ms must be between 400 and 2000 ms");
        }
        if (xr_publish_rate_hz_ < 1.0 || xr_publish_rate_hz_ > 200.0) {
            throw std::runtime_error("xr.publish_rate_hz must be between 1 and 200 Hz");
        }
        if (mode_command_topic_.empty()) {
            throw std::runtime_error("mode_command_topic must not be empty");
        }
        if (click_profile_ != "dexcap" && click_profile_ != "xr") {
            throw std::runtime_error("click_profile must be 'dexcap' or 'xr'");
        }
        if (cartesian_calibrate_service_.empty() || cartesian_enable_service_.empty()) {
            throw std::runtime_error("cartesian teleop service names must not be empty");
        }
    }

    std::string resolveDevicePath() const
    {
        if (device_path_ != "auto") {
            return device_path_;
        }

        constexpr const char * pattern =
            "/dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd";
        constexpr const char * name_prefix = "usb-SayoDevice_CM6K_";
        constexpr const char * name_suffix = "-event-kbd";

        glob_t matches{};
        const int result = glob(pattern, 0, nullptr, &matches);
        if (result != 0 && result != GLOB_NOMATCH) {
            globfree(&matches);
            throw std::runtime_error("failed to enumerate SayoDevice CM6K input devices");
        }

        struct Candidate {
            std::string path;
            std::string serial;
        };
        std::vector<Candidate> candidates;
        for (std::size_t index = 0; index < matches.gl_pathc; ++index) {
            std::string path = matches.gl_pathv[index];
            const auto slash = path.find_last_of('/');
            const std::string name = slash == std::string::npos ? path : path.substr(slash + 1);

            // The auxiliary interface also presents keyboard events; only interface 00 lacks if01.
            if (name.find("-if01-") != std::string::npos ||
                name.rfind(name_prefix, 0) != 0 ||
                name.size() <= std::strlen(name_prefix) + std::strlen(name_suffix) ||
                name.compare(name.size() - std::strlen(name_suffix), std::strlen(name_suffix),
                             name_suffix) != 0) {
                continue;
            }

            const std::size_t serial_begin = std::strlen(name_prefix);
            const std::size_t serial_length =
                name.size() - serial_begin - std::strlen(name_suffix);
            std::string serial = name.substr(serial_begin, serial_length);
            if (device_serial_.empty() || serial == device_serial_) {
                candidates.push_back({std::move(path), std::move(serial)});
            }
        }
        globfree(&matches);

        if (candidates.size() == 1) {
            RCLCPP_INFO(get_logger(), "Auto-selected SayoDevice CM6K, serial=%s",
                        candidates.front().serial.c_str());
            return candidates.front().path;
        }

        std::ostringstream error;
        if (candidates.empty()) {
            if (device_serial_.empty()) {
                error << "no SayoDevice CM6K primary keyboard interface found";
            } else {
                error << "no SayoDevice CM6K found with serial '" << device_serial_ << "'";
            }
        } else {
            error << "multiple SayoDevice CM6K pedals found; select one with "
                  << "-p device_serial:=<serial>. Candidates:";
            for (const auto & candidate : candidates) {
                error << "\n  " << candidate.serial << " -> " << candidate.path;
            }
        }
        throw std::runtime_error(error.str());
    }

    void openDevice()
    {
        device_fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (device_fd_ < 0) {
            throw std::runtime_error(
                "cannot open '" + device_path_ + "': " + std::strerror(errno) +
                ". Check the path and /dev/input permissions");
        }

        char device_name[256]{};
        if (ioctl(device_fd_, EVIOCGNAME(sizeof(device_name)), device_name) < 0) {
            std::strncpy(device_name, "unknown", sizeof(device_name) - 1);
        }

        if (grab_device_) {
            if (ioctl(device_fd_, EVIOCGRAB, 1) < 0) {
                const std::string error = std::strerror(errno);
                closeDevice();
                throw std::runtime_error(
                    "cannot exclusively grab '" + device_path_ + "': " + error +
                    ". Another process may own it, or input permissions are missing");
            }
            grabbed_ = true;
        }

        RCLCPP_INFO(get_logger(), "Opened foot pedal: %s (%s)", device_name,
                    device_path_.c_str());
        RCLCPP_INFO(get_logger(), "Exclusive device grab: %s", grabbed_ ? "ON" : "OFF");
    }

    void receiveFsmState(const std::int32_t state)
    {
        const bool changed = !fsm_state_received_ || current_fsm_state_ != state;
        current_fsm_state_ = state;
        fsm_state_received_ = true;

        if (changed) {
            RCLCPP_INFO(get_logger(), "FSM state: %s (%d)", fsmName(state), state);
        }
    }

    void receiveWbcState(const arms_ros2_control_msgs::msg::WbcCurrentState & state)
    {
        const bool left_enabled =
            state.left_arm_state == arms_ros2_control_msgs::msg::WbcCurrentState::ARM_ENABLED;
        const bool right_enabled =
            state.right_arm_state == arms_ros2_control_msgs::msg::WbcCurrentState::ARM_ENABLED;
        const bool bimanual_coupled =
            state.bimanual_state ==
            arms_ros2_control_msgs::msg::WbcCurrentState::BIMANUAL_COUPLED;

        const bool changed = !wbc_state_received_ || left_arm_enabled_ != left_enabled ||
                             right_arm_enabled_ != right_enabled ||
                             bimanual_coupled_ != bimanual_coupled;
        left_arm_enabled_ = left_enabled;
        right_arm_enabled_ = right_enabled;
        bimanual_coupled_ = bimanual_coupled;
        wbc_state_received_ = true;

        if (changed) {
            RCLCPP_INFO(get_logger(), "WBC arms: left=%s right=%s coupling=%s",
                        left_arm_enabled_ ? "ON" : "OFF", right_arm_enabled_ ? "ON" : "OFF",
                        bimanual_coupled_ ? "COUPLED" : "INDEPENDENT");
        }
    }

    std::int32_t commandForKey(const std::uint16_t key) const
    {
        if (key == key_hold_) {
            return kFsmHold;
        }
        if (key == key_home_) {
            return kFsmHome;
        }
        if (key == key_ocs2_) {
            return kFsmOcs2;
        }
        if (key == key_movej_) {
            return kFsmMoveJ;
        }
        return 0;
    }

    void handleKeyEvent(const input_event & event)
    {
        std::int32_t command = commandForKey(event.code);
        if (command == 0 || event.value == 2) {
            return;
        }

        if (event.value == 0) {
            if (active_key_ == event.code) {
                const int completed_gesture_key =
                    active_gesture_key_ == event.code ? active_gesture_key_ : -1;
                if (long_press_timer_) {
                    long_press_timer_->cancel();
                }
                if (active_fsm_command_) {
                    publishCommand(0);
                }
                active_key_ = -1;
                active_fsm_command_ = false;
                active_gesture_key_ = -1;
                if (completed_gesture_key >= 0 && !long_press_triggered_) {
                    finishGestureClick(completed_gesture_key);
                }
                long_press_triggered_ = false;
            }
            return;
        }

        if (event.value != 1) {
            return;
        }

        // HOLD is the safety command and may preempt any other pedal command.
        if (command == kFsmHold) {
            cancelAllClickSequences();
            active_key_ = event.code;
            active_fsm_command_ = true;
            active_gesture_key_ = -1;
            if (click_profile_ == "dexcap") {
                requestCartesianEnable(false, "F13 HOLD");
            }
            publishCommand(command);
            RCLCPP_WARN(get_logger(), "Pedal F13 requested HOLD");
            return;
        }

        if (active_key_ == key_hold_) {
            RCLCPP_WARN(get_logger(), "Ignoring %s while the HOLD pedal is pressed", fsmName(command));
            return;
        }
        if (active_key_ >= 0) {
            RCLCPP_WARN(get_logger(), "Ignoring %s while another pedal is pressed", fsmName(command));
            return;
        }
        if (isGestureKey(event.code)) {
            startGesturePress(event.code);
            return;
        }
    }

    void publishCommand(const std::int32_t command)
    {
        std_msgs::msg::Int32 message;
        message.data = command;
        fsm_command_publisher_->publish(message);
    }

    struct ClickState {
        bool waiting_second_click{false};
        bool double_click_in_progress{false};
        rclcpp::TimerBase::SharedPtr single_click_timer;
    };

    bool isGestureKey(const int key) const
    {
        return key == key_home_ || key == key_ocs2_ || key == key_movej_;
    }

    ClickState & clickStateForKey(const int key)
    {
        if (key == key_home_) {
            return f14_click_state_;
        }
        if (key == key_ocs2_) {
            return f15_click_state_;
        }
        return f16_click_state_;
    }

    void cancelClickState(ClickState & state)
    {
        if (state.single_click_timer) {
            state.single_click_timer->cancel();
        }
        state.waiting_second_click = false;
        state.double_click_in_progress = false;
    }

    void cancelOtherClickSequences(const int key)
    {
        if (key != key_home_) {
            cancelClickState(f14_click_state_);
        }
        if (key != key_ocs2_) {
            cancelClickState(f15_click_state_);
        }
        if (key != key_movej_) {
            cancelClickState(f16_click_state_);
        }
    }

    void startGesturePress(const int key)
    {
        cancelOtherClickSequences(key);
        auto & state = clickStateForKey(key);
        active_key_ = key;
        active_fsm_command_ = false;
        active_gesture_key_ = key;
        long_press_triggered_ = false;
        if (state.waiting_second_click) {
            if (state.single_click_timer) {
                state.single_click_timer->cancel();
            }
            state.waiting_second_click = false;
            state.double_click_in_progress = true;
        }

        long_press_timer_ = create_wall_timer(
            std::chrono::milliseconds(long_press_ms_), [this, key]() {
                long_press_timer_->cancel();
                if (active_key_ != key || active_gesture_key_ != key) {
                    return;
                }
                long_press_triggered_ = true;
                cancelAllClickSequences();
                executeLongPressFsm(key);
            });
    }

    void finishGestureClick(const int key)
    {
        auto & state = clickStateForKey(key);
        if (state.double_click_in_progress) {
            state.double_click_in_progress = false;
            executeClickGesture(key, true);
            return;
        }

        state.waiting_second_click = true;
        state.single_click_timer = create_wall_timer(
            std::chrono::milliseconds(double_click_ms_), [this, key]() {
                auto & click_state = clickStateForKey(key);
                click_state.single_click_timer->cancel();
                if (!click_state.waiting_second_click) {
                    return;
                }
                click_state.waiting_second_click = false;
                executeClickGesture(key, false);
            });
    }

    void cancelAllClickSequences()
    {
        cancelClickState(f14_click_state_);
        cancelClickState(f15_click_state_);
        cancelClickState(f16_click_state_);
        active_gesture_key_ = -1;
    }

    void executeClickGesture(const int key, const bool double_click)
    {
        if (click_profile_ == "dexcap") {
            if (key == key_home_) {
                requestCartesianCalibration(double_click ? "F14 double-click" : "F14 click");
            } else if (key == key_ocs2_) {
                if (fsm_state_received_ && current_fsm_state_ == kFsmOcs2) {
                    toggleArm(!double_click);
                } else {
                    RCLCPP_WARN(get_logger(),
                                "Ignoring F15 click: arm toggle is available only in OCS2");
                }
            } else if (key == key_movej_) {
                requestCartesianEnable(double_click,
                                       double_click ? "F16 double-click" : "F16 click");
            }
            return;
        }

        if (key == key_home_) {
            publishXrEvent(double_click ? kXrEnd : kXrStart);
        } else if (key == key_ocs2_) {
            if (fsm_state_received_ && current_fsm_state_ == kFsmOcs2) {
                toggleArm(!double_click);
            } else {
                RCLCPP_WARN(get_logger(),
                            "Ignoring F15 click: arm toggle is available only in OCS2");
            }
        } else if (key == key_movej_) {
            publishXrEvent(double_click ? kXrDelete : kXrManualIntervention);
        }
    }

    void requestCartesianCalibration(const char * gesture)
    {
        if (!cartesian_calibrate_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(),
                        "%s ignored: Cartesian calibration service is unavailable: %s",
                        gesture, cartesian_calibrate_service_.c_str());
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        cartesian_calibrate_client_->async_send_request(
            request,
            [this, gesture](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                const auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(get_logger(), "%s calibration succeeded: %s", gesture,
                                response->message.c_str());
                } else {
                    RCLCPP_ERROR(get_logger(), "%s calibration failed: %s", gesture,
                                 response->message.c_str());
                }
            });
        RCLCPP_INFO(get_logger(), "%s requested Cartesian calibration", gesture);
    }

    void requestCartesianEnable(const bool enable, const char * gesture)
    {
        if (!cartesian_enable_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "%s: Cartesian %s service is unavailable: %s",
                        gesture, enable ? "enable" : "disable",
                        cartesian_enable_service_.c_str());
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;
        cartesian_enable_client_->async_send_request(
            request,
            [this, enable, gesture](
                rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                const auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(get_logger(), "%s Cartesian teleop %s: %s", gesture,
                                enable ? "enabled" : "disabled", response->message.c_str());
                } else {
                    RCLCPP_ERROR(get_logger(), "%s failed to %s Cartesian teleop: %s", gesture,
                                 enable ? "enable" : "disable", response->message.c_str());
                }
            });
        RCLCPP_INFO(get_logger(), "%s requested Cartesian teleop %s", gesture,
                    enable ? "enable" : "disable");
    }

    void executeLongPressFsm(const int key)
    {
        std::int32_t command = commandForKey(key);
        if (!fsm_state_received_) {
            RCLCPP_WARN(get_logger(), "Ignoring long-press %s: /fsm_state not received",
                        fsmName(command));
            return;
        }
        if (key == key_home_ && current_fsm_state_ == kFsmHome) {
            command = kHomePoseSwitch;
        } else if (current_fsm_state_ != kFsmHold) {
            RCLCPP_WARN(get_logger(),
                        "Ignoring long-press %s: current FSM state is %s; press HOLD first",
                        fsmName(command), fsmName(current_fsm_state_));
            return;
        }

        active_fsm_command_ = true;
        publishCommand(command);
        if (command == kHomePoseSwitch) {
            RCLCPP_INFO(get_logger(), "F14 long press requested HOME pose switch (command=100)");
        } else {
            RCLCPP_INFO(get_logger(), "Pedal long press requested %s", fsmName(command));
        }
    }

    void publishXrEvent(const std::int32_t event)
    {
        pending_xr_event_ = event;
        RCLCPP_INFO(get_logger(), "Pedal queued /xr/controller_state=%d", event);
    }

    void publishXrControllerState()
    {
        std_msgs::msg::Int32 message;
        message.data = pending_xr_event_;
        xr_controller_state_publisher_->publish(message);
        if (pending_xr_event_ != 0) {
            RCLCPP_INFO(get_logger(), "Pedal published /xr/controller_state=%d for one frame",
                        pending_xr_event_);
            pending_xr_event_ = 0;
        }
    }

    void toggleArm(const bool left)
    {
        if (!fsm_state_received_ || current_fsm_state_ != kFsmOcs2) {
            RCLCPP_WARN(get_logger(), "Ignoring F15 click: FSM is no longer OCS2");
            return;
        }
        if (!wbc_state_received_) {
            RCLCPP_WARN(get_logger(),
                        "Ignoring F15 click: /ocs2_wbc_controller/current_state not received");
            return;
        }
        if (bimanual_coupled_) {
            RCLCPP_WARN(get_logger(),
                        "Arm toggle blocked: bimanual coupling is enabled; switch to independent first");
            return;
        }

        const bool enabled = left ? left_arm_enabled_ : right_arm_enabled_;
        std_msgs::msg::String message;
        if (left) {
            message.data = enabled ? "LEFT_ARM_DISABLE" : "LEFT_ARM_ENABLE";
        } else {
            message.data = enabled ? "RIGHT_ARM_DISABLE" : "RIGHT_ARM_ENABLE";
        }
        mode_command_publisher_->publish(message);
        RCLCPP_INFO(get_logger(), "F15 %s-click published mode command: %s",
                    left ? "single" : "double", message.data.c_str());
    }

    void readEvents()
    {
        input_event events[32]{};

        while (rclcpp::ok()) {
            const ssize_t bytes = read(device_fd_, events, sizeof(events));
            if (bytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return;
                }
                if (errno == EINTR) {
                    continue;
                }
                handleDeviceFailure(std::string("read failed: ") + std::strerror(errno));
                return;
            }

            if (bytes == 0) {
                handleDeviceFailure("device disconnected");
                return;
            }

            if (bytes % static_cast<ssize_t>(sizeof(input_event)) != 0) {
                RCLCPP_WARN(get_logger(), "Discarding incomplete input event (%zd bytes)", bytes);
                continue;
            }

            const auto count = static_cast<std::size_t>(bytes) / sizeof(input_event);
            for (std::size_t i = 0; i < count; ++i) {
                if (events[i].type == EV_KEY) {
                    handleKeyEvent(events[i]);
                }
            }
        }
    }

    void handleDeviceFailure(const std::string & reason)
    {
        if (device_failed_) {
            return;
        }
        device_failed_ = true;
        RCLCPP_ERROR(get_logger(), "Foot pedal failure: %s (%s)", reason.c_str(),
                     device_path_.c_str());

        if (hold_on_disconnect_) {
            if (click_profile_ == "dexcap") {
                requestCartesianEnable(false, "pedal disconnect");
            }
            publishCommand(kFsmHold);
            RCLCPP_ERROR(get_logger(), "Fail-safe published fsm_command=2 (HOLD)");
        }

        input_timer_->cancel();
        closeDevice();
        shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
            shutdown_timer_->cancel();
            rclcpp::shutdown();
        });
    }

    void closeDevice()
    {
        if (device_fd_ < 0) {
            return;
        }
        if (grabbed_) {
            if (ioctl(device_fd_, EVIOCGRAB, 0) < 0) {
                RCLCPP_WARN(get_logger(), "Failed to release exclusive device grab: %s",
                            std::strerror(errno));
            }
            grabbed_ = false;
        }
        close(device_fd_);
        device_fd_ = -1;
    }

    std::string device_path_;
    std::string device_serial_;
    bool grab_device_{true};
    bool hold_on_disconnect_{true};
    int double_click_ms_{300};
    int long_press_ms_{700};
    double xr_publish_rate_hz_{30.0};
    std::string mode_command_topic_{"mode_command"};
    std::string click_profile_{"dexcap"};
    std::string cartesian_calibrate_service_;
    std::string cartesian_enable_service_;
    int key_hold_{KEY_F13};
    int key_home_{KEY_F14};
    int key_ocs2_{KEY_F15};
    int key_movej_{KEY_F16};

    int device_fd_{-1};
    bool grabbed_{false};
    bool device_failed_{false};
    int active_key_{-1};
    bool active_fsm_command_{false};
    int active_gesture_key_{-1};
    bool long_press_triggered_{false};
    std::int32_t pending_xr_event_{0};
    bool fsm_state_received_{false};
    std::int32_t current_fsm_state_{0};
    bool wbc_state_received_{false};
    bool left_arm_enabled_{false};
    bool right_arm_enabled_{false};
    bool bimanual_coupled_{false};

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr xr_controller_state_publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cartesian_calibrate_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cartesian_enable_client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_state_subscription_;
    rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCurrentState>::SharedPtr
        wbc_state_subscription_;
    rclcpp::TimerBase::SharedPtr input_timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    rclcpp::TimerBase::SharedPtr long_press_timer_;
    rclcpp::TimerBase::SharedPtr xr_publish_timer_;
    ClickState f14_click_state_;
    ClickState f15_click_state_;
    ClickState f16_click_state_;
};

int main(int argc, char ** argv)
{
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<FootPedalTeleop>());
    } catch (const std::exception & error) {
        std::fprintf(stderr, "foot_pedal_teleop: %s\n", error.what());
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
