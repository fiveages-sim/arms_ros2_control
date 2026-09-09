// USB keyboard foot-pedal adapter for the XR controller event stream.

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
#include <std_msgs/msg/int32.hpp>

namespace {

constexpr std::int32_t kFsmUp = 11;
constexpr std::int32_t kFsmDown = 12;
constexpr std::int32_t kLeftArmToggle = 3;
constexpr std::int32_t kRightArmToggle = 6;
constexpr std::int32_t kWholeTeleopToggle = 4;

const char * eventName(const std::int32_t event)
{
    switch (event) {
    case kFsmUp:
        return "FSM_UP";
    case kFsmDown:
        return "FSM_DOWN";
    case kLeftArmToggle:
        return "LEFT_ARM_TELEOP_TOGGLE";
    case kRightArmToggle:
        return "RIGHT_ARM_TELEOP_TOGGLE";
    case kWholeTeleopToggle:
        return "WHOLE_TELEOP_TOGGLE";
    default:
        return "UNKNOWN";
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
        publish_rate_hz_ = declare_parameter<double>("xr.publish_rate_hz", 30.0);
        controller_state_topic_ = declare_parameter<std::string>(
            "xr.controller_state_topic", "/xr/controller_state");
        enable_whole_teleop_toggle_ =
            declare_parameter<bool>("enable_whole_teleop_toggle", false);

        key_fsm_up_ = static_cast<int>(
            declare_parameter<std::int64_t>("keys.fsm_up", KEY_F13));
        key_fsm_down_ = static_cast<int>(
            declare_parameter<std::int64_t>("keys.fsm_down", KEY_F14));
        key_left_arm_toggle_ = static_cast<int>(
            declare_parameter<std::int64_t>("keys.left_arm_toggle", KEY_F15));
        key_right_arm_toggle_ = static_cast<int>(
            declare_parameter<std::int64_t>("keys.right_arm_toggle", KEY_F16));
        key_whole_teleop_toggle_ = static_cast<int>(
            declare_parameter<std::int64_t>("keys.whole_teleop_toggle", KEY_F17));

        validateParameters();
        device_path_ = resolveDevicePath();

        controller_state_publisher_ = create_publisher<std_msgs::msg::Int32>(
            controller_state_topic_, rclcpp::QoS(10).reliable());
        openDevice();

        RCLCPP_INFO(
            get_logger(),
            "Pedal mapping: F13=FSM up(11), F14=FSM down(12), "
            "F15=left arm toggle(3), F16=right arm toggle(6), F17=whole teleop toggle(4:%s)",
            enable_whole_teleop_toggle_ ? "enabled" : "disabled");
        RCLCPP_INFO(get_logger(), "Publishing XR events to %s",
                    controller_state_topic_.c_str());

        input_timer_ = create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&FootPedalTeleop::readEvents, this));
        const auto state_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        state_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(state_period),
            std::bind(&FootPedalTeleop::publishIdleState, this));
    }

    ~FootPedalTeleop() override { closeDevice(); }

    FootPedalTeleop(const FootPedalTeleop &) = delete;
    FootPedalTeleop & operator=(const FootPedalTeleop &) = delete;

private:
    void validateParameters() const
    {
        if (device_path_.empty()) {
            throw std::runtime_error("parameter 'device' must be 'auto' or an evdev device path");
        }
        if (controller_state_topic_.empty()) {
            throw std::runtime_error("xr.controller_state_topic must not be empty");
        }
        if (publish_rate_hz_ < 1.0 || publish_rate_hz_ > 200.0) {
            throw std::runtime_error("xr.publish_rate_hz must be between 1 and 200 Hz");
        }

        const std::array<int, 5> keys{
            key_fsm_up_, key_fsm_down_, key_left_arm_toggle_, key_right_arm_toggle_,
            key_whole_teleop_toggle_};
        for (const int key : keys) {
            if (key < 0 || key > KEY_MAX) {
                throw std::runtime_error("configured pedal key code is outside the Linux input range");
            }
        }
        auto active_keys = keys;
        const auto active_end = enable_whole_teleop_toggle_ ? active_keys.end() : active_keys.end() - 1;
        std::sort(active_keys.begin(), active_end);
        if (std::adjacent_find(active_keys.begin(), active_end) != active_end) {
            throw std::runtime_error("enabled pedal functions must use distinct Linux key codes");
        }
    }

    std::string resolveDevicePath() const
    {
        if (device_path_ != "auto") {
            return device_path_;
        }

        constexpr const char * pattern =
            "/dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd";
        constexpr const char * prefix = "usb-SayoDevice_CM6K_";
        constexpr const char * suffix = "-event-kbd";
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
            if (name.find("-if01-") != std::string::npos || name.rfind(prefix, 0) != 0 ||
                name.size() <= std::strlen(prefix) + std::strlen(suffix) ||
                name.compare(name.size() - std::strlen(suffix), std::strlen(suffix), suffix) != 0) {
                continue;
            }
            const std::size_t serial_begin = std::strlen(prefix);
            const std::size_t serial_length =
                name.size() - serial_begin - std::strlen(suffix);
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
            error << (device_serial_.empty() ? "no SayoDevice CM6K primary keyboard interface found"
                                            : "no SayoDevice CM6K found with serial '" +
                                                  device_serial_ + "'");
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
            throw std::runtime_error("cannot open '" + device_path_ + "': " +
                                     std::strerror(errno) +
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
                throw std::runtime_error("cannot exclusively grab '" + device_path_ + "': " +
                                         error);
            }
            grabbed_ = true;
        }
        RCLCPP_INFO(get_logger(), "Opened foot pedal: %s (%s)", device_name,
                    device_path_.c_str());
    }

    std::int32_t eventForKey(const std::uint16_t key) const
    {
        if (key == key_fsm_up_) return kFsmUp;
        if (key == key_fsm_down_) return kFsmDown;
        if (key == key_left_arm_toggle_) return kLeftArmToggle;
        if (key == key_right_arm_toggle_) return kRightArmToggle;
        if (enable_whole_teleop_toggle_ && key == key_whole_teleop_toggle_) {
            return kWholeTeleopToggle;
        }
        return 0;
    }

    void handleKeyEvent(const input_event & event)
    {
        // Publish once on the rising edge; ignore release and kernel key-repeat.
        if (event.value != 1) return;
        const std::int32_t xr_event = eventForKey(event.code);
        if (xr_event == 0) return;

        std_msgs::msg::Int32 message;
        message.data = xr_event;
        controller_state_publisher_->publish(message);
        RCLCPP_INFO(get_logger(), "Published %s=%d to %s", eventName(xr_event), xr_event,
                    controller_state_topic_.c_str());
    }

    void publishIdleState()
    {
        std_msgs::msg::Int32 message;
        message.data = 0;
        controller_state_publisher_->publish(message);
    }

    void readEvents()
    {
        input_event events[32]{};
        while (rclcpp::ok()) {
            const ssize_t bytes = read(device_fd_, events, sizeof(events));
            if (bytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) return;
                if (errno == EINTR) continue;
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
            for (std::size_t index = 0; index < count; ++index) {
                if (events[index].type == EV_KEY) handleKeyEvent(events[index]);
            }
        }
    }

    void handleDeviceFailure(const std::string & reason)
    {
        if (device_failed_) return;
        device_failed_ = true;
        RCLCPP_ERROR(get_logger(), "Foot pedal failure: %s (%s)", reason.c_str(),
                     device_path_.c_str());
        publishIdleState();
        input_timer_->cancel();
        closeDevice();
        shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
            shutdown_timer_->cancel();
            rclcpp::shutdown();
        });
    }

    void closeDevice()
    {
        if (device_fd_ < 0) return;
        if (grabbed_) {
            if (ioctl(device_fd_, EVIOCGRAB, 0) < 0) {
                RCLCPP_WARN(get_logger(), "Failed to release device grab: %s",
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
    double publish_rate_hz_{30.0};
    std::string controller_state_topic_{"/xr/controller_state"};
    bool enable_whole_teleop_toggle_{false};
    int key_fsm_up_{KEY_F13};
    int key_fsm_down_{KEY_F14};
    int key_left_arm_toggle_{KEY_F15};
    int key_right_arm_toggle_{KEY_F16};
    int key_whole_teleop_toggle_{KEY_F17};
    int device_fd_{-1};
    bool grabbed_{false};
    bool device_failed_{false};
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr controller_state_publisher_;
    rclcpp::TimerBase::SharedPtr input_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char ** argv)
{
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<FootPedalTeleop>());
    } catch (const std::exception & error) {
        std::fprintf(stderr, "foot_pedal_teleop: %s\n", error.what());
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
