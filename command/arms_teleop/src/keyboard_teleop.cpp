//
// Keyboard teleop: stdin (TTY) + 20Hz timer; aligns with joystick_teleop topics & kinematics.
//

#include "arms_teleop/keyboard_teleop.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstring>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

namespace {

constexpr float kNoHandCommand = std::numeric_limits<float>::quiet_NaN();

constexpr std::uint16_t kArrowUp = 0xE001;
constexpr std::uint16_t kArrowDown = 0xE002;
constexpr std::uint16_t kArrowLeft = 0xE003;
constexpr std::uint16_t kArrowRight = 0xE004;
constexpr std::uint16_t kDiscretePlus = 0xE010;
constexpr std::uint16_t kDiscreteEnter = 0xE011;

struct termios g_saved_termios{};
std::atomic<bool> g_tty_saved_global{false};

void restoreTerminalGlobal()
{
    if (g_tty_saved_global.load()) {
        tcsetattr(STDIN_FILENO, TCSADRAIN, &g_saved_termios);
        g_tty_saved_global.store(false);
    }
}

void signalHandler(int signum)
{
    restoreTerminalGlobal();
    std::fputc('\n', stdout);
    std::fflush(stdout);
    _Exit(128 + signum);
}

} // namespace

KeyboardTeleop::KeyboardTeleop() : Node("keyboard_teleop_node")
{
    inputs_pub_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    chassis_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    waist_lift_pub_ =
        create_publisher<std_msgs::msg::Float64>("/body_joint_controller/waist_lifting_command", 10);
    waist_turn_pub_ =
        create_publisher<std_msgs::msg::Float64>("/body_joint_controller/waist_turning_command", 10);

    declare_parameter("update_rate", 20.0);
    declare_parameter("movement_key_stale_ms", 120);
    declare_parameter("discrete_key_stale_ms", 320);
    declare_parameter("chassis.linear_scale", 0.25);
    declare_parameter("chassis.angular_scale", 0.5);
    declare_parameter("arm_axes_activation_threshold", 0.5);
    declare_parameter("show_status_line", true);
    declare_parameter("mirror_movement", false);

    update_hz_ = get_parameter("update_rate").as_double();
    movement_key_stale_ms_ =
        std::chrono::milliseconds(get_parameter("movement_key_stale_ms").as_int());
    discrete_key_stale_ms_ =
        std::chrono::milliseconds(get_parameter("discrete_key_stale_ms").as_int());
    chassis_linear_scale_ = get_parameter("chassis.linear_scale").as_double();
    chassis_angular_scale_ = get_parameter("chassis.angular_scale").as_double();
    arm_activation_threshold_ = get_parameter("arm_axes_activation_threshold").as_double();
    show_status_line_ = get_parameter("show_status_line").as_bool();
    mirror_movement_ = get_parameter("mirror_movement").as_bool();

    inputs_.target = target_arm_;
    inputs_.hand_command = kNoHandCommand;

    const auto period = std::chrono::duration<double>(1.0 / update_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&KeyboardTeleop::timerCallback, this));

    if (!isTtyOk()) {
        RCLCPP_FATAL(get_logger(),
                     "stdin is not a TTY. Run in an interactive terminal: ros2 run arms_teleop keyboard_teleop");
        throw std::runtime_error("keyboard_teleop requires a TTY on stdin");
    }

    struct sigaction sa {};
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    stdin_thread_ = std::thread(&KeyboardTeleop::stdinThreadFunc, this);

    if (rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN) !=
        RCUTILS_RET_OK) {
        // Non-fatal; console may still show INFO from this node.
    }

    std::fprintf(stdout,
                 "keyboard_teleop — WASD / I K / J L / arrows | N mode | M mirror | +/- speed | Space arm | "
                 "Enter grip | Ctrl+C quit\n");
    std::fflush(stdout);
}

KeyboardTeleop::~KeyboardTeleop()
{
    stdin_running_.store(false);
    if (stdin_thread_.joinable()) {
        stdin_thread_.join();
    }
    restoreTerminal();
}

bool KeyboardTeleop::isTtyOk() const
{
    return isatty(STDIN_FILENO) != 0;
}

void KeyboardTeleop::restoreTerminal()
{
    if (tty_saved_) {
        tcsetattr(STDIN_FILENO, TCSADRAIN, &orig_termios_);
        tty_saved_ = false;
        g_tty_saved_global.store(false);
    }
}

void KeyboardTeleop::publishWaistZero()
{
    std_msgs::msg::Float64 z{};
    z.data = 0.0;
    waist_lift_pub_->publish(z);
    waist_turn_pub_->publish(z);
}

double KeyboardTeleop::speedMultiplier() const
{
    const int k = std::clamp(speed_level_, 1, 10);
    return static_cast<double>(k) * 0.1;
}

void KeyboardTeleop::refreshStatusLine()
{
    if (!show_status_line_) {
        return;
    }
    char line[512];
    const double sm = speedMultiplier();
    const char *mode_str = mode_ == ControlMode::Arm ? "ARM" : "CHASSIS";
    const char *mir_str = mirror_movement_ ? "ON" : "OFF";

    if (mode_ == ControlMode::Arm) {
        const bool gopen = (target_arm_ == 1) ? left_gripper_open_ : right_gripper_open_;
        std::snprintf(line, sizeof(line),
                      "\033[2K\r[keyboard] %s | speed %d (%.2fx) | mirror %s | arm %s | gripper %s",
                      mode_str, speed_level_, sm, mir_str, target_arm_ == 1 ? "LEFT" : "RIGHT",
                      gopen ? "OPEN" : "CLOSED");
    } else {
        std::snprintf(line, sizeof(line),
                      "\033[2K\r[keyboard] %s | speed %d (%.2fx) | mirror %s | drive WASD turn JL",
                      mode_str, speed_level_, sm, mir_str);
    }
    std::fputs(line, stdout);
    std::fflush(stdout);
}

bool KeyboardTeleop::isActive(std::uint16_t code,
                              std::chrono::steady_clock::time_point now,
                              std::chrono::milliseconds stale) const
{
    auto it = last_key_time_.find(code);
    if (it == last_key_time_.end()) {
        return false;
    }
    return (now - it->second) < stale;
}

void KeyboardTeleop::stdinThreadFunc()
{
    if (tcgetattr(STDIN_FILENO, &orig_termios_) != 0) {
        return;
    }
    memcpy(&g_saved_termios, &orig_termios_, sizeof(termios));
    g_tty_saved_global.store(true);
    tty_saved_ = true;

    termios raw = orig_termios_;
    cfmakeraw(&raw);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSADRAIN, &raw) != 0) {
        return;
    }

    std::vector<unsigned char> buf(1);
    while (stdin_running_.load()) {
        pollfd pfd{};
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        const int pr = poll(&pfd, 1, 200);
        if (pr < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (pr == 0) {
            continue;
        }
        ssize_t n = read(STDIN_FILENO, buf.data(), 1);
        if (n <= 0) {
            break;
        }
        unsigned char c = buf[0];
        if (c == 3U || c == 4U) {
            stdin_running_.store(false);
            rclcpp::shutdown();
            break;
        }
        std::lock_guard<std::mutex> lock(key_mutex_);
        const auto t = std::chrono::steady_clock::now();

        if (c == 27U) {
            unsigned char seq[2]{};
            pollfd q{};
            q.fd = STDIN_FILENO;
            q.events = POLLIN;
            if (poll(&q, 1, 40) <= 0 || read(STDIN_FILENO, &seq[0], 1) != 1) {
                continue;
            }
            if (seq[0] != '[') {
                continue;
            }
            if (poll(&q, 1, 40) <= 0 || read(STDIN_FILENO, &seq[1], 1) != 1) {
                continue;
            }
            switch (seq[1]) {
                case 'A':
                    last_key_time_[kArrowUp] = t;
                    break;
                case 'B':
                    last_key_time_[kArrowDown] = t;
                    break;
                case 'C':
                    last_key_time_[kArrowRight] = t;
                    break;
                case 'D':
                    last_key_time_[kArrowLeft] = t;
                    break;
                default:
                    break;
            }
            continue;
        }

        switch (c) {
            case 'w':
            case 'W':
                last_key_time_[static_cast<std::uint16_t>('w')] = t;
                break;
            case 'a':
            case 'A':
                last_key_time_[static_cast<std::uint16_t>('a')] = t;
                break;
            case 's':
            case 'S':
                last_key_time_[static_cast<std::uint16_t>('s')] = t;
                break;
            case 'd':
            case 'D':
                last_key_time_[static_cast<std::uint16_t>('d')] = t;
                break;
            case 'i':
            case 'I':
                last_key_time_[static_cast<std::uint16_t>('i')] = t;
                break;
            case 'k':
            case 'K':
                last_key_time_[static_cast<std::uint16_t>('k')] = t;
                break;
            case 'j':
            case 'J':
                last_key_time_[static_cast<std::uint16_t>('j')] = t;
                break;
            case 'l':
            case 'L':
                last_key_time_[static_cast<std::uint16_t>('l')] = t;
                break;
            case 'n':
            case 'N':
                last_key_time_[static_cast<std::uint16_t>('n')] = t;
                break;
            case 'm':
            case 'M':
                last_key_time_[static_cast<std::uint16_t>('m')] = t;
                break;
            case ' ':
                last_key_time_[static_cast<std::uint16_t>(' ')] = t;
                break;
            case '\r':
            case '\n':
                last_key_time_[kDiscreteEnter] = t;
                break;
            case '-':
            case '_':
                last_key_time_[static_cast<std::uint16_t>('-')] = t;
                break;
            case '+':
            case '=':
                last_key_time_[kDiscretePlus] = t;
                break;
            default:
                break;
        }
    }

    restoreTerminal();
}

void KeyboardTeleop::timerCallback()
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(key_mutex_);

    const auto mv = movement_key_stale_ms_;
    const auto dv = discrete_key_stale_ms_;

    const bool aw = isActive(static_cast<std::uint16_t>('w'), now, mv);
    const bool aa = isActive(static_cast<std::uint16_t>('a'), now, mv);
    const bool as = isActive(static_cast<std::uint16_t>('s'), now, mv);
    const bool ad = isActive(static_cast<std::uint16_t>('d'), now, mv);
    const bool ai = isActive(static_cast<std::uint16_t>('i'), now, mv);
    const bool ak = isActive(static_cast<std::uint16_t>('k'), now, mv);
    const bool aj = isActive(static_cast<std::uint16_t>('j'), now, mv);
    const bool al = isActive(static_cast<std::uint16_t>('l'), now, mv);
    const bool aup = isActive(kArrowUp, now, mv);
    const bool adown = isActive(kArrowDown, now, mv);
    const bool aleft = isActive(kArrowLeft, now, mv);
    const bool aright = isActive(kArrowRight, now, mv);

    const bool an = isActive(static_cast<std::uint16_t>('n'), now, dv);
    const bool am = isActive(static_cast<std::uint16_t>('m'), now, dv);
    const bool aspace = isActive(static_cast<std::uint16_t>(' '), now, dv);
    const bool aenter = isActive(kDiscreteEnter, now, dv);
    const bool aminus = isActive(static_cast<std::uint16_t>('-'), now, dv);
    const bool aplus = isActive(kDiscretePlus, now, dv);

    if (prev_active_n_ && !an) {
        mode_ = (mode_ == ControlMode::Arm) ? ControlMode::Chassis : ControlMode::Arm;
        inputs_.x = inputs_.y = inputs_.z = inputs_.roll = inputs_.pitch = inputs_.yaw = 0.0;
        chassis_cmd_.linear.x = chassis_cmd_.linear.y = chassis_cmd_.linear.z = 0.0;
        chassis_cmd_.angular.x = chassis_cmd_.angular.y = chassis_cmd_.angular.z = 0.0;
    }
    prev_active_n_ = an;

    if (prev_active_m_ && !am) {
        mirror_movement_ = !mirror_movement_;
    }
    prev_active_m_ = am;

    if (prev_active_minus_ && !aminus) {
        speed_level_ = std::max(1, speed_level_ - 1);
    }
    prev_active_minus_ = aminus;

    if (prev_active_plus_ && !aplus) {
        speed_level_ = std::min(10, speed_level_ + 1);
    }
    prev_active_plus_ = aplus;

    if (mode_ == ControlMode::Arm && prev_active_space_ && !aspace) {
        target_arm_ = (target_arm_ == 1) ? 2 : 1;
        inputs_.target = target_arm_;
    }
    prev_active_space_ = aspace;

    if (mode_ == ControlMode::Arm && prev_active_enter_ && !aenter) {
        const bool open_now = (target_arm_ == 1) ? left_gripper_open_ : right_gripper_open_;
        const bool should_open = !open_now;
        inputs_.hand_command = should_open ? 1.0f : 0.0f;
        if (target_arm_ == 1) {
            left_gripper_open_ = should_open;
        } else {
            right_gripper_open_ = should_open;
        }
    }
    prev_active_enter_ = aenter;

    double ls_x = static_cast<double>(aa) - static_cast<double>(ad);
    double ls_y = static_cast<double>(aw) - static_cast<double>(as);
    double rs_x = static_cast<double>(aj) - static_cast<double>(al);
    double rs_y = static_cast<double>(ai) - static_cast<double>(ak);
    double dp_x = static_cast<double>(aleft) - static_cast<double>(aright);
    double dp_y = static_cast<double>(aup) - static_cast<double>(adown);

    auto applyActivation = [this](double v) -> double {
        return (std::abs(v) >= arm_activation_threshold_) ? v : 0.0;
    };

    // Apply mirroring directly from the runtime flag to match status/UI semantics:
    // mirror OFF => no inversion, mirror ON => mirrored arm controls.
    const bool mir_apply = mirror_movement_;

    double ls_x_a = ls_x;
    double ls_y_a = ls_y;
    double rs_x_a = rs_x;
    double rs_y_a = rs_y;
    if (mir_apply) {
        ls_x_a = -ls_x_a;
        ls_y_a = -ls_y_a;
        rs_x_a = -rs_x_a;
    }

    double dp_x_a = dp_x;
    double dp_y_a = dp_y;
    if (!mir_apply) {
        dp_x_a = -dp_x_a;
    }

    ls_x_a = applyActivation(ls_x_a);
    ls_y_a = applyActivation(ls_y_a);
    rs_x_a = applyActivation(rs_x_a);
    rs_y_a = applyActivation(rs_y_a);

    // Chassis uses the same stick axes as joystick processChassisAxes — mirror OFF.
    const double ls_x_ch = applyActivation(ls_x);
    const double ls_y_ch = applyActivation(ls_y);
    const double rs_x_ch = applyActivation(rs_x);

    const double sm = speedMultiplier();

    if (mode_ == ControlMode::Arm) {
        inputs_.x = static_cast<float>(ls_y_a * sm);
        inputs_.y = static_cast<float>(ls_x_a * sm);
        inputs_.z = static_cast<float>(rs_y_a * sm);
        inputs_.roll = static_cast<float>(dp_x_a * sm);
        inputs_.pitch = static_cast<float>(-dp_y_a * sm);
        inputs_.yaw = static_cast<float>(rs_x_a * sm);
        inputs_.target = target_arm_;

        publishWaistZero();
        chassis_cmd_.linear.x = chassis_cmd_.linear.y = chassis_cmd_.linear.z = 0.0;
        chassis_cmd_.angular.x = chassis_cmd_.angular.y = chassis_cmd_.angular.z = 0.0;
        chassis_pub_->publish(chassis_cmd_);

        inputs_pub_->publish(inputs_);
        if (std::isfinite(inputs_.hand_command)) {
            inputs_.hand_command = kNoHandCommand;
        }

    } else {
        inputs_.x = inputs_.y = inputs_.z = inputs_.roll = inputs_.pitch = inputs_.yaw = 0.0;
        inputs_.target = target_arm_;
        inputs_pub_->publish(inputs_);

        chassis_cmd_.linear.x = ls_y_ch * chassis_linear_scale_ * sm;
        chassis_cmd_.linear.y = ls_x_ch * chassis_linear_scale_ * sm;
        chassis_cmd_.linear.z = 0.0;
        chassis_cmd_.angular.x = 0.0;
        chassis_cmd_.angular.y = 0.0;
        chassis_cmd_.angular.z = rs_x_ch * chassis_angular_scale_ * sm;
        chassis_pub_->publish(chassis_cmd_);

        publishWaistZero();
    }

    refreshStatusLine();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<KeyboardTeleop>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        restoreTerminalGlobal();
        fprintf(stderr, "keyboard_teleop: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    restoreTerminalGlobal();
    std::fputc('\n', stdout);
    std::fflush(stdout);
    rclcpp::shutdown();
    return 0;
}
