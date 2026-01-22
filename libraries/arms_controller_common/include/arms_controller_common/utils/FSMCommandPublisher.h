//
// FSM Command Publisher - Common utility for publishing FSM commands
//
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

namespace arms_controller_common
{
    /**
     * @brief FSM Command Publisher - Utility class for publishing FSM commands with special handling
     *
     * This utility class provides common functions for publishing FSM commands to /fsm_command topic.
     * It handles special cases like command=100 (pose switching) which requires delayed reset command.
     *
     * Usage example:
     * ```cpp
     * auto fsm_publisher = node->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
     * FSMCommandPublisher cmd_publisher(node, fsm_publisher);
     *
     * // Regular commands (automatically handled)
     * cmd_publisher.publishCommand(2);  // HOLD - simple publish
     *
     * // Command with special handling (command=100)
     * cmd_publisher.publishCommand(100);  // Will auto-send command=0 after 100ms
     * ```
     */
    class FSMCommandPublisher
    {
    public:
        using PublisherT = rclcpp::Publisher<std_msgs::msg::Int32>;
        using PublisherSharedPtr = typename PublisherT::SharedPtr;

        /**
         * @brief Constructor
         * @param node ROS2 node (for creating timers)
         * @param publisher FSM command publisher
         */
        FSMCommandPublisher(
            rclcpp::Node::SharedPtr node,
            PublisherSharedPtr publisher)
            : node_(std::move(node)), publisher_(std::move(publisher))
        {
        }

        /**
         * @brief Destructor - cancels any outstanding timer
         */
        ~FSMCommandPublisher()
        {
            std::lock_guard<std::mutex> lk(timer_mutex_);
            if (reset_timer_)
            {
                reset_timer_->cancel();
                reset_timer_.reset();
            }
        }

        FSMCommandPublisher(const FSMCommandPublisher&) = delete;
        FSMCommandPublisher& operator=(const FSMCommandPublisher&) = delete;
        FSMCommandPublisher(FSMCommandPublisher&&) = delete;
        FSMCommandPublisher& operator=(FSMCommandPublisher&&) = delete;

        /**
         * @brief Publish FSM command with automatic handling of special cases
         *
         * Special handling:
         * - command=100: Automatically sends command=0 after 100ms delay to reset debounce flag
         *
         * @param command FSM command value (1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ, 100=switch pose, 0=reset)
         */
        void publishCommand(std::int32_t command)
        {
            if (!publisher_)
            {
                return;
            }

            // Publish the command immediately
            std_msgs::msg::Int32 msg;
            msg.data = command;
            publisher_->publish(msg);

            // Special handling: command=100 requires delayed reset command
            if (command == 100)
            {
                if (!node_)
                {
                    return;
                }

                // Cancel existing timer if any
                {
                    std::lock_guard<std::mutex> lk(timer_mutex_);
                    if (reset_timer_)
                    {
                        reset_timer_->cancel();
                        reset_timer_.reset();
                    }
                }

                // Capture publisher (not `this`) to avoid use-after-free
                auto pub = publisher_;

                // Use shared_ptr to hold weak_ptr so lambda can see post-assignment value
                auto weak_timer_holder = std::make_shared<std::weak_ptr<rclcpp::TimerBase>>();

                auto timer = node_->create_wall_timer(
                    std::chrono::milliseconds(100),
                    [pub, weak_timer_holder]()
                    {
                        if (pub)
                        {
                            std_msgs::msg::Int32 reset_msg;
                            reset_msg.data = 0;
                            pub->publish(reset_msg);
                        }

                        // One-shot: cancel self
                        if (auto t = weak_timer_holder->lock())
                        {
                            t->cancel();
                        }
                    });

                *weak_timer_holder = timer;

                {
                    std::lock_guard<std::mutex> lk(timer_mutex_);
                    reset_timer_ = timer;
                }
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        PublisherSharedPtr publisher_;

        std::mutex timer_mutex_;
        rclcpp::TimerBase::SharedPtr reset_timer_;
    };
} // namespace arms_controller_common
