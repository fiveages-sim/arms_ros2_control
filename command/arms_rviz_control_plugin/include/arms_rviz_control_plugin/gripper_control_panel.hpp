#pragma once

#include <memory>

#include <QPushButton>
#include <QLabel>

#include <QTimer>
#include <string>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <map>

namespace arms_rviz_control_plugin
{
    class GripperControlPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        GripperControlPanel(QWidget* parent = nullptr);
        ~GripperControlPanel() override;

        void onInitialize() override;

        // Load and save configuration data
        void load(const rviz_common::Config& config) override;
        void save(rviz_common::Config config) const override;

    private Q_SLOTS:
        void onLeftGripperToggle();
        void onRightGripperToggle();

    private:
        void onTargetCommandReceived(const std::string& controller_name, const std_msgs::msg::Int32::SharedPtr msg);
        void publishGripperCommand(bool open, const std::string& controller_name);
        void updateGripperDisplay();
        void updateButtonVisibility();

        // Helper functions
        void createTargetCommandPublishers();
        void createTargetCommandSubscriptions();
        void determineControllerMapping();
        std::string getDisplayNameFromControllerName(const std::string& controller_name);


        // ROS2
        rclcpp::Node::SharedPtr node_;

        // target_command publishers (controller name -> publisher)
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> target_command_publishers_;

        // target_command subscriptions (controller name -> subscription)
        std::map<std::string, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> target_command_subscriptions_;

        // Controller name to button state mapping (controller name -> is_open)
        std::map<std::string, bool*> controller_to_state_;

        // Controller names for left and right buttons
        std::string left_controller_name_;
        std::string right_controller_name_;
        
        // Display names for buttons (computed once during initialization)
        std::string left_display_name_;
        std::string right_display_name_;


        // UI Elements
        std::unique_ptr<QPushButton> left_gripper_btn_;
        std::unique_ptr<QPushButton> right_gripper_btn_;
        std::unique_ptr<QLabel> no_controller_label_;

        // Robot configuration
        bool is_dual_arm_mode_ = false;
        std::vector<std::string> hand_controllers_;

        // Gripper state tracking
        bool left_gripper_open_ = false; // Left arm gripper state
        bool right_gripper_open_ = false; // Right arm gripper state
    };
} // namespace arms_rviz_control_plugin
