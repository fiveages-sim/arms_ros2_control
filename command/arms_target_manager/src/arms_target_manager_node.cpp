//
// Created for Arms ROS2 Control - ArmsTargetManager Node
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_target_manager/ControlInputHandler.h"
#include "arms_target_manager/VRInputHandler.h"

using namespace arms_ros2_control::command;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("arms_target_manager_node");

    bool dual_arm_mode = node->declare_parameter("dual_arm_mode", false);
    std::string control_base_frame = node->declare_parameter("control_base_frame", "world");
    std::string marker_fixed_frame = node->declare_parameter("marker_fixed_frame", "base_link");
    std::string body_controller_name = node->declare_parameter("body_controller_name", "body_joint_controller");
    double linear_scale = node->declare_parameter("linear_scale", 0.1);
    double angular_scale = node->declare_parameter("angular_scale", 0.25);
    double control_input_rate = node->declare_parameter("control_input_rate", 50.0);
    double vr_pose_scale = node->declare_parameter("vr_pose_scale", 1.0);
    std::string reference_link = node->declare_parameter("reference_link", "head_link2");
    std::string vr_follow_frame =
        node->declare_parameter("vr_follow_frame", "base_footprint");
    if (vr_follow_frame.empty())
    {
        RCLCPP_WARN(
            node->get_logger(),
            "vr_follow_frame is empty; falling back to base_footprint");
        vr_follow_frame = "base_footprint";
    }

    bool enable_vr = node->declare_parameter("enable_vr", true);
    bool enable_movej_cartesian_markers = node->declare_parameter(
        "enable_movej_cartesian_markers",
#ifdef HAS_LINA_PLANNING
        true
#else
        false
#endif
    );

    std::vector<std::string> hand_controllers = node->declare_parameter("hand_controllers", std::vector<std::string>());

    bool auto_detected_dual_arm = hand_controllers.size() > 1;

    if (!hand_controllers.empty())
    {
        std::string controllers_list;
        for (size_t i = 0; i < hand_controllers.size(); ++i) {
            if (i > 0) controllers_list += ", ";
            controllers_list += hand_controllers[i];
        }

        if (dual_arm_mode != auto_detected_dual_arm)
        {
            RCLCPP_WARN(node->get_logger(),
                        "⚠️  Dual arm mode mismatch: parameter=%s, auto-detected=%s (from %zu hand controllers)",
                        dual_arm_mode ? "true" : "false",
                        auto_detected_dual_arm ? "true" : "false",
                        hand_controllers.size());
            RCLCPP_WARN(node->get_logger(),
                        "⚠️  Detected hand controllers: %s",
                        controllers_list.c_str());
        }
        else
        {
            RCLCPP_INFO(node->get_logger(),
                        "✅ Hand controllers detected: %zu controller(s) - %s mode",
                        hand_controllers.size(),
                        auto_detected_dual_arm ? "DUAL-ARM" : "SINGLE-ARM");
            RCLCPP_INFO(node->get_logger(), "   Controllers: %s", controllers_list.c_str());
        }
    }

    RCLCPP_INFO(node->get_logger(),
                "Starting ArmsTargetManager with dual_arm_mode: %s, control_base_frame: %s, marker_fixed_frame: %s, body_controller: %s",
                dual_arm_mode ? "true" : "false",
                control_base_frame.c_str(),
                marker_fixed_frame.c_str(),
                body_controller_name.c_str());
    RCLCPP_INFO(node->get_logger(),
                "Joystick/keyboard scales: linear=%.3f, angular=%.3f (deadzone handled at input source)",
                linear_scale, angular_scale);

    RCLCPP_INFO(node->get_logger(),
            "VR control: enabled=%s",
            enable_vr ? "true" : "false");
    if (enable_vr)
    {
        RCLCPP_INFO(node->get_logger(),
                    "VR thumbstick uses joystick velocity scales: linear=%.3f m/s, angular=%.3f rad/s",
                    linear_scale, angular_scale);
        RCLCPP_INFO(node->get_logger(),
                    "VR pose scale: left=1.0, right=1.0 (fixed, not configurable)");
        RCLCPP_INFO(node->get_logger(),
                    "VR reference_link: %s (from target_manager.yaml, default=head_link2)",
                    reference_link.c_str());
        RCLCPP_INFO(
            node->get_logger(),
            "VR follow frame: %s (used by VR target mapping)",
            vr_follow_frame.c_str());
    }

    try
    {
        auto pub_left_target = node->create_publisher<geometry_msgs::msg::Pose>("left_target", 1);
        auto pub_left_target_stamped =
            node->create_publisher<geometry_msgs::msg::PoseStamped>("left_target/stamped", 1);
        auto pub_left_twist =
            node->create_publisher<geometry_msgs::msg::Twist>("left_target/twist", 10);
        auto pub_right_twist =
            node->create_publisher<geometry_msgs::msg::Twist>("right_target/twist", 10);
        auto pub_head_target = node->create_publisher<geometry_msgs::msg::Pose>("head_target", 10);
        auto pub_head_target_stamped =
            node->create_publisher<geometry_msgs::msg::PoseStamped>("head_target/stamped", 10);
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_target_stamped;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dual_target_stamped;
        if (dual_arm_mode)
        {
            pub_right_target = node->create_publisher<geometry_msgs::msg::Pose>("right_target", 1);
            pub_right_target_stamped =
                node->create_publisher<geometry_msgs::msg::PoseStamped>("right_target/stamped", 1);
            pub_dual_target_stamped =
                node->create_publisher<nav_msgs::msg::Path>("dual_target/stamped", 1);
        }

        RCLCPP_INFO(node->get_logger(),
                    "✅ Created unified target publishers (dual_arm=%s)",
                    dual_arm_mode ? "true" : "false");

        const bool movej_cartesian_markers =
#ifdef HAS_LINA_PLANNING
            enable_movej_cartesian_markers;
#else
            false;
#endif
        if (enable_movej_cartesian_markers && !movej_cartesian_markers)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "enable_movej_cartesian_markers is true but lina_planning is not compiled in; MOVEJ markers stay hidden");
        }

        std::vector<int32_t> disable_auto_update_states{3};
        if (movej_cartesian_markers)
        {
            disable_auto_update_states.push_back(4);
        }
        RCLCPP_INFO(
            node->get_logger(),
            "Interactive arm markers: OCS2%s",
            movej_cartesian_markers ? "+MOVEJ (IK MoveL)" : " only");

        auto target_manager = std::make_unique<ArmsTargetManager>(
            node, dual_arm_mode, control_base_frame, marker_fixed_frame,
            20.0,
            disable_auto_update_states,
            0.05);

        target_manager->initialize(
            pub_left_target, pub_left_target_stamped,
            pub_right_target, pub_right_target_stamped,
            pub_head_target, pub_head_target_stamped,
            pub_dual_target_stamped);

        auto control_handler = std::make_unique<ControlInputHandler>(
            node, target_manager.get(),
            pub_left_twist, pub_right_twist,
            linear_scale, angular_scale, control_input_rate, hand_controllers);

        std::unique_ptr<VRInputHandler> vr_handler = nullptr;
        if (enable_vr)
        {
            vr_handler = std::make_unique<VRInputHandler>(
                node, target_manager.get(),
                pub_left_target, pub_right_target,
                pub_left_twist, pub_right_twist,
                pub_head_target, pub_dual_target_stamped,
                target_manager->getTfBuffer(),
                hand_controllers,
                linear_scale, angular_scale,
                vr_pose_scale, reference_link, vr_follow_frame);
        }

        auto control_subscription = node->create_subscription<arms_ros2_control_msgs::msg::Inputs>(
            "control_input", 10,
            [control_handler_ptr = control_handler.get()](
            const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg)
            {
                if (control_handler_ptr)
                {
                    control_handler_ptr->processControlInput(msg);
                }
            });

        auto fsm_command_subscription = node->create_subscription<std_msgs::msg::Int32>(
            "fsm_command", 10,
            [target_manager_ptr = target_manager.get(), vr_handler_ptr = vr_handler.get()](
            const std_msgs::msg::Int32::ConstSharedPtr msg)
            {
                if (target_manager_ptr)
                {
                    target_manager_ptr->fsmCommandCallback(msg);
                }
                if (vr_handler_ptr)
                {
                    auto shared_msg = std::const_pointer_cast<std_msgs::msg::Int32>(msg);
                    vr_handler_ptr->fsmCommandCallback(shared_msg);
                }
            });

        if (enable_vr && vr_handler)
        {
            target_manager->setWbcStateCallback(
                [vr_handler_ptr = vr_handler.get()](
                    const arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr& msg)
                {
                    if (vr_handler_ptr)
                    {
                        vr_handler_ptr->wbcStateCallback(msg);
                    }
                });

            target_manager->setCurrentPoseCallback("left",
                [vr_handler_ptr = vr_handler.get()](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
                {
                    if (vr_handler_ptr)
                    {
                        auto shared_msg = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(msg);
                        vr_handler_ptr->robotLeftPoseCallback(shared_msg);
                    }
                });

            if (dual_arm_mode)
            {
                target_manager->setCurrentPoseCallback("right",
                    [vr_handler_ptr = vr_handler.get()](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
                    {
                        if (vr_handler_ptr)
                        {
                            auto shared_msg = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(msg);
                            vr_handler_ptr->robotRightPoseCallback(shared_msg);
                        }
                    });
            }
        }

        RCLCPP_INFO(node->get_logger(), "ArmsTargetManager is ready!");
        RCLCPP_INFO(node->get_logger(), "Use RViz to interact with the markers.");
        RCLCPP_INFO(node->get_logger(), "Control input handler is ready for joystick/keyboard control!");
        if (enable_vr)
        {
            RCLCPP_INFO(node->get_logger(), "VR input handler is ready for VR control!");
        }

        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in ArmsTargetManager: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
