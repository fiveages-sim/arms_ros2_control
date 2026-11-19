//
// ArmsTargetManager - 节点门面。
//
// 负责三件事，别的都不做：
//   ① 创建并维护 RViz 里的交互 marker（左臂 / 右臂 / 头 / 胸）；
//   ② 把 marker 的拖动、菜单操作翻译成“目标提交”交给 TargetRouter；
//   ③ 汇总 WBC / FSM 状态，回答“这个 marker 现在该不该显示、该不该跟随”。
//
// 目标发布不在这里发生：所有话题都由 core/TargetRouter 统一发，
// marker 只负责显示。改这里时请保持这个边界，否则控制权会再次分裂。
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <array>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include "arms_target_manager/MarkerFactory.h"
#include "arms_target_manager/core/FsmClient.h"
#include "arms_target_manager/core/HumanoidModeClient.h"
#include "arms_target_manager/core/TargetRouter.h"
#include "arms_target_manager/marker/HeadMarker.h"
#include "arms_target_manager/marker/ArmMarker.h"
#include "arms_target_manager/marker/BodyMarker.h"

namespace arms_ros2_control::command
{
    class ArmsTargetManager
    {
    public:
        ArmsTargetManager(
            rclcpp::Node::SharedPtr node,
            bool dualArmMode = false,
            std::string frameId = "world",
            std::string markerFixedFrame = "base_footprint",
            double publishRate = 20.0,
            const std::vector<int32_t>& disableAutoUpdateStates =
#ifdef HAS_LINA_PLANNING
                {3, 4},
#else
                {3},
#endif
            double markerUpdateInterval = 0.05);

        ~ArmsTargetManager() = default;

        void initialize(
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_target_stamped,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target = nullptr,
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_target_stamped = nullptr);

        void setMarkerPose(
            const std::string& armType,
            const geometry_msgs::msg::Point& position,
            const geometry_msgs::msg::Quaternion& orientation);

        void updateMarkerPoseIncremental(
            const std::string& armType,
            const std::array<double, 3>& positionDelta,
            const std::array<double, 3>& rpyDelta,
            bool publish = true);

        bool updateBodyMarkerPoseIncremental(
            const std::array<double, 3>& positionDelta,
            const std::array<double, 3>& rpyDelta);

        /**
         * 直接设置身体 marker 的绝对目标位姿（全身遥操骨架驱动用）。
         * 前置条件与 updateBodyMarkerPoseIncremental 相同（身体 marker 存在且可见），
         * 设置后按当前 marker 发布模式（连续/单次）更新可视化与身体目标。
         * @return true 表示已应用；false 表示身体 marker 不可用
         */
        bool setBodyMarkerPose(const geometry_msgs::msg::Pose& pose);

        geometry_msgs::msg::Pose getMarkerPose(const std::string& armType) const;

        void togglePublishMode();

        MarkerState getCurrentMode() const;
        /** RViz 交互 marker 的固定坐标系（通常是 world）。 */
        const std::string& getMarkerFixedFrame() const { return marker_fixed_frame_; }
        /** 目标发布的坐标系：下游 WBC 的 base_frame_（移动底盘=world，固定底盘=baseFrame）。 */
        const std::string& getControlBaseFrame() const { return control_base_frame_; }

        /** 目标汇聚器；VR / control_input / 兼容接口都通过它下发目标。 */
        [[nodiscard]] TargetRouter* router() const { return router_.get(); }

        /** 摇杆/键盘的速度流现在能不能写这条手臂（被 VR 或手动拖动占用时返回 false）。 */
        [[nodiscard]] bool canAcceptSpeedCommand(bool is_left) const;

        void sendTargetPose(const std::string& marker_type = "arm");

        void sendDualArmTargetPose();

        void sendBodyTargetPose();

        bool isStateDisabled(int32_t state) const;

        bool shouldThrottle(rclcpp::Time& last_time, double interval);

        bool shouldShowLeftArmMarker() const;
        bool shouldShowRightArmMarker() const;
        bool shouldShowBodyMarker() const;
        bool shouldShowWbcHeadMarker() const;
        bool shouldShowHeadMarker() const;
        void setHeadVrActive(bool active);

        /**
         * 「VR 独占臂目标」标志：全身遥操作的绝对映射正在下发某臂目标时置位。
         * 当前唯一消费者是 `ControlInputHandler` 的**手柄/键盘 `control_input`**：
         * 置位期间它不再改写该臂目标（避免 twist 速度流与 VR 绝对映射抢同一目标）。
         * 注意：**RViz 交互 marker 的拖拽/菜单不再门控**——它们是调试通道，
         * 允许随时手工干预（跟踪中拖拽会被 VR 下一帧覆盖）。
         */
        void setArmVrActive(bool is_left, bool active);
        [[nodiscard]] bool isArmVrActive(bool is_left) const { return is_left ? left_arm_vr_active_ : right_arm_vr_active_; }
        int getCurrentBodyState() const;
        int getCurrentHeadState() const { return head_state_; }
        int getCurrentBaseState() const;
        int getCurrentBimanualState() const;
        int getCurrentLeftArmState() const;
        int getCurrentRightArmState() const;
        const std::string& getCurrentFsmState() const { return current_fsm_state_; }

        void markPendingChanges();

        void markerUpdateTimerCallback();

        /**
         * 接入 FSM 统一入口。
         *
         * 接入后本类不再自己订阅 fsm_command 也不再推断状态：
         * `current_fsm_state_` / `current_controller_state_` 变成为兼容旧代码保留的
         * 缓存，由 FsmClient 的状态变化回调刷新。
         */
        void attachFsmClient(FsmClient* client);

        /** 当前 FSM 统一入口（未接入时返回 nullptr）。 */
        [[nodiscard]] FsmClient* fsmClient() const { return fsm_; }

        /**
         * 接入 WBC 模式统一入口。
         *
         * 接入后本类不再自己订阅 `<controller>/current_state`，
         * 而是由 HumanoidModeClient 推送；`wbcStateCallback` 的逻辑保持不变。
         */
        void attachHumanoidModeClient(HumanoidModeClient* client);

        void wbcStateCallback(const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg);

        /**
         * 转发某条通道的实测位姿消息（复用 marker 已有的订阅，不重复建订阅）。
         *
         * @param armType "left" / "right" / "body"
         * @param callback 收到 `*_current_pose` 时被调用
         */
        void setCurrentPoseCallback(
            const std::string& armType,
            std::function<void(const geometry_msgs::msg::PoseStamped::ConstSharedPtr&)> callback);

        void setWbcStateCallback(
            std::function<void(
                const arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr&)>
                callback);

        void currentTargetJointCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg);

    private:
        visualization_msgs::msg::InteractiveMarker buildMarker(
            const std::string& name,
            const std::string& markerType) const;

        void handleMarkerFeedback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

        void setupMenu();

        void setupMarkerMenu(
            std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
            interactive_markers::MenuHandler::EntryHandle& send_handle,
            interactive_markers::MenuHandler::EntryHandle& toggle_handle,
            std::function<void()> sendCallback);

        void setupDualArmMenu(
            std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
            interactive_markers::MenuHandler::EntryHandle& both_handle);

        void updateMarkerShape();

        void updateMenuVisibility();

        void createPublishersAndSubscribers();

        void updateBodyMarkerVisibility();
        void updateHeadMarkerVisibility();
        void refreshArmMarkersFromLatestCurrentTargets();
        void refreshArmMarkersFromLatestCurrentPoses();

        /** 把 FSM 状态应用到 marker 形态、菜单和可见性（旧 fsmCommandCallback 的动作）。 */
        void applyFsmState(FsmState state);

        [[nodiscard]] bool isBimanualCoupled() const;

        [[nodiscard]] bool shouldStreamPoseCommands() const;

        void followCoupledRightMarker(
            const geometry_msgs::msg::Pose& old_left_pose,
            const geometry_msgs::msg::Pose& old_right_pose,
            const geometry_msgs::msg::Pose& new_left_pose);

        void followCoupledLeftMarker(
            const geometry_msgs::msg::Pose& old_right_pose,
            const geometry_msgs::msg::Pose& old_left_pose,
            const geometry_msgs::msg::Pose& new_right_pose);

        static bool bodyJointTargetsChanged(
            const std::vector<double>& previous,
            const std::vector<double>& current,
            double threshold);

        geometry_msgs::msg::Pose transformPose(
            const geometry_msgs::msg::Pose& pose,
            const std::string& sourceFrameId,
            const std::string& targetFrameId) const;

        void setServerPose(const std::string& name, const geometry_msgs::msg::Pose& pose);

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

        std::shared_ptr<MarkerFactory> marker_factory_;

        rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        std::function<void(
            const arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr&)>
            wbc_state_callback_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr body_joint_target_subscriber_;

        // 目标汇聚：所有输入源都提交到 router_，由它决定谁在写、写什么、何时显示。
        std::unique_ptr<TargetRouter> router_;

        std::shared_ptr<interactive_markers::MenuHandler> left_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> right_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> head_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> body_menu_handler_;

        interactive_markers::MenuHandler::EntryHandle left_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle left_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle left_both_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_both_handle_{};
        interactive_markers::MenuHandler::EntryHandle head_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle head_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle body_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle body_toggle_handle_{};

        bool dual_arm_mode_;
        std::string control_base_frame_;
        std::string marker_fixed_frame_;
        std::string body_controller_name_{"body_joint_controller"};
        std::vector<double> last_body_joint_targets_;
        double publish_rate_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        // Relative tf/tf_static subs (not TransformListener): Isaac dual robots
        // publish /<ns>/tf; absolute /tf is empty under multi-robot namespaces.
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

        MarkerState current_mode_ = MarkerState::SINGLE_SHOT;

        mutable std::mutex state_update_mutex_;
        int32_t current_controller_state_ = 2;
        std::string current_fsm_state_ = "HOLD";
        std::vector<int32_t> disable_auto_update_states_;
        // FSM 统一入口。状态变化由它推送，本类不再自行订阅与推断。
        FsmClient* fsm_{nullptr};
        uint64_t fsm_observer_token_{0};
        // WBC 模式统一入口。状态变化由它推送，本类不再自行订阅。
        HumanoidModeClient* mode_client_{nullptr};
        uint64_t mode_state_observer_token_{0};

        int left_arm_state_{1};
        int right_arm_state_{1};
        int bimanual_state_{0};
        int body_state_{0};
        int head_state_{0};
        int base_state_{
            arms_ros2_control_msgs::msg::WbcCurrentState::BASE_LOCKED};
        bool wbc_head_marker_visible_{false};
        bool head_vr_active_{false};
        bool left_arm_vr_active_{false};
        bool right_arm_vr_active_{false};

        rclcpp::Time last_marker_update_time_;
        double marker_update_interval_;
        rclcpp::Time last_publish_time_;

        rclcpp::TimerBase::SharedPtr marker_update_timer_;
        std::atomic<bool> pending_changes_{false};

        std::shared_ptr<ArmMarker> left_arm_marker_;
        std::shared_ptr<ArmMarker> right_arm_marker_;
        std::shared_ptr<HeadMarker> head_marker_;
        std::shared_ptr<BodyMarker> body_marker_;
    };
} // namespace arms_ros2_control::command
