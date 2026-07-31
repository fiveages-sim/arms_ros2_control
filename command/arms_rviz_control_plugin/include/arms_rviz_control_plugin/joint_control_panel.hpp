#pragma once

#include <memory>
#include <QLabel>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QScrollArea>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <mutex>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <arms_controller_common/utils/JointLimitsManager.h>
#include <rclcpp/parameter_client.hpp>

namespace arms_rviz_control_plugin
{
    class JointControlPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        JointControlPanel(QWidget* parent = nullptr);
        ~JointControlPanel() override;

        void onInitialize() override;

        // Load and save configuration data
        void load(const rviz_common::Config& config) override;
        void save(rviz_common::Config config) const override;

    private Q_SLOTS:
        void onSendButtonClicked();
        void onCategoryChanged();
        void onWaistLiftingSliderChanged(int value);
        void onWaistTurningSliderChanged(int value);
        void onDisplayUnitChanged();
        void onPoseModeChanged();

        void onWaistUpPressed();
        void onWaistUpReleased();

        void onWaistDownPressed();
        void onWaistDownReleased();

        void onWaistLeftPressed();
        void onWaistLeftReleased();

        void onWaistRightPressed();
        void onWaistRightReleased();

        void onWaistRepeatTimeout();

    private:
        struct PoseXyzRpy
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double roll = 0.0;   // rad
            double pitch = 0.0;  // rad
            double yaw = 0.0;    // rad
        };

        void onFsmCommandReceived(const std_msgs::msg::Int32::SharedPtr msg);
        void onJointStateReceived(const sensor_msgs::msg::JointState::SharedPtr msg);
        void onLeftCurrentTargetReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void onRightCurrentTargetReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void updatePanelVisibility();
        void publishJointPositions();
        void updateJointValuesFromState();
        std::string classifyJoint(const std::string& joint_name);
        void updateJointVisibility();
        std::string getControllerNameForCategory(const std::string& category);
        std::string getWaistControllerName() const;
        void updatePublisher();
        void updateCategoryOptions();
        bool hasControllerForCategory(const std::string& category);
        bool shouldShowSendButton() const;
        void updateSpinboxRanges();
        bool refreshJointMetadataFromCache();
        void initializeJoints(const std::vector<std::string>& joint_names_source);
        void publishWaistLifting(double value);
        void publishWaistTurning(double value);

        // UI 单位成对：米+弧度 / 厘米+角度；内部与 ROS 始终为 m、rad
        // 末端位姿 XYZ/RPY 继续使用下列换算；普通关节使用类型感知接口
        double toDisplayAngle(double radians) const;
        double fromDisplayAngle(double display_value) const;
        double toDisplayLength(double meters) const;
        double fromDisplayLength(double display_value) const;
        bool isPrismaticJoint(size_t index) const;
        double toDisplayJointValue(size_t index, double value_si) const;
        double fromDisplayJointValue(size_t index, double display_value) const;
        QString jointUnitSuffix(size_t index) const;
        void setJointSpinboxFromSiValue(size_t index, double value_si);
        double getJointSpinboxSiValue(size_t index) const;
        void applyDisplayUnitToJointSpinboxes();
        bool isPoseTargetCommand() const;
        void applyDisplayUnitToPoseSpinboxes(
            std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes);

        // 绝对：xyz + 四元数（无 RPY 转换）；相对：xyz + RPY
        void setArmPoseSpinboxesFromPose(
            std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes,
            const geometry_msgs::msg::Pose& pose);
        geometry_msgs::msg::Pose getArmPoseFromSpinboxes(
            const std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes) const;
        void setArmRelativeSpinboxes(
            std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes,
            const PoseXyzRpy& xyz_rpy);
        PoseXyzRpy getArmRelativeSpinboxesRadians(
            const std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes) const;
        void configureArmPoseSpinbox(QDoubleSpinBox* spinbox, size_t index) const;
        void applyArmPoseUiMode(
            const std::string& side_prefix,
            std::vector<std::unique_ptr<QLabel>>& labels,
            std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes,
            std::vector<std::unique_ptr<QVBoxLayout>>& layouts);
        void zeroArmPoseSpinboxes(std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes);
        void updatePoseModeControlsVisibility();
        bool publishOcs2ArmPose(bool is_left);
        void refreshOcs2FrameParams();
        std::string getOcs2ControllerName() const;

        double getWaistLiftingScale() const;
        double getWaistTurningScale() const;
        void updateWaistScaleLabels();

        void updateWaistRepeatTimerState();
        void stopWaistLifting();
        void stopWaistTurning();

        void updateWaistControlsVisibility(bool visible);

        void onBodyCurrentTargetReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        void refreshWaistEnabledState();

        // 读取指定 UI 分类对应控制器的 joints 参数，用作该分类的显示/发送顺序。
        std::vector<std::string> getControllerJointOrderForCategory(const std::string& category) const;

        // 按控制器提供的关节顺序重排面板内部 joint_names_；
        // 没有控制器顺序的分类保持原来的 joint_states 顺序。
        std::vector<std::string> reorderJointsByControllerOrder(
            const std::vector<std::string>& old_order,
            const std::map<std::string, std::string>& joint_to_category,
            const std::map<std::string, std::vector<std::string>>& category_order) const;

        // ROS2
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_relative_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr right_relative_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_current_target_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_current_target_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_lifting_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_turning_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr body_current_target_subscriber_;

        // Frame IDs: base from current_target (or controller base_frame); EE from controller params
        std::string left_target_frame_id_;
        std::string right_target_frame_id_;
        std::string ocs2_base_frame_;
        std::string left_ee_frame_;
        std::string right_ee_frame_;
        std::mutex frame_id_mutex_;

        geometry_msgs::msg::Pose left_current_pose_;
        geometry_msgs::msg::Pose right_current_pose_;
        bool left_current_pose_valid_ = false;
        bool right_current_pose_valid_ = false;

        // Joint limits manager
        std::shared_ptr<arms_controller_common::JointLimitsManager> joint_limits_manager_;
        std::string robot_description_cache_;
        bool robot_description_received_ = false;
        bool joint_types_ready_ = false;
        // Non-empty when refresh failed for a specific joint; shown instead of generic wait text
        QString joint_metadata_status_;

        // UI Elements
        std::unique_ptr<QGroupBox> joint_control_group_;
        std::unique_ptr<QVBoxLayout> joint_layout_;
        std::unique_ptr<QScrollArea> scroll_area_;
        std::unique_ptr<QHBoxLayout> category_layout_;
        std::unique_ptr<QComboBox> category_combo_;
        std::unique_ptr<QComboBox> display_unit_combo_;
        // OCS2 位姿类型 / 发送后保持：放在发送按钮上方
        std::unique_ptr<QVBoxLayout> ocs2_pose_send_layout_;
        std::unique_ptr<QHBoxLayout> pose_mode_row_layout_;
        std::unique_ptr<QLabel> pose_mode_label_;
        std::unique_ptr<QComboBox> pose_mode_combo_;
        std::unique_ptr<QCheckBox> relative_keep_input_checkbox_;
        std::unique_ptr<QLabel> status_label_;
        std::unique_ptr<QPushButton> send_button_;
        std::vector<std::unique_ptr<QVBoxLayout>> joint_row_layouts_;
        std::vector<std::unique_ptr<QLabel>> joint_labels_;
        std::vector<std::unique_ptr<QDoubleSpinBox>> joint_spinboxes_;

        // OCS2 末端：绝对 xyz+qx/qy/qz/qw（7）；相对复用前 6 为 xyz+rpy，隐藏 qw
        std::vector<std::unique_ptr<QVBoxLayout>> left_arm_row_layouts_;
        std::vector<std::unique_ptr<QLabel>> left_arm_labels_;
        std::vector<std::unique_ptr<QDoubleSpinBox>> left_arm_spinboxes_;
        std::vector<std::unique_ptr<QVBoxLayout>> right_arm_row_layouts_;
        std::vector<std::unique_ptr<QLabel>> right_arm_labels_;
        std::vector<std::unique_ptr<QDoubleSpinBox>> right_arm_spinboxes_;

        // Joint information
        std::vector<std::string> joint_names_;
        std::vector<double> joint_positions_;
        std::map<std::string, size_t> joint_name_to_index_;
        std::map<std::string, std::string> joint_to_category_; // joint_name -> category
        std::map<std::string, std::vector<size_t>> category_to_joints_; // category -> joint indices
        bool joints_initialized_ = false;
        // 无 left/right 前缀的单臂：归到 left，走 left_target
        bool single_arm_mode_ = false;
        // true = 厘米+角度；false = 米+弧度
        bool use_cm_deg_ = false;
        // absolute | relative_base | relative_ee
        std::string pose_mode_ = "absolute";
        bool use_relative_pose_ = false;  // true if pose_mode_ starts with relative_
        bool relative_keep_input_ = false;  // false = clear after send (default)
        // 避免在 /joint_states 高频回调中反复 wait_for_service 卡住 RViz
        std::chrono::steady_clock::time_point last_body_joint_order_attempt_{};
        std::string current_category_ = "all";

        // Control state
        int32_t current_command_ = 2;
        bool is_joint_control_enabled_ = false;
        std::string current_state_ = "HOLD"; // Track current FSM state for transition validation

        // Available controllers
        std::vector<std::string> available_controllers_;
        std::set<std::string> available_categories_;
        std::map<std::string, std::string> category_to_controller_; // category -> controller name
        
        // Waist control
        std::unique_ptr<QVBoxLayout> waist_control_layout_;

        std::unique_ptr<QVBoxLayout> waist_lifting_layout_;
        std::unique_ptr<QHBoxLayout> waist_lifting_slider_layout_;
        std::unique_ptr<QLabel> waist_lifting_label_;
        std::unique_ptr<QSlider> waist_lifting_slider_;
        std::unique_ptr<QLabel> waist_lifting_value_label_;

        std::unique_ptr<QVBoxLayout> waist_turning_layout_;
        std::unique_ptr<QHBoxLayout> waist_turning_slider_layout_;
        std::unique_ptr<QLabel> waist_turning_label_;
        std::unique_ptr<QSlider> waist_turning_slider_;
        std::unique_ptr<QLabel> waist_turning_value_label_;

        std::unique_ptr<QHBoxLayout> waist_button_layout_top_;
        std::unique_ptr<QHBoxLayout> waist_button_layout_bottom_;

        std::unique_ptr<QPushButton> waist_up_button_;
        std::unique_ptr<QPushButton> waist_down_button_;
        std::unique_ptr<QPushButton> waist_left_button_;
        std::unique_ptr<QPushButton> waist_right_button_;

        std::unique_ptr<QTimer> waist_repeat_timer_;

        std::unique_ptr<QWidget> scroll_content_widget_;
        std::unique_ptr<QVBoxLayout> scroll_content_layout_;
        std::unique_ptr<QGroupBox> waist_group_box_;

        bool waist_up_pressed_ = false;
        bool waist_down_pressed_ = false;
        bool waist_left_pressed_ = false;
        bool waist_right_pressed_ = false;
        bool is_waist_enabled_ = false;
        bool waist_enabled_checked_ = false;
        bool shouldShowWaistControls() const;
    };
} // namespace arms_rviz_control_plugin
