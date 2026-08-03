//
// Created for Arms ROS2 Control - VRInputHandler
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <optional>
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_controller_common/utils/FSMCommandPublisher.h"

namespace arms_ros2_control::command
{
    /**
     * VRInputHandler - VR输入处理器Wrapper
     * 
     * 基于VRMarkerWrapper功能，适配arms_target_manager
     * 实现VR pose订阅、状态切换机制和marker更新功能
     * 
     * 功能特性:
     * - 支持存储模式和更新模式切换
     * - 基于left_thumbstick的状态切换
     * - VR pose变化检测和频闪优化
     * - 与ArmsTargetManager的集成
     */
    class VRInputHandler
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param targetManager ArmsTargetManager指针
         * @param pub_left_target 外部传入的左臂目标位姿发布器（统一管理）
         * @param pub_right_target 外部传入的右臂目标位姿发布器（统一管理）
         * @param handControllers 手部/夹爪控制器名称列表（用于映射夹爪命令）
         * @param vr_thumbstick_linear_scale VR摇杆线性缩放因子（单位 m/step）
         * @param vr_thumbstick_angular_scale VR摇杆角度缩放因子（单位 rad/step）
         * @param vr_pose_scale 手柄位姿位置缩放因子（左右共用，可由组合键动态校准）
         * @param reference_link 参考link名称（例如 head_link2），用于VR/头显关联
         * @param vr_follow_frame FULL_BODY 下 VR 末端目标计算坐标系（默认 base_footprint）
         */
        VRInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
            const std::vector<std::string>& handControllers,
            double vr_thumbstick_linear_scale,
            double vr_thumbstick_angular_scale,
            double vr_pose_scale,
            const std::string& reference_link,
            const std::string& vr_follow_frame);

        ~VRInputHandler() = default;

        /**
         * 启用VR控制
         */
        void enable();

        /**
         * 禁用VR控制
         */
        void disable();

        /**
         * 检查VR控制是否启用
         * @return true如果启用，false否则
         */
        bool isEnabled() const { return enabled_.load(); }

        /**
         * 检查是否处于镜像模式
         * @return true如果启用镜像模式，false否则
         */
        bool isMirrorMode() const { return mirror_mode_.load(); }

        enum class ControlTopology
        {
            UNKNOWN,
            SPLIT_BODY,
            FULL_BODY,
        };

        ControlTopology controlTopology() const noexcept
        {
            return control_topology_.load();
        }

        bool isSplitBodyMode() const noexcept
        {
            return controlTopology() == ControlTopology::SPLIT_BODY;
        }

        bool isFullBodyMode() const noexcept
        {
            return controlTopology() == ControlTopology::FULL_BODY;
        }

        /**
         * 检查节点是否存在
         * @param node ROS节点指针
         * @param targetNodeName 目标节点名称
         * @return true如果节点存在，false否则
         */
        bool checkNodeExists(const std::shared_ptr<rclcpp::Node>& node, const std::string& targetNodeName);

        /**
         * FSM命令回调函数（用于跟踪FSM状态）
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg FSM命令消息
         */
        void fsmCommandCallback(std_msgs::msg::Int32::SharedPtr msg);

        /**
         * 机器人左臂当前pose回调函数
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg 机器人pose消息
         */
        void robotLeftPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 机器人右臂当前pose回调函数
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg 机器人pose消息
         */
        void robotRightPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void wbcStateCallback(
            arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr msg);

    private:
        enum class WbcToggleTarget : uint8_t
        {
            NONE = 0,
            BASE,
            BIMANUAL,
            LEFT_ARM,
            RIGHT_ARM
        };

        /**
         * VR左臂pose回调函数
         * @param msg VR pose消息
         */
        void vrLeftCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        
        /**
         * VR右臂pose回调函数
         * @param msg VR pose消息
         */
        void vrRightCallback(geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * VR头显pose回调函数
         * @param msg VR 头显 pose 消息
         */
        void vrHeadCallback(geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * 摇杆轴值回调函数（合并处理左右摇杆）
         * @param msg Twist消息，linear.x/y 表示左摇杆，angular.x/y 表示右摇杆
         */
        void thumbstickAxesCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        /**
         * 扳机模拟量回调函数（合并处理左右扳机）
         * @param msg Twist消息，linear.x 表示左扳机拉动比例，angular.x 表示右扳机拉动比例
         */
        void triggerValuesCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        /**
         * 处理左摇杆轴值（内部辅助函数）
         */
        void processLeftThumbstickAxes();
        
        /**
         * 处理右摇杆轴值（内部辅助函数）
         */
        void processRightThumbstickAxes();

        /**
         * 处理 BODY_TRACKING 下的左摇杆轴值，把增量累加到腰部 marker
         * XY 模式：Y→X, X→Y；Z+Yaw 模式：Y→Z, X→Yaw
         */
        void processBodyThumbstickAxes();

        /**
         * WBC 是否已确认腰部跟随（FSM=OCS2 且 body_state=BODY_TRACKING）
         */
        bool isBodyTrackingActive() const;

        /**
         * 更新身体模式请求的 pending 状态
         * @return true 表示请求仍在 pending，左摇杆此时不得驱动腰部或左臂
         */
        bool updateBodyModeRequestState();

        /**
         * 统一处理 case 21–24 身体模式请求
         */
        void requestBodyMode(
            int32_t eventCase,
            const char* direction,
            const char* modeName,
            const char* command,
            int expectedBodyState);

        bool readWbcToggleState(
            WbcToggleTarget target,
            bool& enabled) const;

        void requestWbcToggle(
            int32_t eventCase,
            const char* sourceDescription,
            WbcToggleTarget target);

        bool updateRightWbcToggleRequestState();

        void updateArmWbcVrState(
            WbcToggleTarget arm,
            int armState);
        bool isArmVrInputSuppressed(WbcToggleTarget arm) const;
        bool prepareArmVrInput(WbcToggleTarget arm);
        bool rebaseArmVrControlFromCurrentPose(WbcToggleTarget arm);
        bool setRobotBaseFromCurrentPose(const std::string& armType);
        void clearLastPublishedTarget(const std::string& armType);

        /**
         * 发布 WBC 模式切换命令
         * @param command 模式命令字符串，例如 "BODY_TRACKING"
         */
        void publishHumanoidModeCommand(const std::string& command);

        /**
         * 处理底盘模式下的摇杆轴值
         * 默认：左摇杆→底盘平移, 右摇杆X→底盘转向(angular.z), 右摇杆Y忽略
         * 非 FULL_BODY 下按住右握把可作为腰部控制修饰符；
         * FULL_BODY 下右握把方向由 WBC 四开关组合消费，右轴会在进入这里前清零。
         */
        void processChassisAxes();

        /**
         * 切换底盘控制模式（case 20 触发）
         * 进入：清零并停止末端摇杆累积；退出：清零底盘与腰部命令
         */
        void toggleChassisMode();

        /**
         * 把 /cmd_vel、waist_lifting_command、waist_turning_command 三个话题各发一次 0
         * 用于退出底盘模式、disable、enabled 变 false 等场景，防止残留运动
         */
        void resetChassisAndWaistCommands();

        /**
         * 按钮事件回调函数（处理按钮事件数字）
         * @param msg Int32消息，包含按钮事件数字 (0=无事件, 1-6=按钮按下, 7=镜像模式切换, 9=左扳机, 10=右扳机)
         */
        void processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg);

        /**
         * 从hand_controllers参数中提取左右控制器名称
         * @param hand_controllers 控制器名称列表
         */
        void detectGripperControllers(const std::vector<std::string>& hand_controllers);

        /**
         * 发布夹爪命令到对应的控制器
         * @param controller_name 控制器名称
         * @param command 命令值 (0=close, 1=open)
         */
        void publishGripperCommand(const std::string& controller_name, int32_t command);

        /**
         * 发布夹爪/灵巧手百分比目标到对应的控制器
         * @param controller_name 控制器名称
         * @param percent 百分比目标 (0.0=完全闭合, 1.0=完全张开)
         */
        void publishGripperPercent(const std::string& controller_name, double percent);

        /**
         * 模式切换后先把左右手同步到张开状态，避免旧模式目标和新模式目标冲突
         * @param percent_mode true=用target_percent张开, false=用target_command张开
         */
        void openGrippersAfterModeSwitch(bool percent_mode);

        /**
         * 按镜像模式解析左右扳机对应的手控制器
         * @param is_left_trigger true=左扳机, false=右扳机
         * @param controller_name 输出控制器名称
         * @param is_target_left_arm 输出是否对应左手/左臂
         */
        void resolveTriggerTarget(bool is_left_trigger, std::string& controller_name, bool& is_target_left_arm) const;

        /**
         * 处理旧式扳机开关控制：每按一次在开/关之间切换
         * @param is_left_trigger true=左扳机, false=右扳机
         */
        void toggleGripperByTrigger(bool is_left_trigger);

        /**
         * 左夹爪状态回调函数（同步夹爪状态）
         * @param msg 夹爪命令消息 (0=close, 1=open)
         */
        void leftGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

        /**
         * 右夹爪状态回调函数（同步夹爪状态）
         * @param msg 夹爪命令消息 (0=close, 1=open)
         */
        void rightGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

        /**
         * 发送FSM状态切换命令
         * @param command FSM命令值 (1=HOME, 2=HOLD, 3=OCS2, 100=切换姿态(REST), 0=重置)
         */
        void sendFsmCommand(int32_t command);

        /**
         * 将计算坐标系下的目标转换到控制发布 frame，再做变化检测并发布到 left_target/right_target。
         * FULL_BODY 下 position/orientation 为 vr_follow_frame_ Pose；非 FULL_BODY 保持旧语义。
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 计算坐标系下的位置
         * @param orientation 计算坐标系下的方向
         * @return true 表示本次请求真正发布；false 表示被耦合拦截、首次解耦重建、TF 失败、未变化或发布器不可用
         */
        bool publishTargetPoseDirect(const std::string& armType,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation);

        /** 读取 ArmsTargetManager 中 WBC 已确认的双臂耦合状态。 */
        bool isBimanualCoupled() const;

        /** 解耦后首次恢复前重建右臂 VR 控制基准，不发布 ROS 目标。 */
        void rebaseRightArmVrControl();

        /**
         * 在 source/target frame 之间转换 Pose；失败时不写输出。
         */
        bool transformPoseBetweenFrames(
            const std::string& armType,
            const Eigen::Vector3d& inputPosition,
            const Eigen::Quaterniond& inputOrientation,
            const std::string& sourceFrame,
            const std::string& targetFrame,
            Eigen::Vector3d& outputPosition,
            Eigen::Quaterniond& outputOrientation);

        /**
         * 确认基准有效后计算相对目标，再交给集中发布函数。
         * 基准因 TF 缺失未就绪时会自动重试建立。
         */
        bool calculateAndPublishTarget(
            const std::string& armType,
            const Eigen::Vector3d& vrCurrentPosition,
            const Eigen::Quaterniond& vrCurrentOrientation,
            const Eigen::Vector3d& vrBasePosition,
            const Eigen::Quaterniond& vrBaseOrientation,
            Eigen::Vector3d& calculatedPosition,
            Eigen::Quaterniond& calculatedOrientation);

        /**
         * 记录最后一次实际发布出去的目标位姿，用作暂停恢复/进入 UPDATE 的 command 连续性基准。
         * FULL_BODY 下保存的是 vr_follow_frame_ Pose。
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 计算坐标系下的位置
         * @param orientation 计算坐标系下的方向
         */
        void recordLastPublishedTarget(const std::string& armType,
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation);

        /**
         * 清空上一次 OCS2 会话发布的 command target 缓存，并使左右臂基准失效。
         */
        void clearLastPublishedTargets();

        /**
         * 将对应手臂的 robot base 设置为最后 command target；若还没有发布历史，则回退到 current pose。
         * FULL_BODY 下 current pose 会从 ee_frame_id_ 转到 vr_follow_frame_；失败时不覆盖基准。
         * @param armType 手臂类型 ("left" 或 "right")
         * @return true 表示基准已在正确计算 frame 中建立
         */
        bool setRobotBaseFromLastCommandOrCurrent(const std::string& armType);

        /**
         * 将Pose消息转换为Eigen::Matrix4d
         * @param msg Pose消息
         * @return 4x4变换矩阵
         */
        Eigen::Matrix4d poseMsgToMatrix(geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * 从4x4矩阵提取位置和方向
         * @param matrix 4x4变换矩阵
         * @param position 输出位置
         * @param orientation 输出方向
         */
        void matrixToPosOri(const Eigen::Matrix4d& matrix,
                            Eigen::Vector3d& position,
                            Eigen::Quaterniond& orientation);

        /**
         * 检查pose是否发生显著变化
         * @param currentPos 当前位置
         * @param currentOri 当前方向
         * @param prevPos 之前位置
         * @param prevOri 之前方向
         * @return true如果pose发生显著变化
         */
        bool hasPoseChanged(const Eigen::Vector3d& currentPos,
                            const Eigen::Quaterniond& currentOri,
                            const Eigen::Vector3d& prevPos,
                            const Eigen::Quaterniond& prevOri);

        /**
         * 基于差值计算pose并应用到机器人base pose
         * @param vrCurrentPos 当前VR位置
         * @param vrCurrentOri 当前VR方向
         * @param vrBasePos VR base位置
         * @param vrBaseOri VR base方向
         * @param robotBasePos 机器人base位置
         * @param robotBaseOri 机器人base方向
         * @param resultPos 输出计算的位置
         * @param resultOri 输出计算的方向
         */
        void calculatePoseFromDifference(const Eigen::Vector3d& vrCurrentPos,
                                         const Eigen::Quaterniond& vrCurrentOri,
                                         const Eigen::Vector3d& vrBasePos,
                                         const Eigen::Quaterniond& vrBaseOri,
                                         const Eigen::Vector3d& robotBasePos,
                                         const Eigen::Quaterniond& robotBaseOri,
                                         Eigen::Vector3d& resultPos,
                                         Eigen::Quaterniond& resultOri);

        using ListControllers = controller_manager_msgs::srv::ListControllers;

        void detectControlTopology();

        rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
        rclcpp::TimerBase::SharedPtr control_topology_timer_;
        std::atomic<ControlTopology> control_topology_{ControlTopology::UNKNOWN};
        std::optional<rclcpp::Client<ListControllers>::FutureAndRequestId>
            pending_topology_request_;
        std::chrono::steady_clock::time_point topology_request_deadline_{};

        // ROS组件
        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        // 发布器（用于直接发布到left_target/right_target）
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dual_target_stamped_;
        // FSM命令发布器（使用通用工具类，自动处理command=100的特殊情况）
        std::unique_ptr<arms_controller_common::FSMCommandPublisher> fsm_command_publisher_;

        // 订阅器
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_left_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_right_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_head_;
        // 按钮事件订阅器（Int32类型）
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_controller_state_;
        // 摇杆轴值订阅器（合并订阅左右摇杆，使用 Twist 消息）
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_thumbstick_axes_;
        // 扳机模拟量订阅器（合并订阅左右扳机，使用 Twist 消息）
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_trigger_values_;
        // 夹爪状态订阅器（用于同步夹爪状态）
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_left_gripper_state_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_right_gripper_state_;
        // 机器人 current_pose 订阅已移除，改为在 arms_target_manager_node 中统一处理
        // FSM命令订阅已移除，改为在 arms_target_manager_node 中统一处理

        // 底盘控制模式相关发布器（case 20 触发的底盘/腰部控制）
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_waist_lifting_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_waist_turning_;

        // WBC 模式切换命令发布器（case 21–24 请求身体模式）
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_command_;

        // VR pose参数
        Eigen::Matrix4d left_ee_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_ee_pose_ = Eigen::Matrix4d::Identity();

        // VR位置和方向参数（世界/机器人坐标系下）
        // *_position_raw_ 表示未应用缩放系数的原始VR位姿
        Eigen::Vector3d vr_left_position_raw_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d vr_right_position_raw_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d left_position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond right_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d vr_head_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_head_orientation_ = Eigen::Quaterniond::Identity();

        // 最后实际发布 frame（FULL_BODY 下为 ee_frame_id_）的 Pose，仅用于变化检测
        Eigen::Vector3d prev_calculated_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prev_calculated_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_right_orientation_ = Eigen::Quaterniond::Identity();

        // 最后一次实际发布对应的计算-frame command target：
        // FULL_BODY 下为 vr_follow_frame_ Pose；非 FULL_BODY 保持旧语义。
        bool has_last_published_left_target_ = false;
        Eigen::Vector3d last_published_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_published_left_orientation_ = Eigen::Quaterniond::Identity();
        bool has_last_published_right_target_ = false;
        Eigen::Vector3d last_published_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_published_right_orientation_ = Eigen::Quaterniond::Identity();

        // 状态管理
        std::atomic<bool> enabled_;
        std::atomic<bool> is_update_mode_; // true = 更新模式, false = 存储模式
        std::atomic<bool> mirror_mode_; // true = 镜像模式, false = 正常模式
        std::atomic<bool> left_arm_paused_; // 左臂是否暂停更新（Y按键控制）
        std::atomic<bool> right_arm_paused_; // 右臂是否暂停更新（B按键控制）
        std::atomic<bool> left_grip_mode_; // 左摇杆控制模式：false=XY平移, true=Z轴+Yaw
        std::atomic<bool> right_grip_mode_; // 右摇杆控制模式：false=XY平移, true=Z轴+Yaw
        std::atomic<bool> chassis_mode_; // true = 底盘控制模式, false = 末端控制模式（case 20 切换）
        std::atomic<bool> left_grip_active_{false};  // 左握把实时状态（预留，当前 chassis 逻辑未使用）
        std::atomic<bool> right_grip_active_{false}; // 右握把实时状态（chassis 模式下用作腰部控制修饰符）
        // FULL_BODY 下左握把方向组合的安全锁存：
        // 松开握把后仍保持抑制，直到左摇杆 X/Y 都回到 0.3 内。
        std::atomic<bool> left_grip_direction_suppressed_{false};
        // FULL_BODY 下右握把方向组合的安全锁存，语义与左侧对称。
        std::atomic<bool> right_grip_direction_suppressed_{false};

        std::atomic<bool> right_wbc_toggle_request_pending_{false};
        std::atomic<WbcToggleTarget> requested_wbc_toggle_target_{
            WbcToggleTarget::NONE};
        std::atomic<bool> requested_wbc_toggle_enabled_{false};
        std::chrono::steady_clock::time_point right_wbc_toggle_request_time_{};
        static constexpr auto right_wbc_toggle_request_timeout_ =
            std::chrono::seconds(2);

        // FULL_BODY 单臂 WBC 禁用：按实际机械臂抑制 VR 输入，并在 ARM_ENABLED 后
        // 等待 state-based rebase。不替代 SPLIT_BODY 的 *_arm_paused_，也不改变
        // right_wbc_toggle_request_pending_ 的互斥/超时/回中语义。
        std::atomic<int> observed_left_wbc_arm_state_{-1};
        std::atomic<int> observed_right_wbc_arm_state_{-1};
        std::atomic<bool> left_wbc_vr_suppressed_{false};
        std::atomic<bool> right_wbc_vr_suppressed_{false};
        std::atomic<bool> left_wbc_rebase_pending_{false};
        std::atomic<bool> right_wbc_rebase_pending_{false};

        std::atomic<bool> has_robot_current_left_pose_{false};
        std::atomic<bool> has_robot_current_right_pose_{false};
        std::atomic<bool> has_vr_left_pose_{false};
        std::atomic<bool> has_vr_right_pose_{false};

        // 双臂耦合期间右臂 VR 目标被抑制后，解耦后的下一次右臂发布前需重建控制基准。
        bool right_vr_target_suppressed_by_bimanual_{false};

        std::atomic<int32_t> current_fsm_state_; // 当前FSM状态：1=HOME, 2=HOLD, 3=OCS2, 100=REST

        // 腰部摇杆控制平面：false=XY平移, true=Z轴+Yaw（与左右臂的 *_grip_mode_ 相互独立）
        std::atomic<bool> body_thumbstick_z_yaw_mode_{false};
        // 身体模式命令发出后到 WBC 确认且左摇杆回中之间的异步窗口。
        std::atomic<bool> body_mode_request_pending_{false};
        std::atomic<int> requested_body_state_{-1};
        std::chrono::steady_clock::time_point body_mode_request_time_{};
        static constexpr auto body_mode_request_timeout_ = std::chrono::seconds(2);

        // VR base poses（摇杆按下时存储）
        Eigen::Vector3d vr_base_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_base_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d vr_base_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_base_right_orientation_ = Eigen::Quaterniond::Identity();

        // 机器人末端基准：FULL_BODY 下为 vr_follow_frame_ 中锁存的 Pose；非全身控制保持旧语义
        Eigen::Vector3d robot_base_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_base_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_base_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_base_right_orientation_ = Eigen::Quaterniond::Identity();
        // 当前锁存末端基准是否已在本模式要求的计算 frame 中建立
        bool left_robot_base_valid_ = false;
        bool right_robot_base_valid_ = false;

        // 当前机器人poses（仍为 current-pose 消息中的 ee_frame_id_ Pose）
        Eigen::Vector3d robot_current_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_current_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_current_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_current_right_orientation_ = Eigen::Quaterniond::Identity();

        // 摇杆轴值（归一化 -1.0 ~ 1.0）
        Eigen::Vector2d left_thumbstick_axes_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d right_thumbstick_axes_ = Eigen::Vector2d::Zero();

        // 未经 left_grip_direction_suppressed_ 清零的原始左摇杆轴值，
        // 供 pending 独立判断物理摇杆是否真的回中。
        Eigen::Vector2d left_thumbstick_axes_raw_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d right_thumbstick_axes_raw_ =
            Eigen::Vector2d::Zero();

        // 摇杆累积偏移量（米）
        Eigen::Vector3d left_thumbstick_offset_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d right_thumbstick_offset_ = Eigen::Vector3d::Zero();

        // 摇杆累积Yaw旋转（弧度）
        double left_thumbstick_yaw_offset_ = 0.0;
        double right_thumbstick_yaw_offset_ = 0.0;

        // 暂停时刻的VR位姿（用于暂停时继续使用摇杆控制）
        Eigen::Vector3d paused_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond paused_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d paused_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond paused_right_orientation_ = Eigen::Quaterniond::Identity();

        // Hand controllers mapping（类似ControlInputHandler）
        std::vector<std::string> hand_controllers_;
        // Publishers for gripper commands (created on demand)
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> gripper_command_publishers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gripper_percent_publishers_;
        std::map<std::string, double> last_gripper_percent_;
        // 检测到的左右控制器名称
        std::string left_gripper_controller_name_;
        std::string right_gripper_controller_name_;
        // 夹爪状态跟踪
        std::atomic<bool> left_gripper_open_;
        std::atomic<bool> right_gripper_open_;
        std::atomic<bool> trigger_percent_mode_;
        std::chrono::steady_clock::time_point last_trigger_mode_toggle_time_{};
        std::chrono::steady_clock::time_point trigger_percent_resume_time_{};

        // VR控制缩放参数（从配置文件读取 / 固定配置）
        double vr_thumbstick_linear_scale_; // 摇杆位置缩放因子（单位 m/step）
        double vr_thumbstick_angular_scale_; // 摇杆旋转缩放因子（单位 rad/step）
        double vr_pose_scale_;  // 手柄位姿位置缩放因子（左右共用，可由组合键动态校准）

        // 底盘/腰部控制硬编码速度比例（后续可参数化，参考 joystick_teleop.cpp）
        static constexpr double chassis_linear_scale_ = 0.25;   // 底盘线速度比例
        static constexpr double chassis_angular_scale_ = 0.5;   // 底盘角速度比例
        static constexpr double waist_lifting_scale_ = 1.0;     // 腰部升降 factor（控制器内部 clamp 到 [-1,1]，>1 无意义）
        static constexpr double waist_turning_scale_ = 1.0;     // 腰部旋转 factor（同上）

        // 参考link名称（用于将VR头显/手柄关联到机器人某个link），从target_manager.yaml读取，默认"base_link"
        std::string reference_link_;

        // FULL_BODY VR 目标的计算坐标系；由 vr_follow_frame 参数配置
        std::string vr_follow_frame_;

        // 来自 left_current_pose/right_current_pose.header.frame_id；
        // 普通 left_target/right_target 在控制器侧采用该隐式坐标系。
        std::string ee_frame_id_;
        bool ee_frame_id_initialized_ = false;

        // TF组件（用于在不同link/frame之间转换）
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // 常量
        static const std::string XR_NODE_NAME;
        static const double POSITION_THRESHOLD;
        static const double ORIENTATION_THRESHOLD;
    };
} // namespace arms_ros2_control::command
