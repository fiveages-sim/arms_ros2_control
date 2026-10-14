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
#include <std_msgs/msg/bool.hpp>
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
#include <arms_ros2_control_msgs/msg/wbc_capability.hpp>

namespace arms_ros2_control::command
{
    /**
     * 全身骨架绝对位姿映射与底盘跟随的可调参数。
     *
     * 这些开关只在「全身遥操作模式」（case 30 已开启）内部生效：
     *   - 短按右摇杆 = 开/关「全部跟踪」→ 胸/头/双臂末端改为 /teleop/body_* 的绝对位姿映射
     *   - 长按右摇杆 = 开/关「底盘跟踪」→ 骨盆水平位姿经位置外环转成 /cmd_vel
     * 非全身模式下的一切既有行为（UPDATE/STORAGE、摇杆微调、镜像、case 20 底盘模式等）不受影响。
     */
    struct AbsoluteTeleopOptions
    {
        // 双臂末端数据源：hand / wrist（对应 /teleop/body_left_hand_pose 等话题）
        std::string ee_source{"hand"};

        // 「数据源坐标系 → 机器人目标坐标系」的固定旋转偏置（在手部体坐标系里右乘），
        // 固定轴 RPY（度，R = Rz(yaw)·Ry(pitch)·Rx(roll)）。
        // 手柄面朝上时：人手 x 轴上、左手 z 轴左 / 右手 z 轴右；而 tcp x 轴上、z 轴前，
        // 因此需要绕手部 x 轴 ∓90°。 换 ee_source=wrist / 换机型（eeFrame=\_eef 等）时必须重调。
        std::vector<double> hand_to_ee_rpy_deg_left{-90.0, 0.0, 0.0};
        std::vector<double> hand_to_ee_rpy_deg_right{90.0, 0.0, 0.0};
        // 胸部（数据源 spine3 → body_link4）与头部（数据源 body_head → head_link2）的同名偏置
        std::vector<double> body_src_to_ee_rpy_deg{0.0, 0.0, 0.0};
        std::vector<double> head_src_to_ee_rpy_deg{0.0, 0.0, 0.0};
        // ===== 骨架分段尺度（在人体根 H 里对「段向量」缩放；只改位置，姿态完全不变）=====
        // 1.0 = 与人体等比；>1 表示机器人的对应尺寸比人体大。
        // 等价于沿链逐段缩放后重新累加（段向量之和 = 链端点之差）：
        //   s_body     : pelvis → spine3（并同样作用于 spine3 → head）
        //   s_shoulder : spine3 → shoulder(L/R)
        //   s_arm      : shoulder → 末端(hand/wrist)
        double body_length_scale{1.0};
        double shoulder_width_scale{1.0};
        double arm_length_scale{1.0};
        // VR 世界系 → 机器人世界系的对齐偏航（度）。
        // 标准摆放（人站在机器人正后方、面朝机器人前方启动头显）下为 0。
        // 注：早期有一个 absolute_map_yaw_deg（人体根轴向 → 底座轴向的固定偏航），
        // 已删除。映射现在按「人体系」锚定：人体朝向即机器人机头朝向，
        // 该关系由「把人体各量先表达在人体根 H 自身坐标系里」自动建立，无需人工常数。
        // 绝对目标的一阶低通系数（0=不滤波，越大越跟随）
        double filter_alpha{0.3};
        // 标定前等待姿态稳定的时长 [s]
        double capture_settle_sec{0.2};
        // 骨架单关节数据看门狗 [s]；超时只冻结对应任务
        double skeleton_timeout_sec{0.2};
        // 人体根 H 的位置（xy + z）估计的低通系数。
        // H 由双足得到（xy 取双足均值、z 取双足较低者）；行走时双足交替会让均值抖动，
        // 因此需要低通抑制，避免抖动直接进入各部位目标。
        double human_root_filter_alpha{0.1};
        // 人体根原点的固定偏置 [m]（三轴），在**人体根自身坐标系**里施加：
        //   x 前 / y 左 / z 上（与 base_footprint 同向）。
        // 人体根已经由踝（xy/偏航）+ 足（z）算出，一般不需额外偏置。
        std::vector<double> human_root_offset{0.0, 0.0, 0.0};
        // 胸部位置沿「脊柱轴」的锚点比例：0 = 取 spine3 位置，1 = 取 pelvis 位置。
        // 机器人的 body_link4 比人体 spine3 偏下、大致在骨盆高度，所以默认取 1。
        double body_anchor_lambda{1.0};
        // 骨架几何一致性容差（臂长偏离比例）
        double geometry_tolerance{0.2};

        // ---- 底盘跟踪（位置外环 → /cmd_vel） ----
        bool base_follow_enable{true};
        double base_follow_rate_hz{50.0};
        double base_follow_tau{2.0};
        double base_follow_tau_yaw{2.0};
        double base_follow_kp{0.8};
        double base_follow_kpsi{1.0};
        double base_follow_deadband{0.02};
        double base_follow_yaw_deadband{0.05};
        double base_vmax{0.4};
        double base_wmax{0.8};
        double base_acc_max{1.0};
        double base_alpha_max{2.0};
        double base_cmd_tau{0.2};
        double base_follow_data_timeout{0.2};
    };

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
         * @param vr_pose_scale 手柄位姿位置缩放因子（左右共用；事件 15 左Y+右B 固定按左侧 Z 距校准）
         * @param reference_link 参考link名称（例如 head_link2），用于VR/头显关联
         * @param vr_follow_frame VR 增量参考轴系，恢复时冻结到控制器系的旋转（默认 base_footprint）
         * @param absolute_options 全身骨架绝对映射 / 底盘跟随参数
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
            const std::string& vr_follow_frame,
            const AbsoluteTeleopOptions& absolute_options = AbsoluteTeleopOptions{});

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

        enum class MirrorModeProfile : uint8_t
        {
            MODE_A = 0,
            MODE_B = 1,
        };

        /**
         * 检查是否处于镜像模式
         * @return true如果启用镜像模式，false否则
         */
        bool isMirrorMode() const { return mirror_mode_.load(); }

        bool isMirrorModeProfileA() const
        {
            return mirror_mode_.load() &&
                   mirror_mode_profile_.load() == static_cast<int>(MirrorModeProfile::MODE_A);
        }

        bool isMirrorModeProfileB() const
        {
            return mirror_mode_.load() &&
                   mirror_mode_profile_.load() == static_cast<int>(MirrorModeProfile::MODE_B);
        }

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
         * 检查全身骨架遥操作是否已开启（case 30 组合键开关）。
         */
        bool isWholeBodyTeleopActive() const
        {
            return whole_body_teleop_active_.load();
        }

        /**
         * 「全部跟踪」是否已开启（全身模式内短按右摇杆 toggle）。
         * 开启后胸 / 头 / 双臂末端目标改为 /teleop/body_* 的绝对位姿映射。
         */
        bool isAbsoluteTrackingActive() const
        {
            return absolute_tracking_active_.load();
        }

        /**
         * 「底盘跟踪」是否已开启（全身模式内长按右摇杆 toggle）。
         */
        bool isBaseTrackingEnabled() const
        {
            return base_tracking_enabled_.load();
        }

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
            RIGHT_ARM,
            HOME_JOINT
        };

        void wbcCapabilityCallback(
            arms_ros2_control_msgs::msg::WbcCapability::ConstSharedPtr msg);

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
        void robotHeadPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        bool headTrackingReady() const;
        void stopHeadTracking();
        void publishHeadTarget();

        // ---- 全身遥操（case 30）：由 /teleop/body_* 骨架数据驱动身体与头部目标 ----
        // 注意：该模式内**只有绝对映射**（按下右摇杆开跟踪后直接采当前位姿），
        // 不存在「按下时记录基准 + 跟踪相对变化量」的相对模式。

        /** 骨架关节回调：只存储缓存，脊柱触发身体目标计算，头部触发头部目标计算 */
        void bodyPelvisCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        void bodySpine3Callback(geometry_msgs::msg::Pose::SharedPtr msg);
        void bodyHeadCallback(geometry_msgs::msg::Pose::SharedPtr msg);

        /** 全身遥操作前置条件：VR enabled + FULL_BODY + FSM=OCS2 + WBC 已确认 BODY_TRACKING + 骨架数据可用 */
        bool wholeBodyTeleopReady() const;

        /** 由骨架数据计算并下发胸部绝对目标（挂在 BodyMarker 上发布 body_target） */
        void publishBodyTarget();

        /** case 30：开关全身遥操作 */
        void toggleWholeBodyTeleop();
        void startWholeBodyTeleop();
        void stopWholeBodyTeleop();

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
        bool isFullBodyYbPauseMode() const noexcept
        {
            return full_body_yb_pause_mode_.load();
        }
        bool isYbModeConversionPending() const;
        void resetYbModeLatchAndConversions();
        void updateYbModeConversionState();
        void applyPauseAfterRebase(WbcToggleTarget arm);
        void clearArmPause(WbcToggleTarget arm);
        bool handleCase16YbModeToggle();
        bool rebaseArmVrControlFromCurrentPose(WbcToggleTarget arm);
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

        // ================= 全身骨架绝对位姿映射（全部跟踪） =================

        /** 骨架手部回调（末端数据源，左/右各一个话题） */
        void bodyHandCallback(bool is_left, geometry_msgs::msg::Pose::SharedPtr msg);

        /** 骨架足部/踝部/肩部回调：用于构造人体根 H 与骨架分段尺度 */
        void bodyPointCallback(bool is_left, bool is_ankle, geometry_msgs::msg::Pose::SharedPtr msg);

        /** 骨架分点：肩部（用于肩宽/臂长尺度） */
        void bodyShoulderCallback(bool is_left, geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * 骨架分段尺度变形后的各点位置（均在人体根 H 坐标系下，单位 m）：
         *   p'spine3  = p_pelvis  + s_body    ·(p_spine3  − p_pelvis)
         *   p'head    = p'spine3 + s_body    ·(p_head    − p_spine3)
         *   p'shoulder= p'spine3 + s_shoulder·(p_shoulder− p_spine3)
         *   p'hand    = p'shoulder+ s_arm     ·(p_hand     − p_shoulder)
         * pelvis 不变，所有关节姿态不变（只缩放段长，不改方向）。
         * 某段端点不可用时该段尺度退回 1.0（节流告警），不阻断其它段。
         */
        struct DeformedSkeleton
        {
            Eigen::Vector3d pelvis = Eigen::Vector3d::Zero();
            Eigen::Vector3d spine3 = Eigen::Vector3d::Zero();
            Eigen::Vector3d head = Eigen::Vector3d::Zero();
            Eigen::Vector3d shoulder_left = Eigen::Vector3d::Zero();
            Eigen::Vector3d shoulder_right = Eigen::Vector3d::Zero();
            Eigen::Vector3d hand_left = Eigen::Vector3d::Zero();
            Eigen::Vector3d hand_right = Eigen::Vector3d::Zero();
        };
        void buildDeformedSkeleton(const Eigen::Isometry3d& T_H_world, DeformedSkeleton& out);

        /**
         * 人体根 H（由下肢骨架构造）：
         *   位置 xy  = 两**踝** xy 的均值（踝比脚尖更接近人的站立点）
         *   位置 z   = 两**足** z 的较低者（行走时支撑脚即地面，天然屏蔽摆动脚）
         *   朝向     = 两**踝** x 轴在地平面投影的平均（直立、非内八外八时即人体前向）
         *   俯仰/滚转恒为 0，即 H 永远是竖直系（与 base_footprint 同构）。
         * 下肢数据断流时冻结上一帧 H，从不建过则返回 false。
         */
        bool humanRootTransform(Eigen::Isometry3d& T_H_world);

        /** 单关节骨架数据是否新鲜（未超时） */
        bool skeletonJointFresh(const std::chrono::steady_clock::time_point& stamp) const;

        /**
         * 机器人底座 → world 的位姿。
         * 优先查 TF（vr_follow_frame → ee_frame_id）；TF 不可用时回退到
         * 「对自己下发 /cmd_vel 的积分」，保证底盘跟随场景下手臂不被拖拽。
         */
        Eigen::Isometry3d currentBaseToWorld();

        /** 积分自己实际下发的 /cmd_vel，作为无 TF 时的底座位姿估计 */
        void integrateBaseOdometry(const geometry_msgs::msg::Twist& velocity);

        /** 全身模式内短按右摇杆：开/关「全部跟踪」 */
        void toggleAbsoluteTracking();
        void startAbsoluteTracking();
        void stopAbsoluteTracking();

        /** 一次性标定：用机器人当前实际位姿与人体当前位姿求出固定映射，之后不再 rebase */
        bool captureAbsoluteTrackingMaps();

        /** 由骨架绝对位姿发布单侧双臂末端目标 */
        void publishAbsoluteArmTarget(bool is_left);

        /** 人体手臂相对躯干长度是否在容差内（拉手/推断失败帧过滤） */
        bool geometryToleranceOk(bool is_left, const Eigen::Vector3d& hand_position) const;

        /** 只保留可达自由度：ZYX 分解后丢弃 roll（W2 腰无 roll、头只有 yaw/pitch） */
        static Eigen::Quaterniond dropRoll(const Eigen::Quaterniond& q);

        /** 清掉绝对映射状态与标定结果（停跟踪 / 失能 / 退出模式） */
        void resetAbsoluteTracking();

        // ================= 底盘跟踪（位置外环 → /cmd_vel） =================

        /** 全身模式内长按右摇杆：开/关底盘跟踪 */
        void toggleBaseTracking();

        /** 底盘外环前置条件 */
        bool baseFollowReady() const;

        /** 开启底盘跟踪时建立软参考基准 */
        void captureBaseFollowBase();

        /** 停止底盘跟踪并清零（发一次零 Twist） */
        void resetBaseFollow();

        /** 底盘外环定时器回调：骨盆水平位姿 → 全向 /cmd_vel */
        void updateBaseFollow();

        // OCS2 且 WBC 底盘规划解锁时屏蔽 VR 底盘速度（包括清零），保留手柄模式。
        void publishChassisVelocity(const geometry_msgs::msg::Twist& velocity);
        [[nodiscard]] bool wbcOwnsChassisVelocity() const;

        /**
         * 切换底盘控制模式（case 20 触发）
         * 进入：清零并停止末端摇杆累积；退出：清零底盘与腰部命令
         */
        void toggleChassisMode();

        /**
         * 把 /cmd_vel、waist_lifting_command、waist_turning_command 三个话题各发一次 0
         * 用于退出底盘模式、disable、enabled 变 false 等场景，防止残留运动
         * OCS2 且 WBC 底盘规划解锁时不向 /cmd_vel 发零速，避免干扰全身控制。
         */
        void resetChassisAndWaistCommands();

        /**
         * 按钮事件回调函数（处理按钮事件数字）
         * @param msg Int32消息，包含按钮事件数字 (0=无事件, 1-6=按钮按下, 7=镜像模式切换, 9=左扳机, 10=右扳机)
         */
        void processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg);

        void handleGripActiveEdge(bool isLeft, bool pressed);
        void applyGripShortPress(bool isLeft);
        void markGripComboConsumed(bool left, bool right);

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
         * 查询目标夹爪是否处于百分比模式
         * @param is_target_left_arm true=左夹爪, false=右夹爪
         */
        bool isGripperPercentMode(bool is_target_left_arm) const;

        /**
         * 按当前扳机映射翻转目标夹爪的百分比/开合模式（含镜像）
         * @param is_left_trigger true=事件13路径（左扳机）, false=事件14路径（右扳机）
         */
        void toggleGripperPercentMode(bool is_left_trigger);

        /**
         * 模式切换时的状态准备：不发任何夹爪指令，夹爪保持当前位置
         * 切到百分比：清 last_gripper_percent_ 缓存并置 armed=false，等扳机回中后才跟随
         * 切到开合：用最后一次百分比把 *_gripper_open_ 对齐到实际位置，避免 9/10 方向反了
         * @param is_target_left_arm true=左夹爪
         * @param percent_mode true=切到百分比模式, false=切到开合模式
         */
        void prepareGripperForModeSwitch(bool is_target_left_arm, bool percent_mode);

        /**
         * 尺度校准。事件 15 固定传 use_left_z=true。
         * @param use_left_z true=头显-左手柄 Z 与左臂末端 Z（现行入口）；false=右手柄路径（保留，热路径不传）
         */
        void runScaleCalibration(bool use_left_z);

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
         * 解析当前 FSM 状态码。优先用 ArmsTargetManager 的校验后状态，
         * 避免 VR 本地缓存漏掉 MOVEJ(4) 等命令后与 ATM 脱节。
         * @return 1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ
         */
        int32_t resolvedFsmState() const;

        /**
         * 将控制器坐标系下的目标平滑后发布到 left_target/right_target。
         * position/orientation 为 ee_frame_id_ Pose。
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 控制器坐标系下的位置
         * @param orientation 控制器坐标系下的方向
         * @return true 表示本次请求真正发布；false 表示首次解耦重建、TF 失败、未变化或发布器不可用
         */
        bool publishTargetPoseDirect(const std::string& armType,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation);

        /**
         * 冻结追赶斜坡：上游 XR 位姿在链路数据未更新时会逐位重发上一帧，恢复后
         * 一帧补上整段位移（实测最大 124.7mm / 11.5°）。这里检测冻结，并把追赶量
         * 按冻结帧数等额摊到随后若干帧，使正常运动逐位不受影响。
         * 仅在真正发布 target 的路径上调用，因此 STORAGE 模式下不生效。
         * @param isLeft true=左臂状态，false=右臂状态
         * @param position 控制器坐标系下的位置，原地改写为摊平后的值
         * @param orientation 控制器坐标系下的方向，原地改写为摊平后的值
         */
        void applyStaleCatchUpRamp(bool isLeft,
                                   Eigen::Vector3d& position,
                                   Eigen::Quaterniond& orientation);

        /** 清空冻结追赶斜坡状态；下一帧重新起基准，不会摊平。 */
        void resetStaleCatchUpRamp(bool isLeft);

        /** 左右臂一起清空。 */
        void resetStaleCatchUpRamp();

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
         * 保存的是 ee_frame_id_ Pose。
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 控制器坐标系下的位置
         * @param orientation 控制器坐标系下的方向
         */
        void recordLastPublishedTarget(const std::string& armType,
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation);

        /**
         * 清空上一次 OCS2 会话发布的 command target 缓存，并使左右臂基准失效。
         */
        void clearLastPublishedTargets();

        /**
         * 设置对应手臂的机器人锚点，可优先沿用最后目标或直接使用实际末端。
         * 锚点保持在 ee_frame_id_，同时冻结 F -> C 旋转；失败时基准无效，不发布。
         * @param armType 手臂类型 ("left" 或 "right")
         * @param preferLastCommand true 优先最后目标、无历史时用实际末端；false 直接用实际末端
         * @return true 表示控制器系锚点和冻结旋转均有效
         */
        bool setRobotBase(const std::string& armType, bool preferLastCommand);

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
        // 底盘跟踪位置外环定时器（仅 base_tracking_enabled_ 时发布）
        rclcpp::TimerBase::SharedPtr base_follow_timer_;
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
        // 全身骨骼关节订阅器（/teleop/body_<关节名>_pose；话题动态增删，出现后自动匹配）
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_pelvis_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_spine3_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_head_;
        // 骨架手部位姿订阅器（全身绝对映射的末端数据源）
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_left_hand_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_right_hand_;
        // 骨架踝部位姿订阅器（人体根 xy/偏航）
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_left_ankle_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_right_ankle_;
        // 骨架肩部位姿订阅器（肩宽 / 臂长尺度链的起点）
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_left_shoulder_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_right_shoulder_;
        // 骨架足部位姿订阅器（人体根 z）
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_left_foot_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_right_foot_;
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
        rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCapability>::SharedPtr
            sub_wbc_capability_;

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

        // All callbacks use the node's default mutually-exclusive callback group.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_head_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_head_target_;
        geometry_msgs::msg::PoseStamped robot_head_pose_;
        bool has_robot_head_pose_{false};
        bool has_vr_head_pose_{false};
        bool head_tracking_active_{false};
        Eigen::Vector3d vr_head_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_head_base_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_head_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_head_base_orientation_ = Eigen::Quaterniond::Identity();
        // Last published command with its current-pose frame, not measured pose.
        std::optional<geometry_msgs::msg::PoseStamped> last_head_target_;

        // ---- 全身遥操（case 30）状态 ----
        std::atomic<bool> whole_body_teleop_active_{false};
        // 骨架缓存（机器人世界坐标系，与 head/controller 同原点）
        Eigen::Vector3d body_pelvis_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_pelvis_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d body_spine3_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_spine3_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d body_head_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_head_orientation_ = Eigen::Quaterniond::Identity();
        bool has_body_pelvis_pose_{false};
        bool has_body_spine3_pose_{false};
        bool has_body_head_pose_{false};

        // ---- 全身骨架绝对位姿映射（全部跟踪） ----
        // 参数
        AbsoluteTeleopOptions absolute_options_;
        // 「全部跟踪」开关（全身模式内短按右摇杆 toggle）
        std::atomic<bool> absolute_tracking_active_{false};
        // 一次性标定是否已完成
        bool absolute_maps_captured_{false};
        // 标定前置稳定窗口：收到第一帧骨架数据后等待稳定才锁定
        bool settle_pending_{false};
        std::chrono::steady_clock::time_point settle_started_at_{};

        // 骨架手部缓存（VR 世界系）
        Eigen::Vector3d body_left_hand_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_left_hand_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d body_right_hand_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_right_hand_orientation_ = Eigen::Quaterniond::Identity();
        bool has_body_left_hand_pose_{false};
        bool has_body_right_hand_pose_{false};
        // 骨架下肢点缓存（足与踝共用结构）：踝提供 xy/偏航，足提供 z
        struct SkeletonPoint
        {
            Eigen::Vector2d xy = Eigen::Vector2d::Zero();
            double z{0.0};
            Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
            bool valid{false};
            std::chrono::steady_clock::time_point stamp{};
        };
        SkeletonPoint body_left_ankle_;
        SkeletonPoint body_right_ankle_;
        SkeletonPoint body_left_foot_;
        SkeletonPoint body_right_foot_;
        SkeletonPoint body_left_shoulder_;
        SkeletonPoint body_right_shoulder_;
        // 人体根 H 的低通状态（xy 与 z）与朝向
        Eigen::Vector2d human_root_xy_ = Eigen::Vector2d::Zero();
        double ground_z_{0.0};
        double human_root_yaw_{0.0};
        bool human_root_yaw_valid_{false};
        bool human_root_initialized_{false};
        // 各骨架话题的最后更新时间（看门狗）
        std::chrono::steady_clock::time_point body_pelvis_stamp_{};
        std::chrono::steady_clock::time_point body_spine3_stamp_{};
        std::chrono::steady_clock::time_point body_head_stamp_{};
        std::chrono::steady_clock::time_point body_left_hand_stamp_{};
        std::chrono::steady_clock::time_point body_right_hand_stamp_{};

        // 标定结果：
        //   人体部位在「人体根 H」里的位姿直接作为机器人对应部位「相对底座」的目标：
        //     位置 = scale · R_align · p_joint_in_H(t)
        //     姿态 = R_align · R_joint_in_H(t) · C_src→ee
        // 其中 H = 骨盆的地面投影（z=0）+ 骨盆偏航，与 base_footprint（也在地面）同基准，
        // 所以位移偏置默认为 0；R_align 只补一个固定偏航。
        // 人体自身的行走/整体转身完全被 H 吸收（只去驱动底盘），
        // 因此底盘锁定时人走动不会拖动手臂目标，底盘跟随时手臂也能保持相对机体的姿态。
        // 已删除 abs_align_rot_：人体根 H 自身坐标系已是「人体系」，
        // 目标直接取其下位姿即可（人体朝向 ≡ 机器人机头朝向）。
        // 数据源坐标系 → 目标坐标系的固定旋转偏置（构造时由 RPY 参数算好）
        Eigen::Matrix3d abs_hand_to_ee_left_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d abs_hand_to_ee_right_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d abs_body_src_to_ee_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d abs_head_src_to_ee_ = Eigen::Matrix3d::Identity();
        // 人体参考臂长（手–脊柱距离），用于几何一致性过滤
        double abs_left_arm_ref_len_{0.0};
        double abs_right_arm_ref_len_{0.0};
        // 底座位姿估计（world 系，仅作 TF 不可用时的回退）
        double base_est_yaw_{0.0};
        Eigen::Vector2d base_est_pos_ = Eigen::Vector2d::Zero();
        bool base_odom_valid_{false};
        std::chrono::steady_clock::time_point base_odom_last_tick_{};

        // 绝对目标的低通状态（位置 + 姿态，逐任务一份）
        bool abs_left_filter_primed_{false};
        bool abs_right_filter_primed_{false};
        bool abs_body_filter_primed_{false};
        bool abs_head_filter_primed_{false};
        Eigen::Vector3d abs_left_pos_lp_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond abs_left_ori_lp_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d abs_right_pos_lp_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond abs_right_ori_lp_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d abs_body_pos_lp_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond abs_body_ori_lp_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d abs_head_pos_lp_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond abs_head_ori_lp_ = Eigen::Quaterniond::Identity();

        // ---- 底盘跟踪（位置外环 → /cmd_vel） ----
        // 「底盘跟踪」开关（全身模式内长按右摇杆 toggle）
        std::atomic<bool> base_tracking_enabled_{false};
        bool base_follow_active_{false};
        // 基准：接管时刻的人体骨盆水平位姿与偏航
        Eigen::Vector3d base_follow_pelvis_ref_ = Eigen::Vector3d::Zero();
        double base_follow_yaw_ref_{0.0};
        // yaw 补偿：积分自己下发的 ω，使机器人自身转向后 VR“前方”仍对应机头前方
        double base_follow_yaw_offset_{0.0};
        // 软参考（跑步机）状态：接管时刻为原点、已转到「接管时刻机器人本体系」F
        Eigen::Vector2d base_follow_p_ref_ = Eigen::Vector2d::Zero();
        // 指令与低通/速率限幅状态
        Eigen::Vector2d base_follow_v_cmd_ = Eigen::Vector2d::Zero();
        double base_follow_w_cmd_{0.0};
        Eigen::Vector2d base_follow_v_out_ = Eigen::Vector2d::Zero();
        double base_follow_w_out_{0.0};
        // 外环上次执行时刻（用于积分与限幅）
        std::chrono::steady_clock::time_point base_follow_last_tick_{};

        // 最后实际发布 frame（ee_frame_id_）的 Pose，仅用于变化检测
        Eigen::Vector3d prev_calculated_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prev_calculated_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_right_orientation_ = Eigen::Quaterniond::Identity();

        // 最后一次实际发布的控制器系 command target（ee_frame_id_ Pose）。
        bool has_last_published_left_target_ = false;
        Eigen::Vector3d last_published_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_published_left_orientation_ = Eigen::Quaterniond::Identity();
        bool has_last_published_right_target_ = false;
        Eigen::Vector3d last_published_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_published_right_orientation_ = Eigen::Quaterniond::Identity();

        // 冻结追赶斜坡状态（每臂一份），语义见 applyStaleCatchUpRamp()。
        struct StaleCatchUpRamp
        {
            bool has_previous_input = false;
            // 上一帧进入斜坡的原始目标，用于逐位比较判定"上游冻结"
            Eigen::Vector3d previous_input_position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond previous_input_orientation = Eigen::Quaterniond::Identity();
            int frozen_frames = 0;
            // 上一帧实际发出的目标，解冻帧据此测量缺口
            Eigen::Vector3d output_position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond output_orientation = Eigen::Quaterniond::Identity();
            // 尚未交付的追赶量；remaining_frames 归零时必然为零
            Eigen::Vector3d residual_position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond residual_orientation = Eigen::Quaterniond::Identity();
            int remaining_frames = 0;
        };
        StaleCatchUpRamp left_stale_ramp_;
        StaleCatchUpRamp right_stale_ramp_;

        // 状态管理
        std::atomic<bool> enabled_;
        std::atomic<bool> is_update_mode_; // true = 更新模式, false = 存储模式
        std::atomic<bool> mirror_mode_; // true = 镜像模式, false = 正常模式
        std::atomic<int> mirror_mode_profile_{static_cast<int>(MirrorModeProfile::MODE_A)};
        std::atomic<bool> left_arm_paused_; // 左臂是否暂停更新（Y按键控制）
        std::atomic<bool> right_arm_paused_; // 右臂是否暂停更新（B按键控制）
        std::atomic<bool> left_grip_mode_; // 左摇杆控制模式：false=XY平移, true=Z轴+Yaw
        std::atomic<bool> right_grip_mode_; // 右摇杆控制模式：false=XY平移, true=Z轴+Yaw
        std::atomic<bool> chassis_mode_; // true = 底盘控制模式, false = 末端控制模式（case 20 切换）
        std::atomic<bool> left_grip_active_{false};  // 左握把实时状态
        std::atomic<bool> right_grip_active_{false}; // 右握把实时状态（chassis 模式下用作腰部控制修饰符）
        std::atomic<bool> left_grip_press_armed_{false};
        std::atomic<bool> right_grip_press_armed_{false};
        std::atomic<bool> left_grip_combo_consumed_{false};
        std::atomic<bool> right_grip_combo_consumed_{false};
        std::chrono::steady_clock::time_point left_grip_press_start_{};
        std::chrono::steady_clock::time_point right_grip_press_start_{};
        static constexpr auto grip_short_press_max_ = std::chrono::milliseconds(400);
        // FULL_BODY 下左握把方向组合的安全锁存：
        // 松开握把后仍保持抑制，直到左摇杆 X/Y 都回到 0.3 内。
        std::atomic<bool> left_grip_direction_suppressed_{false};
        // FULL_BODY 下右握把方向组合的安全锁存，语义与左侧对称。
        std::atomic<bool> right_grip_direction_suppressed_{false};

        std::atomic<bool> has_home_joint_reference_{false};
        std::atomic<bool> home_joint_reference_enabled_{false};

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

        // FULL_BODY 下 case 16 锁存：false=Y/B 为 WBC 单臂禁用，true=Y/B 为 VR 暂停
        std::atomic<bool> full_body_yb_pause_mode_{false};
        std::atomic<bool> left_pause_after_enable_{false};
        std::atomic<bool> right_pause_after_enable_{false};
        std::atomic<bool> left_clear_pause_after_disable_{false};
        std::atomic<bool> right_clear_pause_after_disable_{false};
        std::chrono::steady_clock::time_point left_yb_conversion_time_{};
        std::chrono::steady_clock::time_point right_yb_conversion_time_{};
        static constexpr auto yb_conversion_timeout_ =
            std::chrono::seconds(2);

        std::atomic<bool> has_robot_current_left_pose_{false};
        std::atomic<bool> has_robot_current_right_pose_{false};
        std::atomic<bool> has_vr_left_pose_{false};
        std::atomic<bool> has_vr_right_pose_{false};

        // 双臂耦合期间仍发布 right_target（由控制器投影到耦合流形）；
        // 解耦后的下一次右臂发布前需重建控制基准，避免冲向耦合期 VR 映射位姿。
        bool right_vr_target_suppressed_by_bimanual_{false};

        std::atomic<int32_t> current_fsm_state_; // 当前FSM状态：1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ

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

        // 每臂恢复时冻结的 F -> C 旋转；有效性由 robot_base_valid_ 管理。
        Eigen::Quaterniond left_follow_rotation_ = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond right_follow_rotation_ = Eigen::Quaterniond::Identity();

        // 机器人末端基准：ee_frame_id_ 中锁存的 Pose
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
        std::atomic<bool> left_gripper_percent_mode_;
        std::atomic<bool> right_gripper_percent_mode_;
        // 切到百分比后的三段式接管，避免"切换即张开/切换即夹死"：
        //   ① saw_release=false：组合键的收尾释放期，不发任何指令
        //      （13/14 的触发条件是"握把按住时扳机上升沿"，见 xr_target_node.py:651，
        //        按下组合键时扳机必然扣着，松开它只是收尾动作，不算夹爪指令）
        //   ② saw_release=true 且 hand_percent > hold：等新指令，夹爪停在 hold
        //   ③ hand_percent <= hold：armed=true，从夹爪当前位置 1:1 接管，无跳变
        std::atomic<bool> left_gripper_percent_armed_;
        std::atomic<bool> right_gripper_percent_armed_;
        std::atomic<bool> left_gripper_percent_saw_release_;
        std::atomic<bool> right_gripper_percent_saw_release_;
        // 切换瞬间夹爪所在位置（0.0=闭合, 1.0=张开），接管点
        std::atomic<double> left_gripper_percent_hold_;
        std::atomic<double> right_gripper_percent_hold_;
        std::chrono::steady_clock::time_point last_left_gripper_mode_toggle_time_{};
        std::chrono::steady_clock::time_point last_right_gripper_mode_toggle_time_{};

        // VR控制缩放参数（从配置文件读取 / 固定配置）
        double vr_thumbstick_linear_scale_; // 摇杆位置缩放因子（单位 m/step）
        double vr_thumbstick_angular_scale_; // 摇杆旋转缩放因子（单位 rad/step）
        double vr_pose_scale_;  // 手柄位姿位置缩放因子（左右共用；事件 15 按左侧校准）

        // 底盘/腰部控制硬编码速度比例（后续可参数化，参考 joystick_teleop.cpp）
        static constexpr double chassis_linear_scale_ = 0.25;   // 底盘线速度比例
        static constexpr double chassis_angular_scale_ = 0.5;   // 底盘角速度比例
        static constexpr double waist_lifting_scale_ = 1.0;     // 腰部升降 factor（控制器内部 clamp 到 [-1,1]，>1 无意义）
        static constexpr double waist_turning_scale_ = 1.0;     // 腰部旋转 factor（同上）
        enum class WaistStickAxis { NONE, LIFTING, TURNING };
        std::atomic<WaistStickAxis> waist_stick_axis_{WaistStickAxis::NONE};
        static constexpr double waist_axis_switch_margin_ = 0.1;


        // 参考link名称（用于将VR头显/手柄关联到机器人某个link），从target_manager.yaml读取，默认"base_link"
        std::string reference_link_;

        // VR 增量的参考轴系；每次恢复时冻结其到控制器系的旋转
        std::string vr_follow_frame_;

        // 来自 left_current_pose/right_current_pose.header.frame_id；
        // 普通 left_target/right_target 在控制器侧采用该隐式坐标系。
        std::string ee_frame_id_;
        bool ee_frame_id_initialized_ = false;

        // TF组件（用于在不同link/frame之间转换）
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // 常量
        static const double POSITION_THRESHOLD;
        static const double ORIENTATION_THRESHOLD;
        // 连续多少帧原始目标逐位相同才判定为上游冻结。实测冻结长度呈双峰分布
        // （1 帧 / ≥5 帧），2..5 取任意值结果相同，取下界即可。
        static const int STALE_MIN_FROZEN_FRAMES;
    };
} // namespace arms_ros2_control::command
