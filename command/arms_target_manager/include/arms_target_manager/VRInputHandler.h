//
// VRInputHandler - VR 遥操作的唯一类（普通模式 + 全身模式）。
//
// 一个类、一份状态：
//   * 输入：/teleop 下的手柄、头显、骨架、按键、摇杆、扳机；
//   * 输出：一律通过 ArmsTargetManager 的 TargetRouter 提交 Intent，
//           手部命令通过 HandCommandBus；
//   * 会话：VrSession 决定当前模式允许哪些动作（见 core/ModeRegistry.h）。
//
// 实现集中在一个 .cpp 里，按“构造 → 按键 → 输入 → 普通映射 → 夹爪 →
// 光学握手 → WBC → 底盘 → 全身 → 工具”分区，新增组合键或功能时
// 先看 .cpp 顶部的分区说明与组合键一览。
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
#include <tf2_msgs/msg/tf_message.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <optional>
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_target_manager/core/Action.h"
#include "arms_target_manager/core/FsmClient.h"
#include "arms_target_manager/core/HandCommandBus.h"
#include "arms_target_manager/core/HumanoidModeClient.h"
#include "arms_target_manager/core/InputHub.h"
#include "arms_target_manager/core/ModeRegistry.h"
#include "arms_target_manager/detail/PoseFilters.h"
#include <arms_controller_common/utils/PoseRamp.h>
#include "arms_controller_common/utils/FSMCommandPublisher.h"
#include <arms_ros2_control_msgs/msg/wbc_capability.hpp>

namespace arms_ros2_control::command
{
    /**
     * 全身骨架绝对位姿映射的可调参数。
     *
     * 这些开关只在「全身遥操作模式」（case 30 已开启）内部生效：
     *   - 右摇杆短按（事件 4） = 开/关「全部跟踪」→ 胸/头/双臂末端改为 /teleop/body_* 绝对位姿映射
     * 底盘跟随不由本节点做速度外环：由 OCS2 负责
     * （事件 26 解锁底盘后，MPC 用底盘自由度去追世界系 marker）。
     * 非全身模式下的一切既有行为（UPDATE/STORAGE、摇杆微调、镜像等）不受影响；
     * 事件 20 的手动底盘遥控在两种模式下都可用。
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
        // ===== 「人体关键点 → 机器人关键点」的固定平移偏置 [x, y, z]（米）=====
        // 表达在人体基坐标系 H 里（x 前 / y 左 / z 上，与 base_footprint 同向），
        // 在分段尺度缩放完成之后整体加到每个关键点上，只平移不改姿态。
        // 用途：两者 footprint 的原点定义本来就不同——人体 H 取骨盆地面投影，
        // 机器人 footprint 由底盘几何决定。若机器人 footprint 落在身体后下方
        // （底盘后悬）、而人体 footprint 在头部正下方，则机器人每个同名关键点
        // 都比人体整体偏前，此时把该前移量填进 x 即可。
        // 默认 [0,0,0] = 不做补偿。
        std::vector<double> human_to_robot_offset_xyz{0.0, 0.0, 0.0};
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
        // 注：人体基坐标系 H 的低通系数与原点偏置已迁到 vr_pose_publisher
        // （/teleop/human_footprint_pose 的发布端），本节点只消费结果。
        // 胸部位置沿「脊柱轴」的锚点比例：0 = 取 spine3 位置，1 = 取 pelvis 位置。
        // 机器人的 body_link4 比人体 spine3 偏下、大致在骨盆高度，所以默认取 1。
        double body_anchor_lambda{1.0};
        // 骨架几何一致性容差（臂长偏离比例）
        double geometry_tolerance{0.2};

        // ---- 接管渐入（仅「开启跟踪」与「数据恢复」的过渡段） ----
        // 全新开启「全部跟踪」时，目标位姿从机器人【当前实测位姿】按速度约束缓慢过渡到
        // 实时 VR 期望位姿；过渡结束后恢复直通，不影响稳态跟随手感。
        // 不能用上一次下发的目标做起点：带底盘时它可能已离开机器人。
        bool pose_ramp_enable{true};
        double pose_ramp_max_linear_velocity{0.15};   // [m/s]
        double pose_ramp_max_angular_velocity{0.4};   // [rad/s]
        double pose_ramp_min_duration_sec{0.2};       // [s]
        double pose_ramp_max_duration_sec{3.0};       // [s]
        // 目标输出中断超过该时长后再恢复（如骨架超时冻结）时，重新渐入 [s]
        double pose_ramp_resume_gap_sec{0.3};
    };

    /**
     * VRInputHandler - VR 遥操作的唯一实现。
     *
     * 普通模式（相对映射）与全身模式（骨架绝对映射）共用同一份状态：
     * 骨架订阅、锚定、渐入都在本类内，两个模式之间只是“用哪个映射”。
     *
     * 与外部的关系：
     *   - 输入：/teleop 下的话题（手柄、骨架、按键、摇杆、扳机）
     *   - 输出：全部经由 ArmsTargetManager 的 TargetRouter 提交 Intent
     *   - 手部：经由 HandCommandBus
     *   - 会话：VrSession 决定“当前模式允许哪些动作”
     */
    class VRInputHandler
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param targetManager ArmsTargetManager指针
         * @param handControllers 手部/夹爪控制器名称列表（用于映射夹爪命令）
         * @param vr_thumbstick_linear_scale VR摇杆线性缩放因子（单位 m/step）
         * @param vr_thumbstick_angular_scale VR摇杆角度缩放因子（单位 rad/step）
         * @param vr_pose_scale 手柄位姿位置缩放因子（左右共用；事件 15 左Y+右B 固定按左侧 Z 距校准）
         * @param reference_link 参考link名称（例如 head_link2），用于VR/头显关联
         * @param vr_follow_frame VR 增量参考轴系，恢复时冻结到控制器系的旋转（默认 base_footprint）
         * @param absolute_options 全身骨架绝对映射 / 底盘跟随参数
         *
         * 注：目标发布器不需要传入——所有目标都由 TargetRouter 统一持有与发布。
         */
        VRInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            const std::vector<std::string>& handControllers,
            double vr_thumbstick_linear_scale,
            double vr_thumbstick_angular_scale,
            double vr_pose_scale,
            const std::string& reference_link,
            const std::string& vr_follow_frame,
            const AbsoluteTeleopOptions& absolute_options = AbsoluteTeleopOptions{});

        ~VRInputHandler();

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
        bool isWholeBodyTeleopActive() const;

        /**
         * 「全部跟踪」是否已开启（全身模式内右摇杆短按 toggle）。
         */
        bool isAbsoluteTrackingActive() const;

        /**
         * 接入 FSM 统一入口。
         *
         * 接入后本类不再缓存 FSM 状态、也不再发布 fsm_command：
         * 状态读 `fsm_->isOcs2()`，命令发 `fsm_->request(意图)`。
         * 未接入时一律按 HOLD 处理（与控制器初始状态一致）。
         */
        void attachFsmClient(FsmClient* client);

        /**
         * 接入 WBC 模式统一入口。
         *
         * 接入后 `publishHumanoidModeCommand` 转发给 client，能力声明也由它推送；
         * 未接入时相关命令会被忽略并打 WARN。
         */
        void attachHumanoidModeClient(HumanoidModeClient* client);

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

        /**
         * 机器人胸部当前pose回调函数
         *
         * 由外部统一订阅（BodyMarker 已订阅 `body_current_pose`）后转发，避免重复订阅。
         * 全身模式的接管渐入需要「机器人现在在哪」，而不能用本节点上一次下发过的目标。
         * @param msg 机器人胸部位姿消息（frame_id 为控制器基准系）
         */
        void robotBodyPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

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

        // ---- 模式路由 ----
        void toggleWholeBodyMode();
        void handleNormalAction(Action action);
        void handleFsmButtonEvent(int32_t code);
        void handleGripperButtonEvent(int32_t code);
        bool isWholeBodyActive() const;

        // ---- 全身模式与普通模式的清场/恢复（同一个类的两半） ----
        void suspendNormalModeState();
        void resumeNormalModeState();

        // ---- 全身模式专用基础设施 hook ----
        bool publishArmTargetForWholeBody(bool is_left,
                                          const Eigen::Vector3d& position,
                                          const Eigen::Quaterniond& orientation);
        bool submitWholeBodyHeadTarget(const geometry_msgs::msg::Pose& pose,
                                       const std::string& frame_id);
        void rebaseAfterManualOverride(bool is_left);
        bool prepareWholeBodyArmInput(bool is_left);
        bool robotCurrentPose(bool is_left,
                              Eigen::Vector3d& position,
                              Eigen::Quaterniond& orientation) const;
        bool robotHeadPose(geometry_msgs::msg::PoseStamped& pose) const;
        /**
         * 机器人胸部的实测位姿（`body_current_pose`），用于接管渐入取锚点。
         * @param[out] position 位置
         * @param[out] orientation 姿态
         * @param[out] frame_id 表达该位姿的坐标系
         * @return false 表示还没有收到过有效数据
         */
        bool robotBodyPose(Eigen::Vector3d& position,
                           Eigen::Quaterniond& orientation,
                           std::string& frame_id) const;

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

        /** Publish latch of mirror_mode_ on /teleop/mirror_mode (TRANSIENT_LOCAL). */
        void publishMirrorMode();

        /**
         * Optional optical R0 handshake (not in main teleop topics):
         *   /teleop/optical_r0_caps     fa-py→ATM  bit0 = publisher implements handshake
         *   /teleop/optical_r0_request  ATM→fa-py  bit0/1 keep I+p_a; 0 = ACK open
         *   /teleop/optical_r0_ready    fa-py→ATM  bit0/1 contract frame on EE
         *
         * Peer support tracks /teleop/optical_r0_caps (assignable, including
         * caps=0). Once any caps message is seen, ready traffic no longer
         * latches support — only caps does. Legacy publishers that never send
         * caps may still learn support from ready. Unlearn only on
         * timeout&&alive (broken peer); a still headset (!alive) cancels
         * without unlearning.
         */
        void opticalR0ReadyCallback(const std_msgs::msg::Int32::SharedPtr msg);
        void opticalR0CapsCallback(const std_msgs::msg::Int32::SharedPtr msg);
        /** Current: fa-py advertises R0 handshake (caps=1, or legacy ready). */
        bool opticalR0HandshakeSupported() const;
        /** Live: ready traffic seen within optical_r0_ready_alive_window_. */
        bool opticalR0HandshakeAlive() const;
        /**
         * 100ms supervisor: completes pending enter/resume even when no VR pose
         * arrives (peer died) and expires them with a WARN.
         */
        void superviseOpticalHandshake();
        /** Drop pendings + request bits without touching arm pause state. */
        void cancelOpticalPending(const char* reason);
        /** Forget peer support until caps=1 (or legacy ready) returns. */
        void unlearnOpticalR0Peer(const char* reason);
        void publishOpticalR0Request(int32_t bits);
        int32_t desiredOpticalR0RequestBits() const;
        void refreshOpticalR0Request();
        void tryCompleteOpticalUpdateEnter();
        bool tryCompleteOpticalSideSnap(bool useLeftVr);
        void tryCompleteOpticalResumes();
        /** Pause→resume one arm: optical handshake or main topic snap. */
        bool resumeArmVrControl(WbcToggleTarget arm, const char* buttonTag);

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
         * 发送FSM命令（兼容入口）。优先使用 requestFsmIntent()，不要拼命令号。
         * @param command FSM命令值 (1=HOME, 2=HOLD, 3=OCS2, 100=切换姿态(REST), 0=重置)
         */
        void sendFsmCommand(int32_t command);

        /** 请求一次“前进/后退”，合法性由 FsmClient 统一判定。 */
        void requestFsmIntent(FsmIntent intent);

        /** 当前是否处于 OCS2（未接入 FsmClient 时恒为 false，即按 HOLD 处理）。 */
        [[nodiscard]] bool isOcs2State() const noexcept;

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
        // 注：left/right/head/body 目标的发布器全部由 TargetRouter 持有，
        // 本类只提交 Intent，不再保存任何发布器。
        // FSM 统一入口（由节点注入；未注入时按 HOLD 处理）
        FsmClient* fsm_{nullptr};
        uint64_t fsm_observer_token_{0};

        // 订阅器
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_left_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_right_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_head_;
        // 按钮事件订阅器（Int32类型）
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_controller_state_;
        // Optional optical R0 handshake (absent ⇒ controller main R0).
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_optical_r0_caps_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_optical_r0_ready_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_optical_r0_request_;
        std::atomic<bool> optical_r0_peer_supports_{false};
        /** True after any caps message; gates whether ready may latch support. */
        std::atomic<bool> optical_caps_seen_{false};
        std::atomic<int32_t> optical_r0_ready_bits_{0};
        std::atomic<int64_t> optical_r0_ready_last_ns_{0};
        std::atomic<int32_t> optical_r0_request_bits_{0};
        std::atomic<bool> optical_update_enter_pending_{false};
        std::chrono::steady_clock::time_point optical_update_enter_pending_since_{};
        std::atomic<bool> optical_resume_pending_left_{false};
        std::atomic<bool> optical_resume_pending_right_{false};
        /** Armed-at times so a dead peer cannot leave a resume pending forever. */
        std::chrono::steady_clock::time_point optical_resume_pending_left_since_{};
        std::chrono::steady_clock::time_point optical_resume_pending_right_since_{};
        /**
         * VR side latched at request time. Mirror can flip while a resume is
         * pending; re-deriving from mirror_mode_ then would snap the wrong
         * wrist base and strand the other side's paused pose.
         */
        std::atomic<bool> optical_resume_use_left_vr_left_{false};
        std::atomic<bool> optical_resume_use_left_vr_right_{false};
        /** One WARN per cancelled pending, not one per 30 Hz frame. */
        std::atomic<bool> optical_fallback_warned_{false};
        rclcpp::TimerBase::SharedPtr optical_handshake_timer_;
        static constexpr int32_t optical_r0_caps_version_ = 1;
        static constexpr auto optical_r0_handshake_timeout_ =
            std::chrono::milliseconds(3000);
        static constexpr auto optical_r0_ready_alive_window_ =
            std::chrono::milliseconds(750);
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

        // WBC 模式切换命令现在统一由 HumanoidModeClient 发布
        // Level latch for hand sinks / DualHand (same bit as EE/trigger mirror).
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_mirror_mode_;
        // WBC 模式统一入口（由节点注入）
        HumanoidModeClient* mode_client_{nullptr};
        uint64_t capability_observer_token_{0};

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
        // 注：head_target 由 TargetRouter 统一发布，本类不再持有发布器。
        geometry_msgs::msg::PoseStamped robot_head_pose_;
        bool has_robot_head_pose_{false};
        // 胸腔实测位姿（body_current_pose）：全身接管渐入的锚点来源之一
        geometry_msgs::msg::PoseStamped robot_body_pose_;
        bool has_robot_body_pose_{false};
        bool has_vr_head_pose_{false};
        bool head_tracking_active_{false};
        Eigen::Vector3d vr_head_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_head_base_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_head_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_head_base_orientation_ = Eigen::Quaterniond::Identity();
        // Last published command with its current-pose frame, not measured pose.
        std::optional<geometry_msgs::msg::PoseStamped> last_head_target_;
        std::chrono::steady_clock::time_point last_head_target_stamp_{};

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
        // 最后一次下发时刻，用于判断锚点是否仍然有效（旧缓存只说明「曾经的意图」
        // 已可能被 RViz marker / control_input 改写过，此时应退回实测位姿）。
        std::chrono::steady_clock::time_point last_published_left_stamp_{};
        std::chrono::steady_clock::time_point last_published_right_stamp_{};

        // 冻结追赶（每臂一份），实现见 detail::StaleCatchUpFilter。
        detail::StaleCatchUpFilter left_stale_ramp_{STALE_MIN_FROZEN_FRAMES};
        detail::StaleCatchUpFilter right_stale_ramp_{STALE_MIN_FROZEN_FRAMES};

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
        std::chrono::steady_clock::time_point left_wbc_rebase_pending_since_{};
        std::atomic<bool> left_wbc_rebase_use_left_vr_{false};
        std::atomic<bool> right_wbc_rebase_pending_{false};
        std::chrono::steady_clock::time_point right_wbc_rebase_pending_since_{};
        std::atomic<bool> right_wbc_rebase_use_left_vr_{false};

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

        // RViz 手动拖动抢占手臂时的“让位”标记：下次恢复控制前先重建基准。
        std::atomic<bool> left_manual_override_{false};
        std::atomic<bool> right_manual_override_{false};

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
        // 手部命令统一出口（与摇杆/键盘共用）
        std::unique_ptr<HandCommandBus> hand_bus_;
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

        // TF（相对 tf/tf_static，多机 ns 下不用绝对 /tf 的 TransformListener）
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

        // 常量
        // 连续多少帧原始目标逐位相同才判定为上游冻结。实测冻结长度呈双峰分布
        // （1 帧 / ≥5 帧），2..5 取任意值结果相同，取下界即可。
        static const int STALE_MIN_FROZEN_FRAMES;

    public:
        // ================= 全身模式（与普通模式同属本类） =================

        /** 某侧手臂最后一次实际下发的目标（供接管渐入取锚点）。 */
        struct LastArmTarget
        {
            bool valid{false};
            Eigen::Vector3d position{Eigen::Vector3d::Zero()};
            Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
            std::chrono::steady_clock::time_point stamp{};
        };

        [[nodiscard]] bool wholeBodyActive() const { return whole_body_teleop_active_.load(); }
        [[nodiscard]] bool absoluteTrackingActive() const
        {
            return absolute_tracking_active_.load();
        }

        /** 事件 30：进入/退出全身遥操作。 */
        void toggleWholeBodyTeleop();
        /** 全身模式内的事件入口。 */
        void handleWholeBodyButtonEvent(int32_t code);
        /** 权威 FSM 变化：离开 OCS2 时自动退出全身模式。 */
        void onWholeBodyFsmStateChanged();

        /** FSM 状态变化回调（由 FsmClient 观察者驱动）。 */
        void onFsmTransition(FsmState from, FsmState to);
        /** 拓扑变化：不再是全身控制器时自动退出。 */
        void onWholeBodyTopologyChanged(bool is_full_body);
        /** 头部状态变化：非跟踪时停止全身头部目标。 */
        void onWholeBodyHeadStateChanged(int head_state);
        /** VR 失能时的全身侧清理。 */
        void deactivateWholeBodyTeleop();
        /** 普通模式的目标缓存被清空时，同步清掉全身渐入锚点。 */
        void onWholeBodyTargetCacheCleared();
        /** 手动拖动结束后重新布置该臂的接管渐入。 */
        void rearmWholeBodyArmRamp(bool is_left);

    private:
        struct SkeletonPoint
        {
            Eigen::Vector2d xy = Eigen::Vector2d::Zero();
            double z{0.0};
            bool valid{false};
            std::chrono::steady_clock::time_point stamp{};
        };

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

        // ---- 全身：初始化与模式开关 ----
        void initializeWholeBodyTeleop();
        void startWholeBodyTeleop();
        void stopWholeBodyTeleop();

        // ---- 全身：骨架输入 ----
        void bodyPelvisCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void bodySpine3Callback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void bodyHeadCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void bodyHandCallback(bool is_left, const geometry_msgs::msg::Pose::SharedPtr msg);
        void bodyShoulderCallback(bool is_left, const geometry_msgs::msg::Pose::SharedPtr msg);
        void humanFootprintCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

        // ---- 全身：绝对位姿映射 ----
        void toggleAbsoluteTracking();
        void startAbsoluteTracking();
        void stopAbsoluteTracking();
        void resetAbsoluteTracking();
        bool wholeBodyTeleopReady() const;
        bool wholeBodyHeadTrackingReady() const;
        void publishWholeBodyHeadTarget();
        void stopWholeBodyHeadTracking();
        void publishBodyTarget();
        void publishAbsoluteArmTarget(bool is_left);
        bool captureAbsoluteTrackingMaps();
        bool humanFootprintTransform(Eigen::Isometry3d& T_H_world);
        void buildDeformedSkeleton(const Eigen::Isometry3d& T_H_world, DeformedSkeleton& out);
        /**
         * 读取「人体系 → 机体系」固定平移偏置，表达在 H 的三轴上。
         * 参数个数不是 3 或含非有限值时返回零向量（并在启动日志里已告警）。
         * @return [x, y, z] 米
         */
        Eigen::Vector3d humanToRobotOffsetInH() const;
        bool skeletonJointFresh(const std::chrono::steady_clock::time_point& stamp) const;
        bool geometryToleranceOk(bool is_left, const Eigen::Vector3d& hand_position) const;
        static Eigen::Quaterniond dropRoll(const Eigen::Quaterniond& q);

        /**
         * 布置四条接管渐入（左/右臂、胸、头）的起点。
         *
         * @param anchor_at_measured_pose
         *        true（默认，用于「开始跟踪」）：起点取机器人**当前实测位姿**，
         *              实测不可用时退回 1 s 内最后下发的目标；
         *        false（用于底盘锁定/解锁切换）：优先沿用最后下发的目标，
         *              保证切换瞬间目标严格连续。
         */
        void armAbsoluteRamps(bool anchor_at_measured_pose = true);
        void resetAbsoluteRamps();

        // ---- 全身：坐标系与锚定 ----
        bool resolveAnchorTransforms(const Eigen::Isometry3d& T_H_world,
                                     Eigen::Isometry3d& T_target_H,
                                     Eigen::Isometry3d& T_marker_H);
        bool basePoseInFrame(const std::string& frame, Eigen::Isometry3d& T_frame_base);
        std::string controlFrame() const;
        std::string markerFrame() const;
        void applyAnchorTransition();
        std::vector<std::string> missingSkeletonSources() const;

        // ---- 全身：与普通模式共享的状态 ----
        [[nodiscard]] bool vrEnabled() const { return enabled_.load(); }
        [[nodiscard]] int32_t fsmState() const noexcept;
        [[nodiscard]] bool fullBodyTopology() const { return isFullBodyMode(); }
        [[nodiscard]] bool eeFrameReady() const { return ee_frame_id_initialized_; }
        [[nodiscard]] bool baseUnlocked() const;
        [[nodiscard]] LastArmTarget lastArmTarget(bool is_left) const;
        bool transformPose(const std::string& label,
                           const std::string& source_frame,
                           const std::string& target_frame,
                           const Eigen::Vector3d& input_position,
                           const Eigen::Quaterniond& input_orientation,
                           Eigen::Vector3d& output_position,
                           Eigen::Quaterniond& output_orientation);

        // ---- 全身：配置与状态 ----
        AbsoluteTeleopOptions absolute_options_;
        VrSession session_;
        InputHub input_hub_;

        std::atomic<bool> whole_body_teleop_active_{false};
        std::atomic<bool> absolute_tracking_active_{false};

        bool anchor_valid_{false};
        bool anchor_base_unlocked_{false};
        Eigen::Isometry3d anchor_M_{Eigen::Isometry3d::Identity()};

        Eigen::Vector3d body_pelvis_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_pelvis_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d body_spine3_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_spine3_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d body_head_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_head_orientation_ = Eigen::Quaterniond::Identity();
        bool has_body_pelvis_pose_{false};
        bool has_body_spine3_pose_{false};
        bool has_body_head_pose_{false};

        Eigen::Vector3d body_left_hand_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_left_hand_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d body_right_hand_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_right_hand_orientation_ = Eigen::Quaterniond::Identity();
        bool has_body_left_hand_pose_{false};
        bool has_body_right_hand_pose_{false};

        SkeletonPoint body_left_shoulder_;
        SkeletonPoint body_right_shoulder_;

        Eigen::Vector3d human_footprint_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond human_footprint_orientation_ = Eigen::Quaterniond::Identity();
        std::chrono::steady_clock::time_point human_footprint_stamp_{};
        bool has_human_footprint_pose_{false};

        std::chrono::steady_clock::time_point body_pelvis_stamp_{};
        std::chrono::steady_clock::time_point body_spine3_stamp_{};
        std::chrono::steady_clock::time_point body_head_stamp_{};
        std::chrono::steady_clock::time_point body_left_hand_stamp_{};
        std::chrono::steady_clock::time_point body_right_hand_stamp_{};

        bool absolute_maps_captured_{false};
        bool settle_pending_{false};
        std::chrono::steady_clock::time_point settle_started_at_{};

        Eigen::Matrix3d abs_hand_to_ee_left_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d abs_hand_to_ee_right_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d abs_body_src_to_ee_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d abs_head_src_to_ee_ = Eigen::Matrix3d::Identity();
        double abs_left_arm_ref_len_{0.0};
        double abs_right_arm_ref_len_{0.0};

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

        arms_controller_common::PoseRamp left_arm_ramp_;
        arms_controller_common::PoseRamp right_arm_ramp_;
        arms_controller_common::PoseRamp body_ramp_;
        arms_controller_common::PoseRamp head_ramp_;

        bool wb_head_tracking_active_{false};
        std::optional<geometry_msgs::msg::PoseStamped> wb_last_head_target_;
        std::chrono::steady_clock::time_point wb_last_head_target_stamp_{};

        bool has_last_body_target_{false};
        Eigen::Vector3d last_body_target_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_body_target_orientation_ = Eigen::Quaterniond::Identity();
        std::chrono::steady_clock::time_point last_body_target_stamp_{};

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_pelvis_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_spine3_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_head_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_left_hand_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_right_hand_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_human_footprint_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_left_shoulder_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_right_shoulder_;
    };
} // namespace arms_ros2_control::command
