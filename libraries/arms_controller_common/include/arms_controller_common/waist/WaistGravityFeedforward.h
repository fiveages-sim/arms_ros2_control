#pragma once

//
// 腰部重力前馈 —— **两个控制器共用的唯一实现**。
//
// 为什么需要它（背景见 arms_ros2_control/README_zh.md「腰部电流环与两层重力补偿」）：
//   腰部进电流环后驱动不再自己保位，重力全靠两层前馈 —— ① 驱动内查表（保命层，标称手臂
//   姿态，~1°）；② 本话题（精度层，真姿态，误差≈0）。驱动**新鲜就用话题、过期退查表**。
//
// 谁发：
//   · ocs2_wbc_controller：全身模式。q = 它自己的 20 关节观测，模型 = 它自己的模型副本。
//   · ocs2_arm_controller：分体模式（没有 WBC）。模型从 /robot_description 建（它自己的
//     模型是双臂树，算不出腰力矩）；腰/头实测角从 /joint_states 取（它只 claim 手臂）。
//   两者不会同时跑（split 与 full_body 是两种启动方式）。
//
// **配置从哪来（重要）：一个参数都不用在 launch / profile 里写** —— 本模块订阅
// /robot_description（= xacro 的产物）自己派生：
//   · 关节名单 + 电机号 ← 驱动段的 <param name="current_motor_ids">（与 motor_ids 逐位对齐：
//     驱动 on_init 强制 motor_ids.size()==joints.size()，所以映射是确定的）
//   · 话题 + τ→电流系数  ← 同一驱动段的 <param name="gravity_ff_topic"/"gravity_ff_ma_per_nm">
//   · 模型              ← 同一个 URDF 字符串（pinocchio::urdf::buildModelFromXML）
// 于是「哪个电机进电流环」只改 xacro/profile 一处，上层自动跟随；仿真/无腰机型里没有
// current_motor_ids ⇒ 自动关闭（本来也没有驱动去消费这个话题）。
//
// 所以差异只有「模型从哪来」和「q 从哪来」两处，其余 —— 参数、末端负载折算、热更新、
// RNEA、降频、mA 换算、reliable 发布、就绪门控 —— 全部在这里，两边不再各写一份。
//
// 参数（两边同名同默认值，都由本类用调用方的 auto_declare 声明；**默认值即“自动”**，
// 只有测试/特殊机型才需要覆盖）：
//   waist_gravity_ff_joints        覆盖：要发的关节名（顺序与 motor_ids 对齐；默认空=自动派生）
//   waist_gravity_ff_motor_ids     覆盖：juxie 电机号（1 起）
//   waist_gravity_ff_topic         覆盖：发到哪个话题（默认空=用驱动段声明的 gravity_ff_topic）
//   waist_gravity_ff_ma_per_nm     覆盖：τ→电流（默认 -1=用驱动段的 gravity_ff_ma_per_nm，无则 30）
//   waist_gravity_ff_decimation    每 N 个 update 周期发一次（WBC 500 Hz→5；arm 100 Hz→1）
//   waist_gravity_ff_data_size     数组长度（驱动要求 ≥ max(motor id)，代码里会自动抬）
//   waist_gravity_ff_urdf_path     覆盖：直接指定模型文件（默认空=用调用方模型/URDF）
//   waist_gravity_ff_robot_description_topic  xacro 产物话题（默认 /robot_description）
//   waist_gravity_ff_joint_states_topic  自订阅 q 用的话题（默认 /joint_states；WBC 不用）
//   left_dyn_param / right_dyn_param / waist_payload_frames   末端负载（与驱动侧同源）
//
// RT 约定：publish()/publish(q) 只做一次 atomic load + 一次 RNEA + 一次 publish；
// 建模、负载快照、订阅回调全部在构造/回调线程（改参数经原子指针换快照，不阻塞运控线程）。
//

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace arms_controller_common
{
    class WaistGravityFeedforward
    {
    public:
        /**
         * 声明参数并开始等 /robot_description（拿到就从 xacro 产物里派生配置 + 建模型）。
         *
         * @param auto_declare       控制器的 auto_declare（模板： (name, default) → value）
         * @param caller_model       调用方自己的模型（WBC 传它的全机模型）；参数 waist_gravity_ff_urdf_path 优先
         * @param consumer_label     日志里的身份，如 "ocs2_wbc_controller" / "ocs2_arm_controller(分体代发)"
         * @param default_decimation 本控制器的 update 频率对应的降频默认值（WBC 500 Hz→5；arm 100 Hz→1）
         */
        template <typename AutoDeclareFunc>
        WaistGravityFeedforward(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                AutoDeclareFunc auto_declare,
                                const pinocchio::Model* caller_model = nullptr,
                                std::string consumer_label = "",
                                int64_t default_decimation = 1)
            : node_(node), consumer_(std::move(consumer_label))
        {
            waist_ff_joints_ = auto_declare("waist_gravity_ff_joints", std::vector<std::string>{});
            {
                const auto motors = auto_declare("waist_gravity_ff_motor_ids", std::vector<int64_t>{});
                waist_ff_motor_ids_.clear();
                waist_ff_motor_ids_.reserve(motors.size());
                for (const auto motor : motors)
                {
                    waist_ff_motor_ids_.push_back(static_cast<size_t>(motor));
                }
            }
            waist_ff_urdf_path_ = auto_declare("waist_gravity_ff_urdf_path", std::string(""));
            waist_ff_topic_ = auto_declare("waist_gravity_ff_topic", std::string(""));
            waist_ff_robot_description_topic_ = auto_declare(
                "waist_gravity_ff_robot_description_topic", std::string("/robot_description"));
            waist_ff_joint_states_topic_ = auto_declare("waist_gravity_ff_joint_states_topic",
                                                       std::string("/joint_states"));
            waist_ff_ma_per_nm_ = auto_declare("waist_gravity_ff_ma_per_nm", -1.0);
            waist_ff_decimation_ = static_cast<size_t>(
                std::max<int64_t>(1, auto_declare("waist_gravity_ff_decimation", default_decimation)));
            waist_ff_data_size_ = static_cast<size_t>(
                std::max<int64_t>(1, auto_declare("waist_gravity_ff_data_size", 6)));
            waist_payload_dyn_param_[0] = auto_declare("left_dyn_param", std::vector<double>{});
            waist_payload_dyn_param_[1] = auto_declare("right_dyn_param", std::vector<double>{});
            {
                const auto frames = auto_declare("waist_payload_frames", std::vector<std::string>{});
                waist_payload_frame_[0] = frames.size() > 0 && !frames[0].empty() ? frames[0] : "left_tcp";
                waist_payload_frame_[1] = frames.size() > 1 && !frames[1].empty() ? frames[1] : "right_tcp";
            }
            setup(caller_model);
        }

        /** 是否已就绪（参数齐全 + 模型建好）。未就绪时 publish() 是空操作。 */
        [[nodiscard]] bool enabled() const { return enabled_; }

        /**
         * 每拍调用（RT）：用**调用方给的 q**（与模型同序、长度 = model.nq）计算并发出去。
         * 内部按 decimation 降频；q 长度不符 / 未就绪时立刻返回。
         */
        void publish(const Eigen::VectorXd& q);

        /** 每拍调用（RT）：用内部从 /joint_states 攒的 q 发（没调 subscribeJointStates 就什么都不做）。 */
        void publish();

        /**
         * 可选：自订阅 /joint_states 填内部的 q（**只 claim 部分关节**的消费者用）。
         * 全部被跟踪的关节都收到过至少一次才会开始发（缺关节会让 RNEA 偏小 = 补偿不足）。
         */
        void subscribeJointStates(const std::string& topic);

        /** 含末端负载的静态力矩（调用方 q）：力控/合规等自算用得上。未启用时返回全 0。 */
        [[nodiscard]] Eigen::VectorXd staticTorques(const Eigen::VectorXd& q) const;

        /**
         * **不含**末端负载的静态力矩：`rnea(model, data, q, v=0, a=0)`。
         * 两个控制器原本各写一份一模一样的调用（给力控/合规算静态前馈）⇒ 收到这里。
         * data 由调用方缓存（每个模型一个），本函数不分配。
         */
        [[nodiscard]] static Eigen::VectorXd rneaStaticTorques(const pinocchio::Model& model,
                                                              pinocchio::Data& data,
                                                              const Eigen::VectorXd& q);

    private:
        /** 只在本线程（构造/参数回调）建模型；RT 只做一次 atomic load。模型已含末端负载。 */
        struct Snapshot
        {
            pinocchio::Model model;
            pinocchio::Data data;  // RNEA 会写它 ⇒ 每个快照自己一份
            explicit Snapshot(pinocchio::Model m) : model(std::move(m)), data(model) {}
        };

        void setup(const pinocchio::Model* caller_model);
        /** 从 /robot_description（xacro 产物）补齐未覆盖的配置；没有腰驱动器段就返回 false。 */
        bool deriveFromRobotDescription(const std::string& xml);
        /** 配置齐了就建模型、起发布者、打开功能。xml 为空表示“该来的都已经有了”。 */
        void configure(const std::string& xml);
        void registerParamCallback();
        void rebuild();        /** 还没收到的（必需的）关节名，拼成一句话给日志用。 */
        [[nodiscard]] std::string missingTrackedJoints() const;        /** 共用尾部：τ（含负载）→ mA 数组 → best_effort 发布。 */
        void publishTau(const Eigen::VectorXd& tau);
        bool allTrackedJointsSeen() const;

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::string consumer_;
        const pinocchio::Model* caller_model_{nullptr};

        // 参数
        std::vector<std::string> waist_ff_joints_;
        std::vector<size_t> waist_ff_motor_ids_;
        std::string waist_ff_urdf_path_;
        std::string waist_ff_topic_;
        std::string waist_ff_robot_description_topic_;
        std::string waist_ff_joint_states_topic_;
        double waist_ff_ma_per_nm_{-1.0};  // ≤0 = 用驱动段声明的 gravity_ff_ma_per_nm
        size_t waist_ff_decimation_{1};
        size_t waist_ff_data_size_{6};
        std::vector<double> waist_payload_dyn_param_[2];
        std::string waist_payload_frame_[2];

        // 运行时
        bool enabled_{false};
        bool configured_{false};
        std::string model_desc_;  // 日志用：模型到底从哪来
        std::vector<size_t> waist_ff_tau_index_;         // 每个关节在 τ 里的下标（模型确定后不变）
        std::optional<pinocchio::Model> base_model_;     // 无负载底模（重建用）
        mutable std::optional<pinocchio::Data> base_data_;  // 仅校核日志用（不带负载的那份）
        std::atomic<Snapshot*> snapshot_{nullptr};       // ⟵ 交接点
        std::vector<std::unique_ptr<Snapshot>> snapshots_;  // 所有权（退休的不在运行期释放）
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
        size_t cycle_{0};
        std::atomic_bool payload_delta_logged_{false};

        // q 来源二：/joint_states（逐关节 relaxed 原子；回调线程 store、RT load）
        std::vector<std::string> tracked_joints_;        // 模型里全部可动关节（槽位顺序）
        std::unordered_map<std::string, size_t> slot_by_joint_;
        std::unordered_map<std::string, char> state_position_joints_;  // URDF 导出了 /position 的关节
        std::unique_ptr<std::atomic<double>[]> staged_q_;
        std::vector<char> staged_seen_;                  // 只回调线程读写
        std::vector<char> staged_required_;              // 是否“必须等它”（同序，只回调线程读）
        size_t staged_required_count_{0};
        std::chrono::steady_clock::time_point js_first_time_{};
        bool js_wait_logged_{false};
        std::atomic_bool staged_ready_{false};
        Eigen::VectorXd q_buffer_;                       // 复用，避免 RT 每拍分配
    };
}
