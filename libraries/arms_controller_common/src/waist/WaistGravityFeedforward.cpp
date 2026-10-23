//
// 腰部重力前馈的唯一实现 —— 说明与参数含义见同名头文件。
//

#include <arms_controller_common/waist/WaistGravityFeedforward.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <sstream>
#include <utility>
#include <vector>

#include <hardware_interface/component_parser.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arms_controller_common
{
    namespace
    {
        double payload_mass(const std::vector<double>& dyn)
        {
            return dyn.empty() ? 0.0 : dyn[0];
        }

        /** 解析 "1, 2, 3" / "[1 2 3]" 这类电机号列表（驱动也只接受这种写法）。 */
        std::vector<size_t> parseIdList(const std::string& value)
        {
            std::vector<size_t> ids;
            std::string token;
            for (const char ch : value + ",")
            {
                if (ch >= '0' && ch <= '9')
                {
                    token += ch;
                    continue;
                }
                if (!token.empty())
                {
                    ids.push_back(static_cast<size_t>(std::strtoul(token.c_str(), nullptr, 10)));
                    token.clear();
                }
            }
            return ids;
        }

        double parseDoubleOr(const std::string& value, double fallback)
        {
            if (value.empty())
            {
                return fallback;
            }
            try
            {
                return std::stod(value);
            }
            catch (const std::exception&)
            {
                return fallback;
            }
        }
    }

    void WaistGravityFeedforward::setup(const pinocchio::Model* caller_model)
    {
        caller_model_ = caller_model;
        dynamic_arm_indices_.clear();
        if (dynamicEnabled())
        {
            // 只取双臂运动：腰部自身速度/加速度不反馈到附加力矩中。
            for (pinocchio::JointIndex j = 1;
                 j < static_cast<pinocchio::JointIndex>(caller_model_->njoints); ++j)
            {
                const auto& name = caller_model_->names[j];
                const auto& joint = caller_model_->joints[j];
                if ((name.rfind("left_joint", 0) == 0 || name.rfind("right_joint", 0) == 0) &&
                    joint.nv() == 1)
                {
                    dynamic_arm_indices_.push_back(joint.idx_v());
                }
            }
            if (dynamic_arm_indices_.empty())
            {
                throw std::invalid_argument("waist_dynamic_ff: model has no left_joint*/right_joint* arm joints");
            }
            dynamic_previous_velocity_ = Eigen::VectorXd::Zero(caller_model_->nv);
            dynamic_previous_acceleration_ = Eigen::VectorXd::Zero(caller_model_->nv);
            dynamic_velocity_ = Eigen::VectorXd::Zero(caller_model_->nv);
            dynamic_acceleration_ = Eigen::VectorXd::Zero(caller_model_->nv);
            dynamic_velocity_used_ = Eigen::VectorXd::Zero(caller_model_->nv);
            dynamic_acceleration_used_ = Eigen::VectorXd::Zero(caller_model_->nv);
            if (dynamic_accel_method_ == AccelMethod::SavitzkyGolay)
            {
                buildSgCoefficients();
                dynamic_sg_samples_.assign(dynamic_arm_indices_.size(),
                                           std::array<double, kMaxSgWindow>{});
            }
            resetDynamicState();
            RCLCPP_WARN(node_->get_logger(),
                        "腰部双臂惯性前馈启用：gain=%.3f, 每关节附加电流限幅=%.0f mA, "
                        "加速度估计=%s, 滤波=%.1f Hz, 加速度限幅=%.1f rad/s², 超前=%.1f ms；"
                        "需核对惯量与电流换算标定",
                        dynamic_gain_, dynamic_max_current_,
                        dynamic_accel_method_ == AccelMethod::SavitzkyGolay
                            ? ("sg(window=" + std::to_string(dynamic_sg_window_) + ")").c_str()
                            : "diff_lpf",
                        dynamic_filter_hz_, dynamic_max_acceleration_,
                        1000.0 * dynamic_advance_s_);
        }

        registerParamCallback();  // 与模型无关的参数（末端负载等）热更新

        // 覆盖参数给全了（测试 / 特殊机型：关节+电机+话题+模型来源都有）⇒ 不必等 URDF。
        if (!waist_ff_joints_.empty() && !waist_ff_topic_.empty() &&
            (!waist_ff_urdf_path_.empty() || caller_model_ != nullptr))
        {
            configure("");
            return;
        }

        // 否则从 /robot_description（xacro 产物）派生：关节名单/电机号/话题/模型全在里面，
        // 所以 launch 与 profile 一行配置都不用写。RSP 用 transient_local 发布 ⇒ 晚订阅也能收到。
        const std::string dr_topic = waist_ff_robot_description_topic_.empty()
                                         ? "/robot_description"
                                         : waist_ff_robot_description_topic_;
        robot_description_sub_ = node_->create_subscription<std_msgs::msg::String>(
            dr_topic, rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg && !configured_)
                {
                    configure(msg->data);
                }
            });
        RCLCPP_INFO(node_->get_logger(),
                    "腰部重力前馈：等 %s（从 xacro 产物里派生关节名单/电机号/话题，并用它建模型）…",
                    dr_topic.c_str());
    }

    bool WaistGravityFeedforward::deriveFromRobotDescription(const std::string& xml)
    {
        std::vector<hardware_interface::HardwareInfo> infos;
        try
        {
            infos = hardware_interface::parse_control_resources_from_urdf(xml);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "腰部重力前馈：解析 robot_description 失败：%s", e.what());
            return false;
        }

        for (const auto& info : infos)
        {
            const auto param = [&info](const std::string& key) -> std::string {
                const auto it = info.hardware_parameters.find(key);
                return it == info.hardware_parameters.end() ? std::string("") : it->second;
            };
            const std::string current_ids = param("current_motor_ids");
            if (current_ids.empty())
            {
                continue;  // 不是「腰部能进电流环」的那套驱动
            }
            const std::string topic = param("gravity_ff_topic");
            if (topic.empty())
            {
                continue;  // 驱动没订阅这个话题 ⇒ 发了也没人用
            }

            const std::vector<size_t> motor_ids = parseIdList(param("motor_ids"));
            const std::vector<size_t> current = parseIdList(current_ids);
            std::vector<std::string> joints;
            std::vector<size_t> joint_motors;
            for (size_t i = 0; i < info.joints.size(); ++i)
            {
                // <joint> 与 motor_ids 逐位对齐（驱动 on_init 强制两者数量相等）；
                // 没写 motor_ids 时按 1..N 计（与 xacro 的默认写法一致）。
                const size_t motor = i < motor_ids.size() ? motor_ids[i] : i + 1;
                if (std::find(current.begin(), current.end(), motor) == current.end())
                {
                    continue;
                }
                joints.push_back(info.joints[i].name);
                joint_motors.push_back(motor);
            }
            if (joints.empty())
            {
                continue;
            }
            // 记下这个 URDF 真的导出了 position 状态接口的关节（= /joint_states 里会有的那些）。
            for (const auto& other : infos)
            {
                for (const auto& joint : other.joints)
                {
                    if (joint.is_mimic != hardware_interface::MimicAttribute::NOT_SET)
                    {
                        continue;  // mimic 关节由另一个关节驱动，不该要求它自己的反馈
                    }
                    for (const auto& iface : joint.state_interfaces)
                    {
                        if (iface.name == "position")
                        {
                            state_position_joints_[joint.name] = 1;
                            break;
                        }
                    }
                }
            }

            if (waist_ff_joints_.empty())
            {
                waist_ff_joints_ = joints;
                waist_ff_motor_ids_ = joint_motors;
            }
            if (waist_ff_topic_.empty())
            {
                waist_ff_topic_ = topic;
            }
            if (waist_ff_ma_per_nm_ <= 0.0)
            {
                // τ→电流系数：与驱动侧那张查表的标定同源（都在描述包里），没写就沿用 30。
                const double ma = parseDoubleOr(param("gravity_ff_ma_per_nm"), 30.0);
                waist_ff_ma_per_nm_ = ma > 0.0 ? ma : 30.0;
            }

            std::string list;
            for (size_t k = 0; k < waist_ff_joints_.size(); ++k)
            {
                list += (k == 0 ? "" : ", ") + waist_ff_joints_[k] + "→motor" +
                        std::to_string(waist_ff_motor_ids_[k]);
            }
            RCLCPP_INFO(node_->get_logger(),
                        "腰部重力前馈：从 robot_description 的 '%s' 段派生（current_motor_ids=%s, "
                        "话题=%s, ma=%.1f mA/(N·m)）→ [%s]",
                        info.name.c_str(), current_ids.c_str(), waist_ff_topic_.c_str(),
                        waist_ff_ma_per_nm_, list.c_str());
            return true;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "腰部重力前馈：robot_description 里没有声明 current_motor_ids 的硬件"
                    "（仿真 / 无 juxie 腰的机型）⇒ 关闭");
        return false;
    }

    void WaistGravityFeedforward::configure(const std::string& xml)
    {
        if (configured_)
        {
            return;
        }
        if (!xml.empty() && !deriveFromRobotDescription(xml))
        {
            return;  // 这个 URDF 里没有腰 ⇒ 保持关闭
        }
        if (waist_ff_joints_.empty() || waist_ff_topic_.empty())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "腰部重力前馈配置不完整（joints=%zu 个, topic='%s'）⇒ 不发布",
                        waist_ff_joints_.size(), waist_ff_topic_.c_str());
            return;
        }
        if (waist_ff_joints_.size() != waist_ff_motor_ids_.size())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "waist_gravity_ff_joints(%zu) 与 waist_gravity_ff_motor_ids(%zu) 长度不一致 ⇒ 不发布",
                        waist_ff_joints_.size(), waist_ff_motor_ids_.size());
            return;
        }

        if (!waist_ff_urdf_path_.empty())
        {
            try
            {
                pinocchio::Model model;
                pinocchio::urdf::buildModel(waist_ff_urdf_path_, model);
                base_model_ = std::move(model);
                model_desc_ = waist_ff_urdf_path_;
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node_->get_logger(), "腰部重力前馈：解析 URDF 失败（%s）：%s",
                             waist_ff_urdf_path_.c_str(), e.what());
                return;
            }
        }
        else if (caller_model_ != nullptr)
        {
            base_model_ = *caller_model_;  // 副本：绝不动调用方的模型（约束/动力学/自碰撞都靠它）
            model_desc_ = "调用方模型";
        }
        else if (!xml.empty())
        {
            try
            {
                pinocchio::Model model;
                pinocchio::urdf::buildModelFromXML(xml, model, false, false);
                base_model_ = std::move(model);
                model_desc_ = "robot_description（同一份 URDF 字符串）";
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "腰部重力前馈：从 robot_description 建模型失败：%s", e.what());
                return;
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "腰部重力前馈：没有模型来源（urdf_path 空 / 调用方没给模型 / 没有 robot_description）"
                        "⇒ 不发布");
            return;
        }

        for (const auto& joint : waist_ff_joints_)
        {
            if (!base_model_->existJointName(joint))
            {
                RCLCPP_WARN(node_->get_logger(),
                            "腰部重力前馈：'%s' 不在模型里 ⇒ 不发布（检查 URDF / 关节名）", joint.c_str());
                return;
            }
        }

        // 驱动要求数组长度 ≥ max(电机 ID)（不足会**整条丢弃**并继续用查表）。
        size_t max_motor = 0;
        for (const auto motor : waist_ff_motor_ids_)
        {
            max_motor = std::max(max_motor, motor);
        }
        if (waist_ff_data_size_ < max_motor)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "waist_gravity_ff_data_size(%zu) < 最大电机 ID(%zu) ⇒ 抬到 %zu（否则驱动整条丢弃）",
                        waist_ff_data_size_, max_motor, max_motor);
            waist_ff_data_size_ = max_motor;
        }

        // τ 下标与 q 槽位：模型确定后不再变（改参数只重建负载，不换模型）。
        waist_ff_tau_index_.clear();
        for (const auto& joint : waist_ff_joints_)
        {
            const auto jid = base_model_->getJointId(joint);
            waist_ff_tau_index_.push_back(static_cast<size_t>(base_model_->joints[jid].idx_v()));
        }
        tracked_joints_.clear();
        slot_by_joint_.clear();
        for (size_t j = 1; j < static_cast<size_t>(base_model_->njoints); ++j)
        {
            slot_by_joint_[base_model_->names[j]] = tracked_joints_.size();
            tracked_joints_.push_back(base_model_->names[j]);
        }
        staged_q_ = std::make_unique<std::atomic<double>[]>(tracked_joints_.size());
        for (size_t i = 0; i < tracked_joints_.size(); ++i)
        {
            staged_q_[i].store(0.0, std::memory_order_relaxed);
        }
        staged_seen_.assign(tracked_joints_.size(), 0);
        // 门槛只要求「URDF 真的导出了 position 状态接口」的关节 —— /joint_states 里只会有它们
        //（JSB 发布的正是各硬件导出的 state interface）。否则像轮子/被动末端那种「URDF 里有
        // 可动关节、但没有硬件导出」的关节会永远收不到 ⇒ 整个功能静默不工作。
        staged_required_.assign(tracked_joints_.size(), 1);
        if (!state_position_joints_.empty())
        {
            size_t required = 0;
            for (size_t i = 0; i < tracked_joints_.size(); ++i)
            {
                const bool exported = state_position_joints_.count(tracked_joints_[i]) > 0;
                staged_required_[i] = exported ? 1 : 0;
                required += exported ? 1 : 0;
            }
            staged_required_count_ = required;
        }
        else
        {
            staged_required_count_ = tracked_joints_.size();
        }
        q_buffer_ = Eigen::VectorXd::Zero(base_model_->nq);

        // QoS 用 reliable（depth 1）：驱动的订阅是 best_effort —— reliable 发布者**能**喂
        // best_effort 订阅者（反过来不行），而且默认 QoS 的 `ros2 topic echo` 也能看到这个话题。
        pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(waist_ff_topic_, rclcpp::QoS(1));
        if (dynamicEnabled())
        {
            auto qos = rclcpp::QoS(1).best_effort();
            qos.lifespan(rclcpp::Duration::from_seconds(0.02));
            dynamic_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                waist_ff_topic_ + "/dynamic", qos);
        }
        // 调用方不给 q 的消费者（arm 控制器只 claim 手臂）才自订阅 /joint_states。
        if (caller_model_ == nullptr && !waist_ff_joint_states_topic_.empty())
        {
            subscribeJointStates(waist_ff_joint_states_topic_);
        }

        configured_ = true;
        enabled_ = true;
        rebuild();

        std::string desc;
        for (size_t k = 0; k < waist_ff_joints_.size(); ++k)
        {
            desc += (k == 0 ? "" : ", ") + waist_ff_joints_[k] + "→motor" +
                    std::to_string(waist_ff_motor_ids_[k]);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "腰部精确重力前馈已启用%s：topic=%s, ma=%.1f mA/(N·m), 每 %zu 周期发一次, "
                    "模型=%s（%zu 个可动关节）, %s, [%s]"
                    "（驱动侧新鲜就用它、过期退回自带查表）",
                    consumer_.empty() ? "" : ("（" + consumer_ + "）").c_str(), waist_ff_topic_.c_str(),
                    waist_ff_ma_per_nm_, waist_ff_decimation_, model_desc_.c_str(),
                    tracked_joints_.size(),
                    caller_model_ != nullptr
                        ? "q 由调用方提供"
                        : ("q 来自 " + waist_ff_joint_states_topic_).c_str(),
                    desc.c_str());
    }

    void WaistGravityFeedforward::registerParamCallback()
    {
        // 参数回调挂在**本节点**上（CM 的 executor 线程），与 update() 不是同一个线程 ⇒
        // 快照重建只在这里发生，RT 侧只做一次原子读。
        param_callback_ = node_->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                // 动态前馈配置只在构造时读取；拒绝运行中突变，也不在回调线程改 RT 滤波状态。
                for (const auto& param : parameters)
                {
                    if (param.get_name().rfind("waist_dynamic_ff_", 0) == 0)
                    {
                        result.successful = false;
                        result.reason = "waist_dynamic_ff parameters require controller reload";
                        return result;
                    }
                }
                bool payload_changed = false;
                for (const auto& param : parameters)
                {
                    if (param.get_name() == "left_dyn_param" || param.get_name() == "right_dyn_param")
                    {
                        waist_payload_dyn_param_[param.get_name() == "left_dyn_param" ? 0 : 1] =
                            param.as_double_array();
                        payload_changed = true;
                    }
                    else if (param.get_name() == "waist_payload_frames")
                    {
                        const auto frames = param.as_string_array();
                        waist_payload_frame_[0] =
                            frames.size() > 0 && !frames[0].empty() ? frames[0] : "left_tcp";
                        waist_payload_frame_[1] =
                            frames.size() > 1 && !frames[1].empty() ? frames[1] : "right_tcp";
                        payload_changed = true;
                    }
                    else if (param.get_name() == "waist_gravity_ff_ma_per_nm")
                    {
                        waist_ff_ma_per_nm_ = param.as_double();
                    }
                    else if (param.get_name() == "waist_gravity_ff_decimation")
                    {
                        waist_ff_decimation_ = static_cast<size_t>(std::max<int64_t>(1, param.as_int()));
                    }
                }
                if (payload_changed && enabled_)
                {
                    rebuild();
                    RCLCPP_INFO(node_->get_logger(),
                                "腰部重力前馈的末端负载已更新（改 dyn_param 立即生效，不阻塞运控线程）");
                }
                return result;
            });
    }

    void WaistGravityFeedforward::rebuild()
    {
        if (!base_model_)
        {
            return;
        }
        auto fresh = std::make_unique<Snapshot>(*base_model_);
        for (size_t side = 0; side < 2; ++side)
        {
            const std::vector<double>& dyn = waist_payload_dyn_param_[side];
            const std::string& frame = waist_payload_frame_[side];
            if (payload_mass(dyn) <= 0.0)
            {
                continue;  // 该侧不折算
            }
            if (dyn.size() < 4 || !fresh->model.existFrame(frame))
            {
                RCLCPP_WARN(node_->get_logger(),
                            "忽略 %s 侧末端负载（dyn_param %zu 项 / 帧 '%s' 不在模型里）",
                            side ? "right" : "left", dyn.size(), frame.c_str());
                continue;
            }
            // 等效质量点：静态重力只看质量+质心（惯量用零矩阵）；质心 mm→m，转到父关节系。
            const auto fid = fresh->model.getFrameId(frame);
            const auto jid = fresh->model.frames[fid].parentJoint;
            const Eigen::Vector3d com(dyn[1] * 1e-3, dyn[2] * 1e-3, dyn[3] * 1e-3);
            fresh->model.inertias[jid] = fresh->model.inertias[jid] +
                pinocchio::Inertia(payload_mass(dyn), fresh->model.frames[fid].placement.act(com),
                                   Eigen::Matrix3d::Zero());
            RCLCPP_INFO(node_->get_logger(),
                        "已折算 %s 末端负载 m=%.3f kg, com=[%.1f %.1f %.1f] mm (%s) → '%s'",
                        side ? "right" : "left", payload_mass(dyn), dyn[1], dyn[2], dyn[3],
                        frame.c_str(), fresh->model.names[jid].c_str());
        }

        // 入所有权表（对象不随 vector 扩容移动）→ 原子换指针；旧快照不在运行期释放
        //（RT 线程可能刚 load 到它）。改参是人/脚本级频率 ⇒ 有界。
        Snapshot* ptr = fresh.get();
        snapshots_.emplace_back(std::move(fresh));
        snapshot_.store(ptr, std::memory_order_release);
        payload_delta_logged_.store(false, std::memory_order_relaxed);  // 让校核日志再打一次
    }

    void WaistGravityFeedforward::subscribeJointStates(const std::string& topic)
    {
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            topic, rclcpp::QoS(1),
            [this, topic](sensor_msgs::msg::JointState::SharedPtr msg) {
                if (!msg || !enabled_)
                {
                    return;
                }
                for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k)
                {
                    const auto it = slot_by_joint_.find(msg->name[k]);
                    if (it == slot_by_joint_.end())
                    {
                        continue;
                    }
                    staged_q_[it->second].store(msg->position[k], std::memory_order_relaxed);
                    staged_seen_[it->second] = 1;
                }
                if (staged_ready_.load(std::memory_order_relaxed))
                {
                    return;
                }
                if (js_first_time_.time_since_epoch().count() == 0)
                {
                    js_first_time_ = std::chrono::steady_clock::now();
                }
                if (allTrackedJointsSeen())
                {
                    staged_ready_.store(true, std::memory_order_release);
                    RCLCPP_INFO(node_->get_logger(),
                                "腰部重力前馈：已从 %s 收齐 %zu 个关节角，开始发布",
                                topic.c_str(), staged_required_count_);
                }
                else if (!js_wait_logged_ &&
                         std::chrono::steady_clock::now() - js_first_time_ > std::chrono::seconds(2))
                {
                    // 只报一次：等不到关节时到底缺谁（不然这个功能会静静地什么都不干）
                    js_wait_logged_ = true;
                    RCLCPP_WARN(node_->get_logger(),
                                "腰部重力前馈：%s 已出数据，但还缺 %s（共需 %zu 个）⇒ 暂不发布"
                                "（缺角的关节会用中立位置算 ⇒ 补偿会偏小，所以宁可等）",
                                topic.c_str(), missingTrackedJoints().c_str(), staged_required_count_);
                }
            });
    }

    bool WaistGravityFeedforward::allTrackedJointsSeen() const
    {
        for (size_t i = 0; i < staged_seen_.size(); ++i)
        {
            if (staged_required_[i] != 0 && staged_seen_[i] == 0)
            {
                return false;
            }
        }
        return true;
    }

    std::string WaistGravityFeedforward::missingTrackedJoints() const
    {
        std::string missing;
        for (size_t i = 0; i < staged_seen_.size(); ++i)
        {
            if (staged_required_[i] != 0 && staged_seen_[i] == 0)
            {
                if (missing.size() > 200)
                {
                    missing += " …";
                    break;
                }
                missing += (missing.empty() ? "" : ", ") + tracked_joints_[i];
            }
        }
        return missing;
    }

    Eigen::VectorXd WaistGravityFeedforward::rneaStaticTorques(const pinocchio::Model& model,
                                                              pinocchio::Data& data,
                                                              const Eigen::VectorXd& q)
    {
        return pinocchio::rnea(model, data, q, Eigen::VectorXd::Zero(model.nv),
                               Eigen::VectorXd::Zero(model.nv));
    }

    Eigen::VectorXd WaistGravityFeedforward::staticTorques(const Eigen::VectorXd& q) const
    {
        // 只做一次原子读；快照由写者线程建好，本线程不分配、不加锁。
        auto* snap = snapshot_.load(std::memory_order_acquire);
        if (snap == nullptr || q.size() != snap->model.nq)
        {
            return Eigen::VectorXd::Zero(base_model_ ? base_model_->nv : 0);
        }
        return rneaStaticTorques(snap->model, snap->data, q);
    }

    void WaistGravityFeedforward::publish(const Eigen::VectorXd& q)
    {
        publishState(q, false);
    }

    void WaistGravityFeedforward::resetDynamicState()
    {
        dynamic_seeded_ = false;
        dynamic_velocity_.setZero();
        dynamic_acceleration_.setZero();
        dynamic_previous_acceleration_.setZero();
        dynamic_velocity_used_.setZero();
        dynamic_acceleration_used_.setZero();
        dynamic_sg_filled_ = 0;
        for (auto& samples : dynamic_sg_samples_)
        {
            samples.fill(0.0);
        }
    }

    void WaistGravityFeedforward::publish(const Eigen::VectorXd& q,
                                          const Eigen::VectorXd& measured_velocity, double dt)
    {
        bool with_dynamics = false;
        if (enabled_ && dynamicEnabled())
        {
            // 控制循环中断/无效采样后重新种下初值，不跨越停顿做差分/拟合。
            if (!base_model_ || q.size() != base_model_->nq || !q.allFinite() ||
                measured_velocity.size() != base_model_->nv || !measured_velocity.allFinite() ||
                !std::isfinite(dt) || dt <= 0.0 || dt > 0.05)
            {
                resetDynamicState();
            }
            else if (dynamic_accel_method_ == AccelMethod::SavitzkyGolay)
            {
                with_dynamics = updateDynamicsSavitzkyGolay(measured_velocity, dt);
            }
            else
            {
                with_dynamics = updateDynamicsDiffLpf(measured_velocity, dt);
            }
        }
        publishState(q, with_dynamics);
    }

    void WaistGravityFeedforward::buildSgCoefficients()
    {
        // 时间轴取 -m..m 的整数采样点（等间隔），拟合 y = c0 + c1 t + c2 t²。
        // 最小二乘伪逆 (AᵀA)⁻¹Aᵀ 的每一行就是 c0/c1/c2 的权重；setup() 里算一次，RT 只用点积。
        const size_t window = static_cast<size_t>(dynamic_sg_window_);
        const double half = 0.5 * static_cast<double>(window - 1);
        Eigen::MatrixXd a(static_cast<Eigen::Index>(window), 3);
        for (size_t k = 0; k < window; ++k)
        {
            const double t = static_cast<double>(k) - half;
            a(static_cast<Eigen::Index>(k), 0) = 1.0;
            a(static_cast<Eigen::Index>(k), 1) = t;
            a(static_cast<Eigen::Index>(k), 2) = t * t;
        }
        const Eigen::Matrix3d normal = a.transpose() * a;
        dynamic_sg_coeffs_ = normal.ldlt().solve(a.transpose());
    }

    bool WaistGravityFeedforward::updateDynamicsDiffLpf(const Eigen::VectorXd& measured_velocity,
                                                        double dt)
    {
        if (!dynamic_seeded_)
        {
            dynamic_previous_velocity_ = measured_velocity;
            dynamic_seeded_ = true;
            return false;
        }
        const double alpha = -std::expm1(-2.0 * 3.14159265358979323846 * dynamic_filter_hz_ * dt);
        const double advance = dynamic_advance_s_;
        for (const auto i : dynamic_arm_indices_)
        {
            const double acceleration = std::clamp(
                (measured_velocity[i] - dynamic_previous_velocity_[i]) / dt,
                -dynamic_max_acceleration_, dynamic_max_acceleration_);
            dynamic_acceleration_[i] += alpha * (acceleration - dynamic_acceleration_[i]);
            dynamic_velocity_[i] += alpha * (measured_velocity[i] - dynamic_velocity_[i]);
            dynamic_previous_velocity_[i] = measured_velocity[i];
            // 前馈链路自身滞后（一阶低通 + 降频发布 + 发帧）约 20~25 ms；
            // 按一阶泰勒把速度/加速度外推到“真正作用时刻”，advance=0 时即低通值。
            if (advance > 0.0)
            {
                const double a_dot =
                    (dynamic_acceleration_[i] - dynamic_previous_acceleration_[i]) / dt;
                dynamic_acceleration_used_[i] = std::clamp(
                    dynamic_acceleration_[i] + advance * a_dot,
                    -dynamic_max_acceleration_, dynamic_max_acceleration_);
                dynamic_velocity_used_[i] = dynamic_velocity_[i] + advance * dynamic_acceleration_[i];
            }
            else
            {
                dynamic_acceleration_used_[i] = dynamic_acceleration_[i];
                dynamic_velocity_used_[i] = dynamic_velocity_[i];
            }
            dynamic_previous_acceleration_[i] = dynamic_acceleration_[i];
        }
        return true;
    }

    bool WaistGravityFeedforward::updateDynamicsSavitzkyGolay(
        const Eigen::VectorXd& measured_velocity, double dt)
    {
        const size_t window = static_cast<size_t>(dynamic_sg_window_);
        const size_t joints = dynamic_arm_indices_.size();
        if (dynamic_sg_samples_.size() != joints)
        {
            dynamic_sg_samples_.assign(joints, std::array<double, kMaxSgWindow>{});
            dynamic_sg_filled_ = 0;
        }
        // 每个手臂关节滚动窗口：丢掉最旧、追加本拍实测速度。
        for (size_t j = 0; j < joints; ++j)
        {
            auto& buf = dynamic_sg_samples_[j];
            std::copy(buf.begin() + 1, buf.begin() + static_cast<std::ptrdiff_t>(window), buf.begin());
            buf[window - 1] = measured_velocity[dynamic_arm_indices_[j]];
        }
        if (++dynamic_sg_filled_ < window)
        {
            return false;  // 窗口未满：先只发静态重力
        }
        const double advance = dynamic_advance_s_;
        for (size_t j = 0; j < joints; ++j)
        {
            const auto& buf = dynamic_sg_samples_[j];
            double c0 = 0.0;
            double c1 = 0.0;
            double c2 = 0.0;
            for (size_t k = 0; k < window; ++k)
            {
                const Eigen::Index idx = static_cast<Eigen::Index>(k);
                const double y = buf[k];
                c0 += dynamic_sg_coeffs_(0, idx) * y;
                c1 += dynamic_sg_coeffs_(1, idx) * y;
                c2 += dynamic_sg_coeffs_(2, idx) * y;
            }
            const Eigen::Index i = dynamic_arm_indices_[j];
            const double smoothed_velocity = c0;
            const double acceleration =
                std::clamp(c1 / dt, -dynamic_max_acceleration_, dynamic_max_acceleration_);
            dynamic_velocity_[i] = smoothed_velocity;
            dynamic_acceleration_[i] = acceleration;
            if (advance > 0.0)
            {
                // 二次拟合的 c2 直接给出加速度导数，不用再对加速度差分一次。
                const double a_dot = 2.0 * c2 / (dt * dt);
                dynamic_acceleration_used_[i] = std::clamp(
                    acceleration + advance * a_dot,
                    -dynamic_max_acceleration_, dynamic_max_acceleration_);
                dynamic_velocity_used_[i] = smoothed_velocity + advance * acceleration;
            }
            else
            {
                dynamic_acceleration_used_[i] = acceleration;
                dynamic_velocity_used_[i] = smoothed_velocity;
            }
        }
        return true;
    }

    void WaistGravityFeedforward::publishState(const Eigen::VectorXd& q, bool with_dynamics)
    {
        if (!enabled_)
        {
            return;
        }
        if (++cycle_ < waist_ff_decimation_)
        {
            return;
        }
        cycle_ = 0;
        if (q.size() != (base_model_ ? base_model_->nq : 0) || !q.allFinite())
        {
            return;  // 观测还没建好（激活首周期）/ 长度不符
        }
        auto* snap = snapshot_.load(std::memory_order_acquire);
        if (snap == nullptr)
        {
            return;
        }
        Eigen::VectorXd tau = rneaStaticTorques(snap->model, snap->data, q);
        // 一次性校核：负载让腰部 τ 变了多少（换 dyn_param 会重打）
        if (snapshot_.load(std::memory_order_relaxed) != nullptr && base_model_ &&
            !payload_delta_logged_.exchange(true, std::memory_order_relaxed))
        {
            Eigen::VectorXd tau_without;
            if (!base_data_)
            {
                base_data_.emplace(*base_model_);
            }
            tau_without = pinocchio::rnea(*base_model_, *base_data_, q,
                                          Eigen::VectorXd::Zero(base_model_->nv),
                                          Eigen::VectorXd::Zero(base_model_->nv));
            double d_max = 0.0;
            for (const size_t ti : waist_ff_tau_index_)
            {
                if (ti < static_cast<size_t>(tau.size()) && ti < static_cast<size_t>(tau_without.size()))
                {
                    d_max = std::max(d_max, std::fabs(tau[ti] - tau_without[ti]));
                }
            }
            RCLCPP_INFO(node_->get_logger(),
                        "负载折算校核：腰部最大 Δτ=%.2f N·m ⇒ 最多多补 %.0f mA（当前姿态）",
                        d_max, d_max * waist_ff_ma_per_nm_);
        }
        Eigen::VectorXd dynamic_tau;
        if (with_dynamics)
        {
            // RNEA = M(q) a + C(q,v) v + g(q)；先减同一快照的 g，避免重复补重力。
            dynamic_tau =
                pinocchio::rnea(snap->model, snap->data, q, dynamic_velocity_used_,
                                dynamic_acceleration_used_) - tau;
        }
        publishTau(tau, with_dynamics && dynamic_tau.allFinite() ? &dynamic_tau : nullptr);
    }

    void WaistGravityFeedforward::publish()
    {
        if (!enabled_ || joint_state_sub_ == nullptr)
        {
            return;  // 没订阅 /joint_states ⇒ 这条路径无数据
        }
        if (!staged_ready_.load(std::memory_order_acquire))
        {
            return;
        }
        if (++cycle_ < waist_ff_decimation_)
        {
            return;
        }
        cycle_ = 0;
        q_buffer_.setZero();  // neutral = 全 0（模型里没有需要非 0 的“静止位”）
        for (size_t i = 0; i < tracked_joints_.size(); ++i)
        {
            const auto jid = base_model_->getJointId(tracked_joints_[i]);
            q_buffer_[base_model_->joints[jid].idx_q()] = staged_q_[i].load(std::memory_order_relaxed);
        }
        publishTau(staticTorques(q_buffer_));
    }

    void WaistGravityFeedforward::publishTau(const Eigen::VectorXd& tau, const Eigen::VectorXd* dynamic_tau)
    {
        if (tau.size() == 0 || waist_ff_tau_index_.size() != waist_ff_motor_ids_.size())
        {
            return;
        }
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(waist_ff_data_size_, 0.0);  // 下标 = 电机 ID − 1
        for (size_t k = 0; k < waist_ff_tau_index_.size(); ++k)
        {
            const size_t motor = waist_ff_motor_ids_[k];
            const size_t ti = waist_ff_tau_index_[k];
            if (motor < 1 || motor > msg.data.size() || ti >= static_cast<size_t>(tau.size()))
            {
                continue;
            }
            msg.data[motor - 1] = tau[ti] * waist_ff_ma_per_nm_;
        }
        pub_->publish(msg);
        if (dynamic_pub_ != nullptr)
        {
            std::fill(msg.data.begin(), msg.data.end(), 0.0);
            if (dynamic_tau != nullptr && std::isfinite(waist_ff_ma_per_nm_) && waist_ff_ma_per_nm_ > 0.0)
            {
                for (size_t k = 0; k < waist_ff_tau_index_.size(); ++k)
                {
                    const size_t motor = waist_ff_motor_ids_[k];
                    const size_t ti = waist_ff_tau_index_[k];
                    if (motor >= 1 && motor <= msg.data.size() && ti < static_cast<size_t>(dynamic_tau->size()))
                        msg.data[motor - 1] = std::clamp(dynamic_gain_ * (*dynamic_tau)[ti] * waist_ff_ma_per_nm_,
                                                       -dynamic_max_current_, dynamic_max_current_);
                }
            }
            dynamic_pub_->publish(msg);
        }
    }
}
