//
// 信息输入面：所有设备的数据先汇到一处，再被映射读取。
//
// 作用：
//   把左右手、头显、摇杆、扳机、骨架各关节、人体基坐标系统一落库，
//   并回答三个问题：这类数据收到过吗（seen）、现在还新吗（fresh）、
//   本次操作需要的一组能力是否齐备（requireFresh）。
//
// 好处：
//   1. 映射不再依赖“哪个回调先到”，左右手谁先发不影响结果；
//   2. 新鲜度、缺失、可用能力统一回答，准入判断只有一个实现；
//   3. 新增关节只是多一个枚举值，不影响已有调用点。
//
// 注意：
//   时间戳由 set 系列函数盖章（不用消息自带的时间），
//   因此“新鲜”反映的是“多久没收到”，而不是设备自己认为过了多久。
//
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "arms_target_manager/core/TeleopTypes.h"

namespace arms_ros2_control::command
{
    /** 能力位：用来回答“这套设备现在能提供什么”。 */
    enum class InputCap : uint32_t
    {
        HandLeft = 1u << 0,
        HandRight = 1u << 1,
        HeadPose = 1u << 2,
        Sticks = 1u << 3,
        Triggers = 1u << 4,
        Grips = 1u << 5,
        SkeletonCore = 1u << 6,
        SkeletonArms = 1u << 7,
        Footprint = 1u << 8,
        SkeletonLegs = 1u << 9,   // 预留
        Count = 10,
    };

    using Caps = uint32_t;

    constexpr Caps capOf(InputCap cap)
    {
        return static_cast<Caps>(cap);
    }

    /** 骨架关节。新增关节在这里加一项即可。 */
    enum class Joint : uint8_t
    {
        Pelvis = 0,
        Spine3,
        Head,
        ShoulderLeft,
        ShoulderRight,
        HandLeft,
        HandRight,
        Count,
    };

    /** 单次位姿采样：位置 + 姿态 + 置信度 + 时间戳。 */
    struct PoseSample
    {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
        float confidence{1.0f};
        bool valid{false};
        std::chrono::steady_clock::time_point stamp{};

        /** 数值可用：标记为有效且所有分量都是有限值。 */
        [[nodiscard]] bool usable() const
        {
            return valid && position.allFinite() && orientation.coeffs().allFinite();
        }
    };

    /** 摇杆采样：左右轴值 + 两个握把的实时状态。 */
    struct StickSample
    {
        Eigen::Vector2d left{Eigen::Vector2d::Zero()};
        Eigen::Vector2d right{Eigen::Vector2d::Zero()};
        bool left_grip{false};
        bool right_grip{false};
        std::chrono::steady_clock::time_point stamp{};
    };

    /** 扳机采样：0~1 的拉开比例。 */
    struct TriggerSample
    {
        double left{0.0};
        double right{0.0};
        std::chrono::steady_clock::time_point stamp{};
    };

    class InputHub
    {
    public:
        /**
         * @brief 构造输入汇聚
         * @param timeout_sec 数据新鲜度阈值 [s]，超过即视为过期
         */
        explicit InputHub(double timeout_sec = 0.2)
            : timeout_sec_(timeout_sec)
        {
        }

        void setTimeout(double seconds) { timeout_sec_ = seconds; }

        // ---- 上报 ----
        void setHand(bool is_left, const PoseSample& pose);
        void setHead(const PoseSample& pose);
        void setSticks(const StickSample& sticks);
        void setTriggers(const TriggerSample& triggers);
        void setJoint(Joint joint, const PoseSample& pose);
        void setFootprint(const PoseSample& pose);

        // ---- 读取 ----
        [[nodiscard]] const PoseSample& hand(bool is_left) const;
        [[nodiscard]] const PoseSample& head() const;
        [[nodiscard]] const StickSample& sticks() const;
        [[nodiscard]] const TriggerSample& triggers() const;
        [[nodiscard]] const PoseSample& joint(Joint joint) const;
        [[nodiscard]] const PoseSample& footprint() const { return footprint_; }

        /** 这一类数据至少上报过一次（不考虑新鲜度）。 */
        [[nodiscard]] bool seen(InputCap cap) const;
        /** 这一类数据当前仍然新鲜（在超时阈值内）。 */
        [[nodiscard]] bool fresh(InputCap cap) const;
        /** 当前仍然新鲜的能力位集合。 */
        [[nodiscard]] Caps available() const;

        /**
         * 检查一组能力是否齐备；不齐时把中文说明填进 missing。
         * 全身模式准入与标定前检查共用这一份实现。
         */
        [[nodiscard]] bool requireFresh(Caps required, std::vector<std::string>* missing) const;

        static const char* capName(InputCap cap);

    private:
        [[nodiscard]] bool freshAt(const PoseSample& sample) const;

        double timeout_sec_{0.2};
        std::array<PoseSample, 2> hands_;
        PoseSample head_;
        PoseSample footprint_;
        std::array<PoseSample, static_cast<size_t>(Joint::Count)> joints_;
        StickSample sticks_;
        TriggerSample triggers_;
        std::array<bool, static_cast<size_t>(InputCap::Count)> seen_{};
    };

} // namespace arms_ros2_control::command
