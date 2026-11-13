//
// 信息输入面：所有设备的数据先汇到一处，再被映射读取。
//
// 好处：
//   1. 映射不再依赖“哪个回调先到”，左右手谁先发不影响结果；
//   2. 新鲜度、缺失、可用能力可以统一回答，准入判断只有一个实现；
//   3. 新增关节只是多一个枚举值，不影响已有调用点。
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

    struct PoseSample
    {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
        float confidence{1.0f};
        bool valid{false};
        std::chrono::steady_clock::time_point stamp{};

        [[nodiscard]] bool usable() const
        {
            return valid && position.allFinite() && orientation.coeffs().allFinite();
        }
    };

    struct StickSample
    {
        Eigen::Vector2d left{Eigen::Vector2d::Zero()};
        Eigen::Vector2d right{Eigen::Vector2d::Zero()};
        bool left_grip{false};
        bool right_grip{false};
        std::chrono::steady_clock::time_point stamp{};
    };

    struct TriggerSample
    {
        double left{0.0};
        double right{0.0};
        std::chrono::steady_clock::time_point stamp{};
    };

    class InputHub
    {
    public:
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

        [[nodiscard]] bool seen(InputCap cap) const;
        [[nodiscard]] bool fresh(InputCap cap) const;
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
