//
// InputHub 实现：设备数据落库、按能力查询新鲜度、回答“缺什么”。
//
#include "arms_target_manager/core/InputHub.h"

#include <algorithm>

namespace arms_ros2_control::command
{
    bool InputHub::freshAt(const PoseSample& sample) const
    {
        if (sample.stamp.time_since_epoch().count() == 0)
        {
            return false;
        }
        const double age =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.stamp).count();
        return age <= std::max(0.0, timeout_sec_) && sample.usable();
    }

    void InputHub::setHand(bool is_left, const PoseSample& pose)
    {
        auto& slot = hands_[is_left ? 0 : 1];
        slot = pose;
        slot.stamp = std::chrono::steady_clock::now();
        slot.valid = pose.position.allFinite() && pose.orientation.coeffs().allFinite();
        seen_[static_cast<size_t>(is_left ? InputCap::HandLeft : InputCap::HandRight)] = true;
    }

    void InputHub::setHead(const PoseSample& pose)
    {
        head_ = pose;
        head_.stamp = std::chrono::steady_clock::now();
        seen_[static_cast<size_t>(InputCap::HeadPose)] = true;
    }

    void InputHub::setSticks(const StickSample& sticks)
    {
        sticks_ = sticks;
        sticks_.stamp = std::chrono::steady_clock::now();
        seen_[static_cast<size_t>(InputCap::Sticks)] = true;
        seen_[static_cast<size_t>(InputCap::Grips)] = true;
    }

    void InputHub::setTriggers(const TriggerSample& triggers)
    {
        triggers_ = triggers;
        triggers_.stamp = std::chrono::steady_clock::now();
        seen_[static_cast<size_t>(InputCap::Triggers)] = true;
    }

    void InputHub::setJoint(Joint joint, const PoseSample& pose)
    {
        auto& slot = joints_[static_cast<size_t>(joint)];
        slot = pose;
        slot.stamp = std::chrono::steady_clock::now();
        slot.valid = pose.position.allFinite() && pose.orientation.coeffs().allFinite();

        switch (joint)
        {
            case Joint::Pelvis:
            case Joint::Spine3:
            case Joint::Head:
                seen_[static_cast<size_t>(InputCap::SkeletonCore)] = true;
                break;
            case Joint::ShoulderLeft:
            case Joint::ShoulderRight:
            case Joint::HandLeft:
            case Joint::HandRight:
                seen_[static_cast<size_t>(InputCap::SkeletonArms)] = true;
                break;
            case Joint::Count:
                break;
        }
    }

    void InputHub::setFootprint(const PoseSample& pose)
    {
        footprint_ = pose;
        footprint_.stamp = std::chrono::steady_clock::now();
        footprint_.valid = pose.position.allFinite();
        seen_[static_cast<size_t>(InputCap::Footprint)] = true;
    }

    const PoseSample& InputHub::hand(bool is_left) const
    {
        return hands_[is_left ? 0 : 1];
    }

    const PoseSample& InputHub::head() const
    {
        return head_;
    }

    const StickSample& InputHub::sticks() const
    {
        return sticks_;
    }

    const TriggerSample& InputHub::triggers() const
    {
        return triggers_;
    }

    const PoseSample& InputHub::joint(Joint joint) const
    {
        return joints_[static_cast<size_t>(joint)];
    }

    bool InputHub::seen(InputCap cap) const
    {
        const auto index = static_cast<size_t>(cap);
        return index < seen_.size() && seen_[index];
    }

    bool InputHub::fresh(InputCap cap) const
    {
        switch (cap)
        {
            case InputCap::HandLeft:
                return freshAt(hands_[0]);
            case InputCap::HandRight:
                return freshAt(hands_[1]);
            case InputCap::HeadPose:
                return freshAt(head_);
            case InputCap::Sticks:
            case InputCap::Grips:
                return seen(InputCap::Sticks) &&
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - sticks_.stamp).count() <=
                    std::max(0.0, timeout_sec_);
            case InputCap::Triggers:
                return seen(InputCap::Triggers) &&
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - triggers_.stamp).count() <=
                    std::max(0.0, timeout_sec_);
            case InputCap::SkeletonCore:
                return freshAt(joint(Joint::Pelvis)) && freshAt(joint(Joint::Spine3));
            case InputCap::SkeletonArms:
                return freshAt(joint(Joint::HandLeft)) && freshAt(joint(Joint::HandRight));
            case InputCap::Footprint:
                return freshAt(footprint_);
            case InputCap::SkeletonLegs:
                return false;
            case InputCap::Count:
                break;
        }
        return false;
    }

    Caps InputHub::available() const
    {
        Caps caps = 0;
        for (uint32_t i = 0; i < static_cast<uint32_t>(InputCap::Count); ++i)
        {
            const auto cap = static_cast<InputCap>(1u << i);
            if (fresh(cap))
            {
                caps |= capOf(cap);
            }
        }
        return caps;
    }

    bool InputHub::requireFresh(Caps required, std::vector<std::string>* missing) const
    {
        bool ok = true;
        for (uint32_t i = 0; i < static_cast<uint32_t>(InputCap::Count); ++i)
        {
            const auto cap = static_cast<InputCap>(1u << i);
            if ((required & capOf(cap)) == 0 || fresh(cap))
            {
                continue;
            }
            ok = false;
            if (missing)
            {
                std::string name = capName(cap);
                if (seen(cap))
                {
                    name += "（已超时）";
                }
                missing->push_back(std::move(name));
            }
        }
        return ok;
    }

    const char* InputHub::capName(InputCap cap)
    {
        switch (cap)
        {
            case InputCap::HandLeft:
                return "左手柄位姿";
            case InputCap::HandRight:
                return "右手柄位姿";
            case InputCap::HeadPose:
                return "头显位姿";
            case InputCap::Sticks:
                return "摇杆";
            case InputCap::Triggers:
                return "扳机";
            case InputCap::Grips:
                return "握把";
            case InputCap::SkeletonCore:
                return "骨架躯干";
            case InputCap::SkeletonArms:
                return "骨架双臂";
            case InputCap::Footprint:
                return "人体基坐标系";
            case InputCap::SkeletonLegs:
                return "骨架下肢";
            case InputCap::Count:
                break;
        }
        return "未知输入";
    }

} // namespace arms_ros2_control::command
