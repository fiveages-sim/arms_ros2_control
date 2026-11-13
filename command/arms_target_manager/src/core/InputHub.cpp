//
// InputHub 实现：设备数据落库、按能力查询新鲜度、回答“缺什么”。
//
// 关键点：
//   * set 系列函数会重新盖时间戳，并顺便把“数值是否有限”写成 valid；
//   * 骨架能力按分组判断（躯干类需 pelvis+spine3，双臂类需双手）；
//   * requireFresh 的缺项描述会区分“从未收到”与“收到过但已超时”。
//
// 注意：
//   本文件不打印日志；提示文字由调用方用 requireFresh 返回的 missing 拼装。
//
#include "arms_target_manager/core/InputHub.h"

#include <algorithm>

namespace arms_ros2_control::command
{
    /**
     * @brief 判断一个位姿采样是否既有效又未超时
     * @param sample 待检查的采样
     * @return true 表示可以直接用于计算
     */
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

    /**
     * @brief 上报一只手柄位姿
     *
     * 时间戳由本函数盖章（不用消息里的时间），因此新鲜度反映的是“多久没收到”，
     * 而不是“设备自己认为过了多久”。同时把数值是否有限写成 valid。
     *
     * @param is_left true=左手，false=右手
     * @param pose 位姿采样
     */
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

    /**
     * @brief 上报一个骨架关节
     *
     * 同时按关节所属分组点亮能力位（躯干类 → SkeletonCore，双臂类 → SkeletonArms），
     * 这样“骨架齐备了吗”只需要查能力位，不用逐个关节判断。
     *
     * @param joint 关节枚举
     * @param pose 位姿采样
     */
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

    /**
     * @brief 查询某类数据当前是否新鲜
     *
     * 与 seen() 的区别：seen() 只回答“曾经收到过”，这里还要求没超时。
     * 复合能力（如 SkeletonCore）需要组内关节同时新鲜才算齐备。
     */
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

    /**
     * @brief 当前仍然新鲜的能力位集合
     */
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

    /**
     * @brief 检查一组能力是否齐备
     *
     * 缺什么会写成中文名字放进 missing；曾经收到过但已超时的会额外标注“（已超时）”，
     * 便于现场区分“头显没开”和“头显卡了”。
     *
     * @param required 需要的能力位组合
     * @param missing 可选输出，缺项描述列表
     * @return true 表示全部齐备
     */
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

    /**
     * @brief 能力位的中文名，用于拼装用户可读的提示
     */
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
