#pragma once
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace arms_controller_common
{
// 单条位姿采样点：绝对时间(秒) + 位置 + 四元数[qx,qy,qz,qw]
struct TrajSample
{
    double stamp_sec{0.0};
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 4> quat_xyzw{0.0, 0.0, 0.0, 1.0};
};

// 进程内单例：控制器实时循环只写内存缓存，flag true->false 时落盘。
class TrajectoryRecorder
{
public:
    static TrajectoryRecorder& instance();

    // flag 由参数回调设置；false->true 清空并开始，true->false 触发落盘。
    void setEnabled(bool enabled, const std::string& out_dir);

    bool enabled() const;

    // 实时安全：仅在持锁下 push_back 预留容量的 vector，不做磁盘 IO。
    void appendCal(const std::string& arm, const TrajSample& s);
    void appendPred(const std::string& arm, const TrajSample& s);
    void appendReal(const std::string& arm, const TrajSample& s);

private:
    TrajectoryRecorder() = default;
    void flushToDiskLocked();  // 在非实时上下文调用
    static void writeCsv(const std::string& path, const std::vector<TrajSample>& data);

    mutable std::mutex mutex_;
    bool enabled_{false};
    std::string out_dir_;
    std::vector<TrajSample> cal_left_, cal_right_, pred_left_, pred_right_, real_left_,
        real_right_;
};
}  // namespace arms_controller_common
