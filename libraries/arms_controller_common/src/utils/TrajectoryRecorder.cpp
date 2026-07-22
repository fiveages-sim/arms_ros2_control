#include "arms_controller_common/utils/TrajectoryRecorder.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace arms_controller_common
{
TrajectoryRecorder& TrajectoryRecorder::instance()
{
    static TrajectoryRecorder inst;
    return inst;
}

bool TrajectoryRecorder::enabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

void TrajectoryRecorder::setEnabled(bool enabled, const std::string& out_dir)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled && !enabled_)
    {
        // false -> true：开始新一段录制
        out_dir_ = out_dir;
        cal_left_.clear();
        cal_right_.clear();
        pred_left_.clear();
        pred_right_.clear();
        real_left_.clear();
        real_right_.clear();
        // 预留约 20 分钟 @ 25Hz = 30000 点（单缓冲 ~1.9MB，6 缓冲共 ~11MB）。
        // 超出后 vector 自动增长（不硬截断），仅可能触发一次 realloc。
        constexpr size_t kReserveSamples = 30000;
        cal_left_.reserve(kReserveSamples);
        cal_right_.reserve(kReserveSamples);
        pred_left_.reserve(kReserveSamples);
        pred_right_.reserve(kReserveSamples);
        real_left_.reserve(kReserveSamples);
        real_right_.reserve(kReserveSamples);
        enabled_ = true;
    }
    else if (!enabled && enabled_)
    {
        // true -> false：先关标志再落盘（落盘在持锁的非实时上下文）
        enabled_ = false;
        flushToDiskLocked();
    }
}

void TrajectoryRecorder::appendCal(const std::string& arm, const TrajSample& s)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_)
    {
        return;
    }
    if (arm == "right")
    {
        cal_right_.push_back(s);
    }
    else
    {
        cal_left_.push_back(s);
    }
}

void TrajectoryRecorder::appendPred(const std::string& arm, const TrajSample& s)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_)
    {
        return;
    }
    if (arm == "right")
    {
        pred_right_.push_back(s);
    }
    else
    {
        pred_left_.push_back(s);
    }
}

void TrajectoryRecorder::appendReal(const std::string& arm, const TrajSample& s)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_)
    {
        return;
    }
    if (arm == "right")
    {
        real_right_.push_back(s);
    }
    else
    {
        real_left_.push_back(s);
    }
}

void TrajectoryRecorder::writeCsv(const std::string& path, const std::vector<TrajSample>& data)
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        std::cerr << "[TrajectoryRecorder] failed to open CSV for write: " << path << std::endl;
        return;
    }
    f << "timestamp_sec,timestamp_nanosec,position_x,position_y,position_z,"
      << "orientation_x,orientation_y,orientation_z,orientation_w\n";
    f << std::fixed << std::setprecision(9);
    for (const auto& s : data)
    {
        const int64_t sec = static_cast<int64_t>(s.stamp_sec);
        const int64_t nsec =
            static_cast<int64_t>((s.stamp_sec - static_cast<double>(sec)) * 1e9);
        f << sec << "," << nsec << "," << s.position[0] << "," << s.position[1] << ","
          << s.position[2] << "," << s.quat_xyzw[0] << "," << s.quat_xyzw[1] << ","
          << s.quat_xyzw[2] << "," << s.quat_xyzw[3] << "\n";
    }
}

void TrajectoryRecorder::flushToDiskLocked()
{
    std::error_code ec;
    std::filesystem::create_directories(out_dir_, ec);
    if (ec)
    {
        std::cerr << "[TrajectoryRecorder] failed to create directory: " << out_dir_
                  << " (" << ec.message() << ")" << std::endl;
        return;
    }
    if (!cal_left_.empty())
    {
        writeCsv(out_dir_ + "/cal_left.csv", cal_left_);
    }
    if (!cal_right_.empty())
    {
        writeCsv(out_dir_ + "/cal_right.csv", cal_right_);
    }
    if (!pred_left_.empty())
    {
        writeCsv(out_dir_ + "/pred_left.csv", pred_left_);
    }
    if (!pred_right_.empty())
    {
        writeCsv(out_dir_ + "/pred_right.csv", pred_right_);
    }
    if (!real_left_.empty())
    {
        writeCsv(out_dir_ + "/real_left.csv", real_left_);
    }
    if (!real_right_.empty())
    {
        writeCsv(out_dir_ + "/real_right.csv", real_right_);
    }
}
}  // namespace arms_controller_common
