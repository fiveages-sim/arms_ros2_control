#pragma once

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ocs2::controller_common {

inline std::string formatCpuList(const std::vector<int>& cpus)
{
    if (cpus.empty())
    {
        return "inherit";
    }
    std::ostringstream oss;
    for (size_t i = 0; i < cpus.size(); ++i)
    {
        if (i > 0)
        {
            oss << ',';
        }
        oss << cpus[i];
    }
    return oss.str();
}

inline void applyThisThreadCpuAffinity(const std::vector<int>& cpus)
{
    if (cpus.empty())
    {
        return;
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    bool any = false;
    for (const int cpu : cpus)
    {
        if (cpu >= 0 && cpu < CPU_SETSIZE)
        {
            CPU_SET(cpu, &set);
            any = true;
        }
    }
    if (!any)
    {
        return;
    }
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
    {
        std::cerr << "WARNING: Failed to set thread CPU affinity\n";
    }
}

/** SCHED_FIFO priority (0 = leave) and optional CPU affinity for the calling thread. */
inline void applyThisThreadScheduling(const int priority, const std::vector<int>& cpus)
{
    applyThisThreadCpuAffinity(cpus);
    ocs2::setThisThreadPriority(priority);
}

inline void applyThisThreadSchedulingLogged(const int priority, const std::vector<int>& cpus,
                                            const rclcpp::Logger& logger, const char* thread_name)
{
    applyThisThreadScheduling(priority, cpus);
    if (priority > 0 || !cpus.empty())
    {
        RCLCPP_INFO(logger, "Applied scheduling to %s thread: priority=%d affinity=[%s]",
                    thread_name, priority, formatCpuList(cpus).c_str());
    }
}

inline std::vector<int> getThisThreadCpuAffinity()
{
    cpu_set_t set;
    CPU_ZERO(&set);
    if (pthread_getaffinity_np(pthread_self(), sizeof(set), &set) != 0)
    {
        return {};
    }
    std::vector<int> cpus;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu)
    {
        if (CPU_ISSET(cpu, &set))
        {
            cpus.push_back(cpu);
        }
    }
    return cpus;
}

/**
 * Split an isolated CPU island so the 500 Hz loop, MPC thread, and DDP/viz workers do not share a core.
 * Requires process affinity of at least 3 CPUs (e.g. taskset 8-11) and a non-empty yaml mpc list.
 */
struct IsolatedCpuPlan
{
    std::vector<int> rt;
    std::vector<int> mpc;
    std::vector<int> workers;
    bool isolated{false};
};

inline IsolatedCpuPlan planIsolatedCpus(const std::vector<int>& mpc_requested)
{
    IsolatedCpuPlan plan;
    plan.mpc = mpc_requested;
    if (mpc_requested.empty())
    {
        return plan;
    }
    const auto mask = getThisThreadCpuAffinity();
    if (mask.size() < 3)
    {
        return plan;
    }

    auto in_mask = [&](const int cpu) {
        return std::find(mask.begin(), mask.end(), cpu) != mask.end();
    };
    plan.mpc.clear();
    for (const int cpu : mpc_requested)
    {
        if (in_mask(cpu))
        {
            plan.mpc.push_back(cpu);
        }
    }
    if (plan.mpc.empty())
    {
        plan.mpc = mpc_requested;
        return plan;
    }

    std::vector<int> rest;
    rest.reserve(mask.size());
    for (const int cpu : mask)
    {
        if (std::find(plan.mpc.begin(), plan.mpc.end(), cpu) == plan.mpc.end())
        {
            rest.push_back(cpu);
        }
    }
    if (rest.empty())
    {
        plan.mpc = mpc_requested;
        return plan;
    }

    plan.rt = {rest.front()};
    for (size_t i = 1; i < rest.size(); ++i)
    {
        plan.workers.push_back(rest[i]);
    }
    if (plan.workers.empty())
    {
        for (const int cpu : mask)
        {
            if (cpu != plan.rt.front())
            {
                plan.workers.push_back(cpu);
            }
        }
    }
    plan.isolated = !plan.rt.empty();
    return plan;
}

} // namespace ocs2::controller_common
