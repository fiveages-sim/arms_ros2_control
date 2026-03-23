#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace arms_controller_common::utils
{
    inline rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    getOrCreateCurrentTargetJointPublisher(
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
    {
        if (!node)
        {
            return nullptr;
        }

        using PublisherT = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>;
        static std::mutex map_mutex;
        static std::unordered_map<const void*, std::weak_ptr<PublisherT>> publisher_map;

        const void* key = static_cast<const void*>(node.get());

        std::lock_guard<std::mutex> lock(map_mutex);
        if (auto it = publisher_map.find(key); it != publisher_map.end())
        {
            if (auto existing = it->second.lock())
            {
                return existing;
            }
        }

        const std::string topic = node->get_name() + std::string("/current_target_joint");
        auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 1);
        publisher_map[key] = publisher;
        return publisher;
    }
} // namespace arms_controller_common::utils
