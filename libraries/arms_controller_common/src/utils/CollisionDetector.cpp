// libraries/arms_controller_common/src/utils/CollisionDetector.cpp
#include "arms_controller_common/utils/CollisionDetector.h"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include <chrono>
#include <sstream>
#include <algorithm>

namespace arms_controller_common
{
    CollisionDetector::CollisionDetector(const pinocchio::Model& pinocchio_model,
                                         const pinocchio::GeometryModel& geometry_model,
                                         const std::vector<std::pair<std::string, std::string>>& collision_pairs)
        : pinocchio_model_(pinocchio_model)
          , geometry_model_(geometry_model)
          , collision_pairs_(collision_pairs)
    {
        // 创建运动学数据
        pinocchio_data_ = pinocchio::Data(pinocchio_model_);

        // 创建几何数据
        geometry_data_ = pinocchio::GeometryData(geometry_model_);

        // 设置碰撞对
        buildCollisionPairsCache();

        std::cout << "CollisionDetector initialized: "
            << collision_pair_indices_.size() << " valid collision pairs out of "
            << collision_pairs_.size() << " total, "
            << "model nq=" << pinocchio_model_.nq << ", "
            << "geometry objects: " << geometry_model_.geometryObjects.size()
            << std::endl;
    }


    CollisionDetector::CollisionDetector(
        const pinocchio::Model& model,
        const std::string& urdf_file,
        const std::vector<std::pair<std::string, std::string>>& collision_pairs)
        : pinocchio_model_(model),
          collision_pairs_(collision_pairs)
    {
        try
        {
            pinocchio_data_ = pinocchio::Data(pinocchio_model_);

            // 从文件读取 URDF 内容
            std::ifstream file(urdf_file);
            if (!file.is_open())
            {
                throw std::runtime_error("Failed to open URDF file: " + urdf_file);
            }

            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string urdf_content = buffer.str();


            std::stringstream urdf_stream(urdf_content);

            pinocchio::urdf::buildGeom(model, urdf_stream, pinocchio::COLLISION, geometry_model_);

            // 创建几何数据
            geometry_data_ = pinocchio::GeometryData(geometry_model_);

            // 设置碰撞对
            buildCollisionPairsCache();

            std::cout << "CollisionDetector initialized from URDF: "
                << collision_pair_indices_.size() << " valid collision pairs out of "
                << collision_pairs_.size() << " total, "
                << "geometry objects: " << geometry_model_.geometryObjects.size()
                << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to initialize CollisionDetector from URDF: " << e.what() << std::endl;
        }
    }

    void CollisionDetector::buildCollisionPairsCache()
    {
        geometry_model_.collisionPairs.clear();
        collision_pair_indices_.clear();

        for (const auto& pair : collision_pairs_)
        {
            // 使用 Frame 名称查找几何对象索引
            int idx1 = getGeometryIndexByFrameName(pair.first);
            int idx2 = getGeometryIndexByFrameName(pair.second);

            if (idx1 >= 0 && idx2 >= 0 && idx1 != idx2)
            {
                collision_pair_indices_.emplace_back(idx1, idx2);
                geometry_model_.addCollisionPair(pinocchio::CollisionPair(idx1, idx2));
            }
            else if (idx1 >= 0 && idx2 >= 0 && idx1 == idx2)
            {
                std::cerr << "Warning: Collision pair [" << pair.first
                    << ", " << pair.second << "] refers to same geometry object, ignored" << std::endl;
            }
            else
            {
                std::cerr << "Warning: Collision pair [" << pair.first
                    << ", " << pair.second << "] not found" << std::endl;
            }
        }

        // 重新创建几何数据以确保同步
        geometry_data_ = pinocchio::GeometryData(geometry_model_);
        geometry_updated_ = false;
    }

    int CollisionDetector::getGeometryIndexByFrameName(const std::string& frame_name) const
    {
        for (size_t i = 0; i < geometry_model_.geometryObjects.size(); ++i)
        {
            const auto& obj = geometry_model_.geometryObjects[i];
            // parentFrame 是 size_t 类型，直接比较即可
            if (obj.parentFrame < pinocchio_model_.frames.size())
            {
                const std::string& frame_name_in_model = pinocchio_model_.frames[obj.parentFrame].name;
                if (frame_name_in_model == frame_name)
                {
                    return static_cast<int>(i);
                }
            }
        }
        return -1;
    }

    void CollisionDetector::updateGeometry(const Eigen::VectorXd& joint_positions)
    {
        // 检查是否需要更新（避免重复计算）
        if (cached_joint_positions_.size() == joint_positions.size() &&
            cached_joint_positions_.isApprox(joint_positions, 1e-8) &&
            geometry_updated_)
        {
            return;
        }

        pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, joint_positions);

        pinocchio::updateGeometryPlacements(
            pinocchio_model_, pinocchio_data_, geometry_model_, geometry_data_, joint_positions);


        cached_joint_positions_ = joint_positions;
        geometry_updated_ = true;
    }

    bool CollisionDetector::checkCollision(const Eigen::VectorXd& joint_positions, double min_distance)
    {
        if (geometry_model_.geometryObjects.empty())
        {
            return false;
        }

        // auto start_time = std::chrono::high_resolution_clock::now();

        updateGeometry(joint_positions);

        pinocchio::computeDistances(geometry_model_, geometry_data_);

        bool collision_detected = false;
        for (const auto& result : geometry_data_.distanceResults)
        {
            if (result.min_distance < min_distance)
            {
                collision_detected = true;
                break; // 只要有一个碰撞就退出
            }
        }

        // auto end_time = std::chrono::high_resolution_clock::now();
        // last_computation_time_ms_ = std::chrono::duration<double, std::milli>(
        //     end_time - start_time).count();

        return collision_detected;
    }

    double CollisionDetector::getMinimumDistance(const Eigen::VectorXd& joint_positions)
    {
        if (geometry_model_.geometryObjects.empty())
        {
            return std::numeric_limits<double>::max();
        }

        updateGeometry(joint_positions);
        pinocchio::computeDistances(geometry_model_, geometry_data_);

        double min_distance = std::numeric_limits<double>::max();
        for (const auto& result : geometry_data_.distanceResults)
        {
            if (result.min_distance < min_distance)
            {
                min_distance = result.min_distance;
            }
        }

        return min_distance;
    }

    std::vector<double> CollisionDetector::getAllDistances(const Eigen::VectorXd& joint_positions)
    {
        std::vector<double> distances;

        if (geometry_model_.geometryObjects.empty())
        {
            return distances;
        }

        updateGeometry(joint_positions);
        pinocchio::computeDistances(geometry_model_, geometry_data_);

        distances.reserve(geometry_data_.distanceResults.size());
        for (const auto& result : geometry_data_.distanceResults)
        {
            distances.push_back(result.min_distance);
        }

        return distances;
    }


    double CollisionDetector::getPairDistance(const Eigen::VectorXd& joint_positions, size_t pair_index)
    {
        if (geometry_model_.geometryObjects.empty() || pair_index >= geometry_data_.distanceResults.size())
        {
            return -1.0;
        }

        updateGeometry(joint_positions);
        pinocchio::computeDistances(geometry_model_, geometry_data_);

        return geometry_data_.distanceResults[pair_index].min_distance;
    }


    double CollisionDetector::getPairDistanceWithPoints(
        const Eigen::VectorXd& joint_positions,
        size_t pair_index,
        Eigen::Vector3d& nearest_point1,
        Eigen::Vector3d& nearest_point2)
    {
        if (geometry_model_.geometryObjects.empty() || pair_index >= geometry_data_.distanceResults.size())
        {
            return -1.0;
        }

        updateGeometry(joint_positions);
        pinocchio::computeDistances(geometry_model_, geometry_data_);

        const auto& result = geometry_data_.distanceResults[pair_index];
        nearest_point1 = result.nearest_points[0];
        nearest_point2 = result.nearest_points[1];

        return result.min_distance;
    }


    void CollisionDetector::setCollisionPairs(
        const std::vector<std::pair<std::string, std::string>>& collision_pairs)
    {
        collision_pairs_ = collision_pairs;
        buildCollisionPairsCache();
        geometry_updated_ = false; // 强制下次更新
    }


    size_t CollisionDetector::getNumCollisionPairs() const
    {
        return collision_pair_indices_.size();
    }


    std::string CollisionDetector::getCollisionPairName(size_t index) const
    {
        if (index >= collision_pairs_.size())
        {
            return "";
        }
        return collision_pairs_[index].first + " <-> " + collision_pairs_[index].second;
    }


    void CollisionDetector::printCollisionPairs() const
    {
        std::cout << "\n=== CollisionDetector Information ===" << std::endl;
        std::cout << "Model: nq=" << pinocchio_model_.nq
            << ", nv=" << pinocchio_model_.nv << std::endl;
        std::cout << "Geometry objects: "
            << geometry_model_.geometryObjects.size() << std::endl;
        std::cout << "Collision pairs: "
            << collision_pair_indices_.size() << " valid out of "
            << collision_pairs_.size() << " configured" << std::endl;

        if (!collision_pairs_.empty())
        {
            std::cout << "\nConfigured collision pairs:" << std::endl;
            for (size_t i = 0; i < collision_pairs_.size(); ++i)
            {
                bool valid = (i < collision_pair_indices_.size());
                std::cout << "  [" << i << "] "
                    << collision_pairs_[i].first
                    << " <-> " << collision_pairs_[i].second
                    << (valid ? " [VALID]" : " [INVALID: geometry not found]")
                    << std::endl;
            }
        }
        std::cout << "=====================================\n" << std::endl;
        std::cout << "Available geometry objects (" << geometry_model_.geometryObjects.size() << "):" << std::endl;
        for (size_t i = 0; i < geometry_model_.geometryObjects.size(); ++i)
        {
            std::cout << "  [" << i << "] " << geometry_model_.geometryObjects[i].name << std::endl;
        }
    }
} // namespace arms_controller_common
