// libraries/arms_controller_common/include/arms_controller_common/utils/CollisionDetector.h
#pragma once

#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <limits>

namespace arms_controller_common
{
    class CollisionDetector
    {
    public:
        /**
         * @brief 构造函数 - 使用已有的模型和几何模型（推荐）
         * @param pinocchio_model Pinocchio 运动学模型
         * @param geometry_model Pinocchio 几何模型（包含碰撞体信息）
         * @param collision_pairs 需要检查的碰撞对（连杆名称对）
         */
        CollisionDetector(const pinocchio::Model& pinocchio_model,
                          const pinocchio::GeometryModel& geometry_model,
                          const std::vector<std::pair<std::string, std::string>>& collision_pairs);

        /**
         * @brief 构造函数 - 从 URDF 字符串构建（备选）
         * @param model Pinocchio 运动学模型
         * @param urdf_file URDF 路径
         * @param collision_pairs 需要检查的碰撞对
         */
        CollisionDetector(const pinocchio::Model& model,
                          const std::string& urdf_file,
                          const std::vector<std::pair<std::string, std::string>>& collision_pairs);

        ~CollisionDetector() = default;

        // // 禁止拷贝，允许移动
        // CollisionDetector(const CollisionDetector&) = delete;
        // CollisionDetector& operator=(const CollisionDetector&) = delete;
        // CollisionDetector(CollisionDetector&&) = default;
        // CollisionDetector& operator=(CollisionDetector&&) = default;

        /**
         * @brief 检查当前关节配置下是否发生碰撞
         * @param joint_positions 关节角度（弧度）
         * @param min_distance 最小安全距离（米），距离 < min_distance 认为发生碰撞
         * @return true 如果发生碰撞
         */
        bool checkCollision(const Eigen::VectorXd& joint_positions, double min_distance);

        /**
         * @brief 获取当前关节配置下的最小距离
         * @param joint_positions 关节角度（弧度）
         * @return 所有碰撞对中的最小距离（米）
         */
        double getMinimumDistance(const Eigen::VectorXd& joint_positions);

        /**
         * @brief 获取所有碰撞对的距离
         * @param joint_positions 关节角度（弧度）
         * @return 距离向量，顺序与构造时的 collision_pairs 一致
         */
        std::vector<double> getAllDistances(const Eigen::VectorXd& joint_positions);

        /**
         * @brief 获取指定碰撞对的距离
         * @param joint_positions 关节角度（弧度）
         * @param pair_index 碰撞对索引
         * @return 距离（米），无效索引返回 -1.0
         */
        double getPairDistance(const Eigen::VectorXd& joint_positions, size_t pair_index);

        /**
         * @brief 获取碰撞对的距离和最近点信息
         * @param joint_positions 关节角度（弧度）
         * @param pair_index 碰撞对索引
         * @param nearest_point1 输出：第一个碰撞体上的最近点（世界坐标）
         * @param nearest_point2 输出：第二个碰撞体上的最近点（世界坐标）
         * @return 距离（米），无效索引返回 -1.0
         */
        double getPairDistanceWithPoints(const Eigen::VectorXd& joint_positions,
                                         size_t pair_index,
                                         Eigen::Vector3d& nearest_point1,
                                         Eigen::Vector3d& nearest_point2);

        /**
         * @brief 更新碰撞对列表
         * @param collision_pairs 新的碰撞对列表
         */
        void setCollisionPairs(const std::vector<std::pair<std::string, std::string>>& collision_pairs);

        /**
         * @brief 获取当前配置的碰撞对数量
         */
        size_t getNumCollisionPairs() const;

        /**
         * @brief 获取碰撞对名称（用于调试）
         * @param index 碰撞对索引
         * @return 名称字符串
         */
        std::string getCollisionPairName(size_t index) const;

        /**
         * @brief 打印所有碰撞对信息（调试用）
         */
        void printCollisionPairs() const;

        // /**
        //  * @brief 获取上次计算耗时（毫秒）
        //  */
        // double getLastComputationTimeMs() const { return last_computation_time_ms_; }


        pinocchio::Model getModel() const { return pinocchio_model_; }


        pinocchio::GeometryModel getGeometryModel() const { return geometry_model_; }

    private:
        /**
         * @brief 更新几何模型位置
         * @param joint_positions 关节角度
         */
        void updateGeometry(const Eigen::VectorXd& joint_positions);

        /**
        * @brief 通过 Frame 名称获取几何对象索引
        * @param frame_name Frame 名称（如 "arm_base"、"left_link5" 等）
        * @return 几何对象索引，未找到返回 -1
        */
        int getGeometryIndexByFrameName(const std::string& frame_name) const;

        /**
         * @brief 内部初始化：构建碰撞对索引缓存
         */
        void buildCollisionPairsCache();

        // pinocchio 模型和数据
        pinocchio::Model pinocchio_model_;
        pinocchio::Data pinocchio_data_;
        pinocchio::GeometryModel geometry_model_;
        pinocchio::GeometryData geometry_data_;

        // 碰撞对配置
        std::vector<std::pair<std::string, std::string>> collision_pairs_;
        std::vector<std::pair<int, int>> collision_pair_indices_; // 缓存的碰撞对索引

        // 缓存的关节角度（用于避免重复计算）
        Eigen::VectorXd cached_joint_positions_;
        bool geometry_updated_{false};

        // 性能统计
        // mutable double last_computation_time_ms_{0.0};
    };
} // namespace arms_controller_common
