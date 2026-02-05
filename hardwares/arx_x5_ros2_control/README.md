# ARX X5 ROS2 Control 硬件接口包

本包提供了 ARX X5 机械臂的 ROS2 Control 硬件接口实现。

## 依赖项

### ROS2 依赖
- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`

### 第三方依赖
- `Eigen3`
- `orocos_kdl`
- `kdl_parser`
- `spdlog`

## 编译步骤

### 1. 编译 external 文件夹下的 SDK

在编译主包之前，需要先编译 `external/arx5-sdk` 目录下的 SDK。

#### 1.1 设置 conda 环境

SDK 需要 conda 环境来管理依赖。推荐使用 mamba（更快），也可以使用 conda：

```bash
# 进入 SDK 目录
cd external/arx5-sdk

# 使用 mamba 创建环境（推荐，约1分钟）
mamba env create -f conda_environments/py312_environment.yaml

# 或使用 conda（较慢，约10分钟）
# conda env create -f conda_environments/py312_environment.yaml

# 激活环境
conda activate arx-py312
```

**注意：** 可用的 Python 版本包括 3.8, 3.9, 3.10, 3.11, 3.12。请根据您的系统选择合适的版本。

#### 1.2 编译 SDK

在 conda 环境中编译 SDK：

```bash
# 确保在 SDK 目录下
cd external/arx5-sdk

# 创建构建目录
mkdir -p build
cd build

# 配置 CMake
cmake ..

# 编译
make -j$(nproc)

# 可选：安装到系统
# make install
```

编译完成后，会在 `build` 目录下生成 `libArxJointController.so` 和 `libArxCartesianController.so` 等库文件。

**重要提示：** 
- 编译 SDK 时必须在 conda 环境中（`conda activate arx-py312`）
- 主包的 CMakeLists.txt 会链接 `${ARX5_SDK_DIR}/build/libArxJointController.so`，因此 SDK 必须先编译

### 2. 编译 ROS2 包

编译完 SDK 后，回到工作空间根目录编译 ROS2 包：

```bash
# 回到工作空间根目录
cd ~/ros2_ws

# 编译包
colcon build --packages-select arx_x5_ros2_control

# 或编译整个工作空间
# colcon build
```


