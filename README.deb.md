# arms_ros2_control Debian 发布说明

仅在 **本仓库（fiveages-sim/arms_ros2_control）** 打 `v*` tag 或手动触发：

[`.github/workflows/release-arms-ros2-control-deb.yml`](.github/workflows/release-arms-ros2-control-deb.yml)

## 发布的包（仅此两种）

| 包名 | 适用场景 |
|------|----------|
| `ros-jazzy-arms-ros2-control` | WBC 完整栈：`lina_planning`、`ocs2_humanoid`、`ocs2_wbc_controller`、command、controllers、公共库等（amd64 + arm64） |
| `ros-jazzy-arms-ros2-control-full` | 上表 + `gripper_hardware_common` + `marvin_ros2_control` + `modbus_ros2_control` + `juxie_ros2_control`（amd64 + arm64） |

安装示例：

```bash
sudo dpkg -i ros-jazzy-arms-ros2-control-full_<version>_<arch>.deb
# 另需同 tag 的 ocs2 / robot_descriptions 依赖 deb（workflow 文档中的外部 Release）
```

## 已退役的独立 deb（请勿再打 tag）

以下 workflow 已停用；功能由 **`ros-jazzy-arms-ros2-control-full`** 覆盖：

- `arms-gripper-hardware-common-jazzy`（gripper 库）
- 各 hardware 仓单独发布的 marvin / modbus / juxie / dobot / eyou deb
- `arms-ros2-control-jazzy-wbc`（旧 WBC 包名，见 `ocs2-wbc-controller` 子模块说明）

## CI 要求

- Secret：**`PRIVATE_SUBMODULES_TOKEN`**（读取 private 子模块）
- **`full` bundle**：构建前删除子模块里可能误提交的 `libKine.so` / `libMarvinSDK.so`，由 CI 按目标架构（amd64 / arm64）从源码重编
- 产物仅含 colcon **install**（二进制 + 公共头文件 + 配置），不含 `.cpp` 源码

## 子模块仓库

各 hardware / `ocs2-wbc-controller` 子仓库中的 `.github/workflows/release-*.yml` 应**整文件删除**（monorepo 内已不再包含这些 workflow）。在子仓库 default 分支删除后，打子模块 tag 不会再误发旧 deb。
