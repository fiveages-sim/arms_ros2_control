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

- Secret：**`PRIVATE_SUBMODULES_TOKEN`**（PAT，不要用默认 `GITHUB_TOKEN` 代替）
  - **Classic PAT**：勾选 `repo`（读 private 仓库）
  - **Fine-grained PAT**：对以下仓库授予 **Contents: Read**（仅构建所需，不要 `submodules: recursive` 拉全表）  
    `lina_planning`、`ocs2-humanoid`、`ocs2-wbc-controller`；full 另需 `marvin-ros2-control`、`modbus-ros2-control`、`juxie-ros2-control`
- Workflow 只 **按需** `git submodule update` 上述路径；不克隆 arx / dobot / eyou / rokae / unitree 等未打包子模块
- CI 容器无 `ssh`：子模块 URL 会在 clone 前从 `git@github.com:` **改写为 HTTPS + PAT**（勿依赖 `git submodule sync`，以免恢复 SSH 地址）
- **`full` bundle**：构建前删除误提交的 `libKine.so` / `libMarvinSDK.so`，按 runner 架构从源码重编 Marvin SDK
- 产物仅含 colcon **install**（二进制 + 公共头文件 + 配置），不含 `.cpp` 源码

## 子模块仓库

各 hardware / `ocs2-wbc-controller` 子仓库中的 `.github/workflows/release-*.yml` 应**整文件删除**（monorepo 内已不再包含这些 workflow）。在子仓库 default 分支删除后，打子模块 tag 不会再误发旧 deb。
