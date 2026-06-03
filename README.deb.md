# arms_ros2_control Debian 发布说明

仅在 **本仓库（fiveages-sim/arms_ros2_control）** 打 `v*` tag 或手动触发：

[`.github/workflows/release-arms-ros2-control-deb.yml`](.github/workflows/release-arms-ros2-control-deb.yml)

## 发布的包（仅此两种）

| 包名 | 适用场景 |
|------|----------|
| `ros-jazzy-arms-ros2-control` | WBC 完整栈：`lina_planning`、`ocs2_humanoid`、`ocs2_wbc_controller`、command、controllers、公共库等（amd64 + arm64）；`Provides: arms-ros2-control-jazzy` |
| `ros-jazzy-arms-ros2-control-full` | 上表 + `gripper_hardware_common` + `marvin_ros2_control` + `modbus_ros2_control` + `juxie_ros2_control` + `rokae_ros2_control`（amd64 + arm64） |

运行时依赖：`ros-jazzy-ocs2`、`ros-jazzy-robot-descriptions-common`（旧 OCS2 包名见 `ros-jazzy-ocs2` 的 `Provides` 字段）。

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
  - **Fine-grained PAT**（推荐核对项）：
    - **Resource owner** 选 **`fiveages-sim`（组织）**，不要只挂在个人账号且只勾 `arms_ros2_control` 一个仓
    - **Repository access**：All repositories（或至少下列每一个）
    - **Permissions → Repository → Contents: Read-only**（克隆子模块只需读；没有 Contents 读权限时 Git 会报 `Write access not granted`）
    - 组织若启用 **SAML SSO**：到 [Fine-grained tokens](https://github.com/settings/tokens?type=beta) 对该 PAT 点 **Configure SSO → Authorize**
  - 子模块仓库：`lina_planning`、`ocs2-humanoid`、`ocs2-wbc-controller`；full 另需 `marvin-ros2-control`、`modbus-ros2-control`、`juxie-ros2-control`、`rokae-ros2-control`
- Workflow 只 **按需** `git submodule update` 上述路径；不克隆 arx / dobot / eyou / unitree 等未打包子模块
- CI 容器无 `ssh`：子模块 URL 改为 **仅** `https://x-access-token:<PAT>@github.com/...`（不要同时设 `http.extraHeader: Authorization`，否则会 `Duplicate header` / 400）
- **`full` bundle**：构建前删除误提交的 `libKine.so` / `libMarvinSDK.so`，按 runner 架构从源码重编 Marvin SDK
- 产物仅含 colcon **install**（二进制 + 公共头文件 + 配置）；CI 拒绝误打包的 `*/src/*` 源码树（**允许** rosidl 生成的 `*_s.c` / `__type_support.cpp` 等）

## 子模块仓库

各 hardware / `ocs2-wbc-controller` 子仓库中的 `.github/workflows/release-*.yml` 应**整文件删除**（monorepo 内已不再包含这些 workflow）。在子仓库 default 分支删除后，打子模块 tag 不会再误发旧 deb。
