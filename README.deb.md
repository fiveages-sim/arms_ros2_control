# arms_ros2_control Debian 发布说明

Workflow：[`.github/workflows/release-arms-ros2-control-deb.yml`](.github/workflows/release-arms-ros2-control-deb.yml)

## Pre-release 与版本号

### 触发与依赖

| 触发 | Arms 产物 | common / ocs2 依赖 |
|------|-----------|-------------------|
| PR 合并进 `main` | 更新唯一 rolling Release **`pre-release`** | 优先各仓库的 **`pre-release`**；不存在则回退 **latest 正式版** |
| 打 `v*` tag 或 `workflow_dispatch` | 正式 Release | **仅** latest 正式版（不使用依赖仓的 pre-release） |
| PR 关闭未合并 | 不构建 | — |

每次 PR 合并会**覆盖** `pre-release` 上的资产；历史 pre-release 会在发布时自动清理。

### 版本号规则

Arms 有 **两层标识**：GitHub Release 的 **tag** 与 deb 包内的 **Version** 字段。

**Pre-release（PR 合并进 main）**

| 项目 | 规则 | 示例 |
|------|------|------|
| GitHub Release tag | 固定 `pre-release` | `pre-release` |
| deb `Version` | latest 正式版基础上 **patch 累加** + `~main.` + 短 SHA | latest `v1.4.0`：第 1/2/3 次 → `1.4.1` / `1.4.2` / `1.4.3` |
| deb 文件名 | `{包名}_{Version}_{arch}.deb` | `ros-jazzy-arms-ros2-control_1.4.2~main.7b73a8b_amd64.deb` |

- 读取 **latest 正式版**；从现有 `pre-release` 的 deb 资产解析上一次基础版本
- **对齐**（`major.minor` 与 latest 一致）：在上一次 patch +1（`1.4.1` → `1.4.2`）
- **重置**（无上一次，或 `major.minor` 不一致，如上次 `1.5.0`/`1.3.0` 而 latest 为 `1.4.0`）：`latest.patch + 1`（→ `1.4.1`）
- 无正式 release 时以 `libraries/arms_controller_common/package.xml` 作 latest 基准
- `standard` 与 `full` 两个 bundle **共用同一 deb Version**，仅包名前缀不同
- `~main.` + 短 SHA 区分每次合并构建（不用 `+`，避免下载 pattern 问题）

**正式 Release（打 `v*` tag）**

| 项目 | 规则 | 示例 |
|------|------|------|
| GitHub Release tag | git tag | `v2.0.0` |
| deb `Version` | tag 去 `v`，`-` 转 `~` | `2.0.0` |

### 安装示例

```bash
# pre-release（rolling）
gh release download pre-release --repo fiveages-sim/arms_ros2_control --pattern 'ros-jazzy-arms-ros2-control_*_amd64.deb'
sudo dpkg -i ros-jazzy-arms-ros2-control_1.4.2~main.7b73a8b_amd64.deb
# 另需与构建时相同来源的 ocs2 / common deb（workflow 日志中的 ocs2_tag / common_tag）

# 正式版
gh release download v2.0.0 --repo fiveages-sim/arms_ros2_control --pattern '*_amd64.deb'
sudo dpkg -i ros-jazzy-arms-ros2-control_2.0.0_amd64.deb
```

## 发布的包（仅此两种）

| 包名 | 适用场景 |
|------|----------|
| `ros-jazzy-arms-ros2-control` | WBC 完整栈：`lina_planning`、`ocs2_humanoid`、`ocs2_wbc_controller`、command、controllers、公共库、`topic_based_ros2_control` 等（amd64 + arm64）；`Provides: arms-ros2-control-jazzy` |
| `ros-jazzy-arms-ros2-control-full` | 上表 + `gripper_hardware_common` + `marvin_ros2_control` + `modbus_ros2_control` + `can_ros2_control` + `juxie_ros2_control` + `rokae_ros2_control`（amd64 + arm64） |

运行时依赖：`ros-jazzy-ocs2`、`ros-jazzy-robot-descriptions-common`（旧 OCS2 包名见 `ros-jazzy-ocs2` 的 `Provides` 字段）。

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
  - 子模块仓库：`lina_planning`、`ocs2-humanoid`、`ocs2-wbc-controller`；full 另需 `marvin-ros2-control`、`modbus-ros2-control`、`can-ros2-control`、`juxie-ros2-control`、`rokae-ros2-control`
- Workflow 只 **按需** `git submodule update` 上述路径；不克隆 arx / dobot / eyou / unitree 等未打包子模块
- **`topic_based_ros2_control`** 为 monorepo 内 vendored 源码（非子模块），standard / full 两种 deb 均包含
- CI 容器无 `ssh`：子模块 URL 改为 **仅** `https://x-access-token:<PAT>@github.com/...`（不要同时设 `http.extraHeader: Authorization`，否则会 `Duplicate header` / 400）
- **`full` bundle**：构建前删除误提交的 `libKine.so` / `libMarvinSDK.so`，按 runner 架构从源码重编 Marvin SDK
- 产物仅含 colcon **install**（二进制 + 公共头文件 + 配置）；CI 拒绝误打包的 `*/src/*` 源码树（**允许** rosidl 生成的 `*_s.c` / `__type_support.cpp` 等）

## 子模块仓库

各 hardware / `ocs2-wbc-controller` 子仓库中的 `.github/workflows/release-*.yml` 应**整文件删除**（monorepo 内已不再包含这些 workflow）。在子仓库 default 分支删除后，打子模块 tag 不会再误发旧 deb。
