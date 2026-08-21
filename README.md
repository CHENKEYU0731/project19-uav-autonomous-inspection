# ROS2 无人机自主巡检系统

基于 PX4、ROS 2 和 Gazebo 的无人机自主巡检项目。项目范围、架构和里程碑以 [`项目方案-ROS2无人机自主巡检系统.md`](项目方案-ROS2无人机自主巡检系统.md) 为准。

## 当前状态

当前里程碑：**M0 - 环境搭建与跑通官方例程（已验收）**。

2026-08-22 已在本机直接验证：PX4 `gz_x500` SITL 命令行起飞/降落、ROS 2 飞控里程计、官方 C++ Offboard 例程 5 m 起飞悬停，以及工作区构建。原始结果摘要和已知限制见 [`docs/m0-environment-audit.md`](docs/m0-environment-audit.md)。M1 尚未开始。

## 固定版本

| 组件 | 版本 |
|---|---|
| Ubuntu | 22.04.5 LTS（WSL2） |
| ROS 2 | Humble |
| Gazebo | Harmonic 8.15.0 |
| PX4 Autopilot | v1.17.0 |
| px4_msgs | v1.17.0 |
| px4_ros_com | `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af` |
| Micro XRCE-DDS Agent | v2.4.3 |

版本锁定位于 [`dependencies.repos`](dependencies.repos)。不要单独升级其中一个 PX4 相关仓库。

## 仓库布局

```text
.
|-- .local/                  # 项目内工具与 Python 用户包（不提交）
|-- .wsl/                    # Ubuntu 22.04 WSL 数据（不提交）
|-- build/ install/ log/     # Colcon 输出（不提交）
|-- docs/
|-- external/                # PX4 与 Agent 外部源码（不提交）
|-- scripts/
|-- src/                     # Colcon 工作空间源码
`-- dependencies.repos
```

所有项目生成的下载、缓存、日志和临时文件均应留在仓库或项目内 WSL 中。`scripts/project-env.sh` 会隔离 Windows PATH，并将常见缓存和日志定向到项目目录。

## 从零搭建

以下步骤以 Windows PowerShell 当前目录为仓库根目录、仓库位于 E 盘为例。若路径不同，只需替换 WSL 中的仓库路径。

### 1. 安装项目内 WSL

在 PowerShell 中执行：

```powershell
wsl --install --distribution Ubuntu-22.04 --location "$((Get-Location).Path)\.wsl\Ubuntu-22.04"
wsl -d Ubuntu-22.04
```

首次进入 Ubuntu 时完成用户名和密码初始化。之后在 Ubuntu 中进入仓库并建立稳定挂载点：

```bash
cd "/mnt/e/codex-work space/project19-无人机开发agent"
bash scripts/mount-project.sh
cd /opt/project19
```

WSL 完全停止后 bind mount 会消失；每次重新启动 WSL 后先再次运行 `scripts/mount-project.sh`。

### 2. 安装 ROS 2 Humble

```bash
sudo apt-get update
sudo apt-get install -y curl software-properties-common
sudo add-apt-repository -y universe

mkdir -p downloads
curl -L \
  -o downloads/ros2-apt-source_1.2.0.jammy_all.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.jammy_all.deb
sudo apt-get install -y ./downloads/ros2-apt-source_1.2.0.jammy_all.deb
sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-ros-gzharmonic \
  python3-colcon-common-extensions \
  python3-vcstool
```

### 3. 获取固定版本源码

```bash
cd /opt/project19
vcs import < dependencies.repos
```

`external/COLCON_IGNORE` 用于阻止 Colcon 扫描第三方源码，不要删除。

### 4. 安装 PX4 仿真依赖

```bash
cd /opt/project19
export PYTHONUSERBASE=/opt/project19/.local/python
export PIP_CACHE_DIR=/opt/project19/.cache/pip
bash external/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```

重新打开 Ubuntu 终端后继续。`--no-nuttx` 只跳过本阶段不需要的真机交叉编译工具链，不影响 SITL。

### 5. 构建 Micro XRCE-DDS Agent

```bash
cd /opt/project19
cmake \
  -S external/Micro-XRCE-DDS-Agent \
  -B .cache/micro-xrce-dds-agent-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/project19/.local/micro-xrce-dds-agent
cmake --build .cache/micro-xrce-dds-agent-build --target install -j "$(nproc)"
```

### 6. 构建 ROS 2 工作区

```bash
source /opt/project19/scripts/project-env.sh
colcon build --symlink-install
```

预期只构建 `px4_msgs` 和 `px4_ros_com`：

```text
Summary: 2 packages finished
```

可执行上游测试：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

固定版本的 `px4_ros_com` 自身不满足 Humble 默认 lint 规则，当前结果为 `692 tests, 0 errors, 672 failures, 7 skipped`。这些是未修改上游源码的格式、版权和静态检查失败，不代表项目测试全绿；详见 M0 审计。

## M0 运行验收

每个新 Ubuntu 终端都先执行：

```bash
source /opt/project19/scripts/project-env.sh
```

### 终端 1：启动 DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

看到 `running... | port: 8888` 后保持运行。

### 终端 2：启动 PX4 SITL

```bash
HEADLESS=1 make -C external/PX4-Autopilot px4_sitl gz_x500
```

看到 `pxh>` 后，可用以下命令确认 DDS：

```text
uxrce_dds_client status
```

无 QGroundControl 的纯本地仿真会被 GCS 丢失保护阻止解锁。仅在本机 SITL 中执行：

```text
param set NAV_DLL_ACT 0
commander takeoff
commander status
listener vehicle_local_position -n 1
commander land
```

不要把 `NAV_DLL_ACT=0` 用于真机。命令行起飞默认高度约 2.5 m，降落后应看到 `Disarmed by landing`。

### 终端 3：验证 ROS 2 里程计

```bash
ros2 topic echo --once /fmu/out/vehicle_odometry
```

命令应输出一帧包含 `position`、`q` 和 `velocity` 的消息并以状态码 0 结束。

### 终端 3：运行官方 Offboard 例程

先确保飞行器已经降落并解除武装，然后运行：

```bash
ros2 run px4_ros_com offboard_control
```

示例会切换至 Offboard、解锁并将 NED 位置设为 `(0, 0, -5)`。在 PX4 控制台中验证：

```text
commander status
listener vehicle_local_position -n 1
```

预期导航模式为 `Offboard`，`z` 接近 `-5 m` 且速度接近零。结束时先在 PX4 控制台输入 `commander land`，再停止 Offboard 节点，并等待 `Landing detected` 与 `Disarmed by landing`。

## M0 验收结果

- [x] `gz_x500` SITL 启动并完成命令行起飞/降落
- [x] ROS 2 收到 `/fmu/out/vehicle_odometry`
- [x] 官方 C++ Offboard 例程起飞至 5 m 并稳定悬停
- [x] 本 README 记录从零环境搭建和复现步骤
