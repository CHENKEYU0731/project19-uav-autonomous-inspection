# M0 环境与验收审计

检查日期：2026-08-22（Asia/Shanghai）

## 结论

M0 四项验收标准均已在当前机器上获得直接运行证据。当前可以结束 M0，但不得把这些仿真结果外推为真机可用，也不得把上游 lint 失败描述为测试全绿。

## 环境与版本

| 项目 | 当前状态 | 证据 |
|---|---|---|
| Git 仓库 | 已初始化 | 分支 `codex/m0-bootstrap` |
| WSL2 | 已安装 | `Ubuntu-22.04`，WSL 版本 2；发行版数据位于项目 `.wsl/Ubuntu-22.04` |
| Ubuntu | 已验证 | `/etc/os-release`：`Ubuntu 22.04.5 LTS` |
| ROS 2 | 已安装 | `/opt/ros/humble/bin/ros2`；`ros-humble-desktop 0.10.0-1jammy.20260804.223343` |
| Gazebo | 已安装 | `gz sim --versions`：`8.15.0`；`ros-humble-ros-gzharmonic 0.244.12-3jammy` |
| PX4 | 已固定 | Git tag `v1.17.0` |
| px4_msgs | 已固定 | Git tag `v1.17.0` |
| px4_ros_com | 已固定 | commit `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af` |
| Micro XRCE-DDS Agent | 已固定 | Git tag `v2.4.3`；安装于 `.local/micro-xrce-dds-agent` |
| 包管理状态 | 正常 | `dpkg --audit` 无输出；`apt-get check` 成功 |

## 构建与测试证据

### Colcon 构建

执行：

```bash
source /opt/project19/scripts/project-env.sh
colcon build --symlink-install
```

结果：

```text
Finished <<< px4_msgs
Finished <<< px4_ros_com
Summary: 2 packages finished [5min 36s]
```

`colcon list` 仅列出 `px4_msgs` 和 `px4_ros_com`，证明 `external/COLCON_IGNORE` 正常隔离 PX4 和 Agent 第三方源码。

### Colcon 测试

执行：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

结果：

```text
Summary: 692 tests, 0 errors, 672 failures, 7 skipped
```

CTest 中只有上游 `px4_ros_com` 的 8 类 lint：copyright、cppcheck、cpplint、flake8、lint_cmake、pep257、uncrustify、xmllint。失败来自固定版本上游源码自身的版权头、制表符、行长、格式和 XML 风格，不来自本项目新增功能。M0 不修改第三方源码以制造“全绿”结果。

## 运行验收证据

### 1. SITL 与 DDS 通信

启动命令：

```bash
MicroXRCEAgent udp4 -p 8888
HEADLESS=1 make -C external/PX4-Autopilot px4_sitl gz_x500
```

关键输出：

```text
running... | port: 8888
Gazebo world is ready
world: default, model: x500_0
[px4] Startup script returned successfully
pxh>
```

PX4 控制台执行 `uxrce_dds_client status`：

```text
Running, connected
Using transport:     udp
Agent IP:            127.0.0.1
Agent port:          8888
timesync converged: true
```

### 2. 命令行起飞与降落

初次执行 `commander takeoff` 被 `No connection to the GCS` 安全检查拒绝。源码确认该门禁由 `NAV_DLL_ACT > 0` 触发；当前 SITL 参数值为 `2`。纯本地、无 QGroundControl 的仿真中执行：

```text
param set NAV_DLL_ACT 0
commander takeoff
```

关键输出：

```text
Ready for takeoff!
Armed by internal command
Using default takeoff altitude: 2.5 m
Takeoff detected
```

悬停状态：

```text
Armed
navigation mode: Hold
in failsafe: no
vehicle_local_position.z: -2.45032
vehicle_local_position.vz: 0.00246
```

执行 `commander land` 后：

```text
Landing at current position
Landing detected
Disarmed by landing
```

`NAV_DLL_ACT=0` 只适用于本次无 GCS 的本地 SITL 验收，禁止据此配置真机。

### 3. ROS 2 飞控里程计

执行：

```bash
ros2 topic echo --once /fmu/out/vehicle_odometry
```

命令以状态码 0 返回一帧完整消息，包含 `timestamp`、`position`、`q`、`velocity`、方差和坐标系字段。该证据来自 ROS 2 订阅端，不是 PX4 内部配置推断。

PX4 v1.17 的本地位置主题名为 `/fmu/out/vehicle_local_position_v1`；验收标准指定的 `/fmu/out/vehicle_odometry` 保持未版本化并可直接订阅。

### 4. 官方 C++ Offboard 例程

执行：

```bash
ros2 run px4_ros_com offboard_control
```

节点输出：

```text
Starting offboard control node...
[offboard_control]: Arm command send
```

PX4 直接状态：

```text
Armed
navigation mode: Offboard
user intended navigation mode: Offboard
in failsafe: no
vehicle_local_position.z: -4.99791
vehicle_local_position.vz: -0.00356
```

同时从 ROS 2 `/fmu/out/vehicle_odometry` 读取：

```text
position: [-0.00662, -0.01652, -4.99858]
velocity: [-0.02463, 0.01026, -0.00289]
```

这些数据共同证明官方示例进入 Offboard、起飞到约 5 m 并稳定悬停。随后执行 `commander land`，停止 Offboard 节点后 PX4 输出 `Landing detected` 和 `Disarmed by landing`。

## 存储边界

- WSL 发行版：`.wsl/Ubuntu-22.04`
- PX4 与 Agent 源码：`external/`
- ROS 2 外部源码：`src/px4_msgs`、`src/px4_ros_com`
- Colcon 输出：`build/`、`install/`、`log/`
- 工具、Python 包与缓存：`.local/`、`.cache/`
- 下载：`downloads/`
- PX4 ULog：被忽略的 `external/PX4-Autopilot/build/.../rootfs/log/`

上述生成物均在当前项目目录内，并由 `.gitignore` 排除。未发现本项目主动写入 C 盘的生成物。

## M0 验收矩阵

| 验收项 | 状态 | 直接证据 |
|---|---|---|
| `gz_x500` 启动并命令行起飞 | 通过 | PX4/Gazebo 启动输出；Hold 模式约 2.45 m；降落并解锁 |
| ROS 2 收到飞控里程计 | 通过 | `ros2 topic echo --once /fmu/out/vehicle_odometry` 状态码 0 和完整消息 |
| 官方 Offboard 起飞悬停 | 通过 | Offboard 模式、无 failsafe、PX4 与 ROS 2 高度均约 5 m |
| README 可从零复现 | 通过 | 固定版本、项目内存储、安装、构建、启动、验证及停止步骤均已记录 |

## 官方来源

- [PX4 Ubuntu Development Environment](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html)
- [PX4 ROS 2 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control.html)
- [PX4 Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [Microsoft WSL basic commands](https://learn.microsoft.com/windows/wsl/basic-commands)
