# M1 Offboard 航点飞行验收审计

检查日期：2026-08-22（Asia/Shanghai）

## 结论

M1 三项验收标准均已获得直接证据。一条 ROS 2 launch 命令在 PX4 v1.17.0
与 Gazebo Harmonic 8.15.0 SITL 中完成起飞、4 个航点、返回起点上方、自动
降落和解除武装，全程无需人工输入。

本审计只证明本地软件在环仿真行为，不证明真机安全性。主要验收指标采用每个
目标段最后 1 秒的 300 个稳态样本；同时公开包含航点阶跃和机动过程的全任务
误差，避免只报告有利口径。

## 实现边界

- `drone_controller`：10 Hz Offboard 心跳和位置设定点、任务状态机、航点判断
- `drone_bringup`：预检项目内依赖，一键启动 Agent、PX4 SITL、控制节点和 rosbag
- PX4 以官方 `-d` 模式运行，不启动交互式 `pxh`，避免提示输出阻塞 launch 事件队列
- NED 目标高度按 `home.z - takeoff_altitude_m` 计算，不假设起点 `z=0`
- Offboard 和解锁必须由 `VehicleStatus` 确认后才进入起飞状态
- 命令拒绝、启动超时、航段超时、失去 Offboard 或 failsafe 均有有界失败路径
- 已武装时的异常优先请求降落；正常完成或失败均使节点退出并返回对应状态码
- 发出降落命令后，只要 PX4 仍处于 Offboard 就继续发送心跳，直至模式切换或解除武装

`NAV_DLL_ACT=0` 仅通过本次 PX4 SITL 进程环境变量覆盖，用于无 QGroundControl
的本地仿真。该配置不得用于真机。

## 构建与测试证据

执行：

```bash
source /opt/project19/scripts/project-env.sh
colcon build --symlink-install \
  --packages-select drone_controller drone_bringup \
  --allow-overriding drone_controller drone_bringup \
  --event-handlers console_direct+
colcon test --packages-select drone_controller drone_bringup \
  --event-handlers console_direct+
colcon test-result --test-result-base build/drone_controller --verbose
colcon test-result --test-result-base build/drone_bringup --verbose
```

结果：

```text
Summary: 2 packages finished
[==========] Running 8 tests from 3 test suites.
[  PASSED  ] 8 tests.
drone_controller: 9 tests, 0 errors, 0 failures, 0 skipped
drone_bringup: 10 tests, 0 errors, 0 failures, 0 skipped
```

8 个 GTest 覆盖三维半径、连续稳定时间、离开半径重置、显式重置、非法配置、
非零起点 `z` 下的相对起飞高度、遥测有效性以及落地/解锁终态。9 个 pytest
覆盖证据完整性、launch 退出码传播、依赖异常退出、依赖预检、控制器成功后自动
关闭依赖、控制器失败后非零退出与依赖清理，以及 PX4 daemon 包装器的参数与
进程组清理。构建启用
`-Wall -Wextra -Wpedantic`，本次输出无编译警告。全工作区仍保留 M0 记录的上游
`px4_ros_com` lint 失败，不能把包级通过外推为上游测试全绿。

无 PX4 数据时另执行 `startup_timeout_s:=0.5` 的失败路径测试。节点在 0.5 秒后
输出 `Mission failed: timed out waiting for PX4 position and status`，并以状态码 1
退出，证明启动等待有界且不会静默成功。

## 一键飞行证据

执行：

```bash
ros2 launch drone_bringup waypoint_mission.launch.py
```

关键控制节点输出：

```text
Mission initialized at home [0.01, 0.02]; priming Offboard
Offboard and arm commands sent; awaiting PX4 confirmation
PX4 confirmed Offboard and armed; taking off to 2.50 m above home
Waypoint 1/4 reached
Waypoint 2/4 reached
Waypoint 3/4 reached
Waypoint 4/4 reached
Inspection waypoints complete; returning home
Home hover point reached; requesting landing
Settled target error summary: samples=6 mean=0.127 m max=0.185 m
Mission complete: vehicle landed and disarmed
```

PX4 同时输出：

```text
Armed by external command
Takeoff detected
Landing detected
Disarmed by landing
```

控制节点干净退出后，launch 立即向 Agent、PX4 和 rosbag 发送停止信号。PX4
包装进程与 Agent 会被 launch 记录为信号终止；这是任务完成后的编排关闭，不是
飞行失败。顶层 launch 命令状态码为 0，rosbag 在退出前完成缓存写入。退出后
单独检查 Agent、PX4、Gazebo、控制器、rosbag 和 launch，均无残留进程。

## 误差与轨迹证据

6 个稳定到达点的误差依次为：

| 目标 | 误差（m） |
|---|---:|
| 起飞悬停点 | 0.181 |
| 航点 1 | 0.071 |
| 航点 2 | 0.078 |
| 航点 3 | 0.185 |
| 航点 4 | 0.115 |
| 返回起点上方 | 0.135 |
| **平均值** | **0.127** |
| **最大值** | **0.185** |

从同一 rosbag 按最新位置设定点对齐实际位置，得到：

| 统计口径 | 样本数 | 平均误差（m） | RMSE（m） | 最大误差（m） |
|---|---:|---:|---:|---:|
| 每个目标段最后 1 秒的稳态窗口 | 300 | 0.119 | 0.133 | 0.273 |
| 包含目标阶跃与全部机动过程 | 1181 | 1.206 | 1.589 | 4.022 |

稳态窗口直接检验悬停和到点保持质量，是 M1 `< 0.3 m` 验收所采用的口径。
全任务值在每次目标从一个角点瞬间跳到下一个角点时包含约 2-4 m 初始误差，
因此不能与稳态阈值混用；它反映了当前阶跃位置控制尚未做轨迹平滑的事实。
原始机器可读结果位于 `docs/assets/m1-tracking-metrics.json`。

rosbag：`log/m1/trajectory_20260822_051355/`

```text
Duration: 31.210341406s
Messages: 1913
/fmu/out/vehicle_local_position_v1: 1557
/fmu/in/trajectory_setpoint: 238
/fmu/out/vehicle_status_v1: 68
/fmu/out/vehicle_land_detected: 41
/fmu/out/vehicle_command_ack: 9
```

PlotJuggler 3.17.2 的 ROS 2 bag 插件已直接打开该数据库，并在同一图中显示
`/fmu/in/trajectory_setpoint/position[0]` 与
`/fmu/out/vehicle_local_position_v1/x`。验证布局保存为
`docs/m1-waypoint-layout.xml`。README 中的四联图由
`scripts/plot_m1_trajectory.py` 从同一 bag 的 238 个目标样本和 1557 个位置样本
生成，同时写出上述误差 JSON；该图不冒充 PlotJuggler 截图。

本地归档 GIF 为 `log/m1/m1-waypoint-flight.gif`，尺寸 704 x 704，共 131 帧，
由同一 rosbag 回放生成。根据仓库规则，bag、运行日志和 GIF 位于被忽略的
`log/` 中，不纳入版本控制；静态轨迹图位于 `docs/assets/m1-trajectory.png`。

## 验收矩阵

| 验收项 | 状态 | 直接证据 |
|---|---|---|
| 一条命令完成起飞、4 个航点和降落 | 通过 | 控制节点 4/4 日志、PX4 起飞/降落/解锁日志、状态码 0 |
| 航点误差量化 | 通过 | 300 个稳态样本均值 0.119 m，RMSE 0.133 m，最大 0.273 m |
| 构建无警告且有单元测试 | 通过 | 两包构建输出；8 个 GTest、9 个 pytest；包级 0 failures |

## 已知限制

- 当前航点是阶跃位置设定点，拐角处存在可见超调；轨迹平滑属于 M3 范围。
- 当前仅在默认空场景、x500 模型和单次端到端运行中验收。
- M1 不包含障碍物、深度相机、建图或避障，不能据此声称自主避障能力。
- 全任务 RMSE 为 1.589 m，说明阶跃目标的机动过程误差较大；M3 需用平滑轨迹改善。
