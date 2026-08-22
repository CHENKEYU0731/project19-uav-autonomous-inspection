# M2 感知与局部建图验收审计

检查日期：2026-08-22（Asia/Shanghai）

## 结论

M2 三项验收标准均获得直接仿真与 rosbag 证据。单条 ROS 2 launch 命令在
Gazebo Harmonic 8.15.0 与 PX4 v1.17.0 SITL 中启动深度相机、TF、局部建图、
航点飞行和证据记录；无人机悬停及移动 2.237 m 后，占据栅格均与同一 Gazebo
场景的右侧墙体对齐。建图中位和全段平均频率均为 10.00 Hz，最大消息间隔为
0.10 s，处理延迟中位数为 0.988 ms、P95 为 1.561 ms。

本审计只证明当前软件在环场景中的 2.5D 局部建图行为，不证明未知真机传感器、
复杂材质、动态障碍物或大规模场景中的性能。失败的调试 bag 不作为验收证据。

## 实现边界

- `drone_sim`：项目内 x500 深度相机模型，以及包含墙体、门框和圆柱的巡检场景
- `drone_perception`：32 位浮点深度图投影、固定高度切片、射线清空和占据端点标记
- 局部地图为 `120 x 120`、分辨率 `0.1 m` 的滚动 2D 占据栅格，固定在 `map` 坐标系
- `px4_tf_broadcaster` 将 PX4 NED/FRD 位姿转换为 ROS ENU/FLU，并发布相机静态外参
- `drone_bringup`：一键启动 Gazebo、DDS Agent、PX4、桥接、TF、建图、任务和 rosbag
- 运行诊断发布处理延迟、输出频率、使用的深度点数和占据单元数
- 非有限深度、越界深度、错误图像布局、无效相机内参和不受支持的 PX4 坐标系均被拒绝

无 QGroundControl 的 M2 仿真需要关闭 GCS 数据链丢失动作。x500 airframe 会在
环境参数覆盖之后把 `NAV_DLL_ACT` 默认值恢复为 2，因此 M2 使用项目内
`scripts/px4-headless-rcS`：先执行官方 `etc/init.d-posix/rcS`，再以
`param set-default NAV_DLL_ACT 0` 对当前 PX4 进程做后置覆盖。该值不保存到 PX4
参数文件，不修改 `external/PX4-Autopilot`，也不得用于真机。

## 构建与测试证据

执行：

```bash
source /opt/project19/scripts/project-env.sh
bash -n scripts/run-px4-sitl.sh scripts/px4-headless-rcS
python3 -m py_compile \
  scripts/analyze_m2_mapping.py \
  src/drone_bringup/launch/local_mapping.launch.py
colcon build --symlink-install \
  --packages-select drone_sim drone_perception drone_controller drone_bringup \
  --allow-overriding drone_sim drone_perception drone_controller drone_bringup \
  --event-handlers console_direct+
colcon test \
  --packages-select drone_sim drone_perception drone_controller drone_bringup \
  --event-handlers console_direct+
```

结果：

```text
Summary: 4 packages finished [16.9s]
DepthGridMapperTest: 7/7 passed
FrameConversions/TimestampAligner: 10/10 passed
Waypoint/Mission GTest: 8/8 passed
M1 bringup/wrapper pytest: 13/13 passed
M2 simulation assets pytest: 4/4 passed
M2 launch pytest: 5/5 passed
M2 evidence analysis pytest: 19/19 passed
Summary: 4 packages finished [36.0s]
Package test-result: perception 19, controller 9, bringup 45; 0 errors/failures
```

构建启用 `-Wall -Wextra -Wpedantic`，本次输出没有编译警告。测试覆盖深度投影、
射线栅格、滚动地图、NED/FRD 到 ENU/FLU 转换、非法及非有限输入、仿真资产、
launch 依赖与进程所有权、RViz 退出门禁、headless PX4 后置参数顺序，以及证据
验证器对缺失深度输入、失败飞行序列、动态或错误相机外参、陈旧 TF、空栅格、
低频、平均频率不足、长卡顿、非连续飞掠和低空爬升冒充悬停的拒绝路径。
ShellCheck（排除运行时外部源码路径的 `SC1091`）、`ament_uncrustify`、
`ament_flake8` 与纯 C++ 核心的 cppcheck 均通过；当前环境未安装 clang-tidy。

全工作区的 `colcon test-result --verbose` 仍会混入固定上游 `px4_ros_com` 的历史
lint 失败；本次结论仅来自上述四个项目包的测试结果目录，不能外推为第三方源码
测试全绿。

## 一键运行证据

执行：

```bash
source /opt/project19/scripts/project-env.sh
ros2 launch drone_bringup local_mapping.launch.py use_rviz:=true
```

正式成功运行日志：`log/m2/launch_runtime_verified.log`

关键输出：

```text
Takeoff target reached; starting waypoint 1/4
Waypoint 1/4 reached
Waypoint 2/4 reached
Waypoint 3/4 reached
Waypoint 4/4 reached
Home hover point reached; requesting landing
Disarmed by landing
Settled target error summary: samples=6 mean=0.101 m max=0.189 m
Mission complete: vehicle landed and disarmed
```

顶层 launch 状态码由无管道的 WSL 进程直接返回为 0，并记录于
`log/m2/launch_runtime_verified.exitcode`。任务成功后，launch 向 Agent、PX4、Gazebo、桥接、TF、
建图和 rosbag 进程发送停止信号；子进程日志中的 SIGINT、PX4 状态码 130 和
Gazebo 停止时状态码 -11 均发生在成功终态之后，属于编排关闭。复核时已确认无
PX4、Gazebo、DDS Agent、RViz、控制器或建图节点残留。

正式证据 bag：`log/m2/mapping_20260822_091517/`

```text
Bag size: 43.3 MiB
Duration: 95.754280670s
Messages: 33388
/camera/depth/image_raw: 476
/camera/depth/camera_info: 476
/local_occupancy_grid: 354
/drone_perception/diagnostics: 354
/tf: 3538
/tf_static: 1
/fmu/out/vehicle_status_v1: 77
/fmu/out/vehicle_land_detected: 45
```

早期 `mapping_20260822_065311`、`065523`、`070002` 是定位 headless PX4 起飞
门禁期间产生的失败记录；`071358` 早于最终生产代码，`081646` 的外层退出码记录
无效。`082409` 早于最终时间戳与边界射线修复，`090849` 未加载项目运行环境，
`090954` 的飞行虽完成但顶层退出码记录被 PowerShell 展开破坏。它们均不参与最终验收指标。

## 坐标系与障碍物对齐证据

TF 链为：

```text
map -> base_link -> camera_optical_frame
```

`map` 未作为任何 TF 的子坐标系，因而是固定根坐标系。每一帧栅格均以 `map`
为 frame；栅格中心与最近 `map -> base_link` 位姿的最大时间差为 7.997 ms，中心
最大位置误差为 0.0086 m。飞行期间 `base_link` 平面移动 2.2366 m，证明移动后
窗口不是悬停样本的重复。

分析脚本把每个占据单元的世界坐标与 `inspection.sdf` 中墙段、门框和圆柱的碰撞
几何比较，容差为 `max(0.25 m, 2 x 栅格分辨率)`。结果为：

| 窗口 | 栅格帧数 | 占据单元累计数 | 对齐率 | 匹配障碍物 |
|---|---:|---:|---:|---|
| 连续低速悬停 | 12 | 360 | 100.0% | `right_wall` |
| 平移至少 1 m 后 | 69 | 2234 | 100.0% | `right_wall` |

悬停窗口按时间顺序选择，不依据对齐率挑选：持续至少 1.0 s、高度至少 2.3 m、
平均路径速度不超过 0.2 m/s、垂直范围不超过 0.05 m，且三维漂移不超过 0.3 m。
本次窗口平均速度 0.194 m/s、垂直范围 0.024 m。两窗口均超过验证器 50% 的最低
对齐阈值且不是空栅格；当前证据只证明相机视野内的右侧墙体，不声称单次视角
重建了场景全部障碍物。

![M2 悬停及移动后的局部栅格对齐](assets/m2-local-grid.png)

## 频率与延迟证据

从正式 bag 的栅格时间戳和诊断消息重新计算：

| 指标 | 结果 | 门槛 |
|---|---:|---:|
| 栅格样本数 | 354 | 至少 10 |
| 栅格中位频率 | 10.00 Hz | 至少 5 Hz |
| 栅格全段平均频率 | 10.00 Hz | 至少 5 Hz |
| P95 消息间隔 | 0.10 s | 不超过 0.20 s |
| 最大消息间隔 | 0.10 s | 不超过 0.50 s |
| 处理延迟中位数 | 0.988 ms | 记录值 |
| 处理延迟 P95 | 1.561 ms | 记录值 |
| 处理延迟最大值 | 2.030 ms | 记录值 |

频率门槛同时检查中位频率、全段平均频率、P95 间隔和最大间隔，避免任一汇总值
掩盖整体低频或长时间卡顿。诊断频率的首个预热样本为 0 Hz，因此最小值不用于
验收。机器可读结果位于 `docs/assets/m2-mapping-metrics.json`；其中还记录 bag、
分析器、world 及 16 个运行关键源码文件的 SHA-256。正式 bag SHA-256 为
`5429ba3c23a7498298dccad7701fdd3224616d833b5c025eb8d122ef7a5c0239`。

## RViz 证据

`use_rviz:=true` 使用项目内 `local_mapping.rviz`，固定坐标系为 `map`，同时显示
占据栅格和 TF。正式 launch 日志确认 RViz 创建了 120 x 120 地图。截图使用同一
正式 bag 做单遍 2 倍速回放，并用当前项目 RViz 配置重现：

```bash
ros2 bag play log/m2/mapping_20260822_091517 --clock --rate 2.0
rviz2 -d install/drone_bringup/share/drone_bringup/config/local_mapping.rviz
```

截图中 `Global Status: Ok`，且可见真实栅格、自由区、未知区、占据单元和 TF 标记：

![M2 RViz 局部占据栅格](assets/m2-rviz.png)

当前 WSLg/Mesa/OGRE2 组合在 RViz Map display 首帧报告 shader sampler 链接警告，
因此截图左侧 `Local Occupancy Grid` 行仍为红色；实际地图持续渲染，TF 与全局状态
正常。软件渲染及 OpenGL 3.3 复测均未消除该环境警告，因此将其保留为非阻塞
可视化风险，而不伪造全绿截图。

项目方案要求的里程碑短媒体已由同一正式 bag 生成到被忽略的本地文件
`log/m2/m2-local-mapping.gif`（900 x 600、60 帧）；它展示滚动栅格、无人机路径
与已知 world 几何，不作为实时 RViz 无警告的证据。

## 验收矩阵

| 验收项 | 状态 | 直接证据 |
|---|---|---|
| 悬停时 RViz 栅格与 Gazebo 障碍物一致 | 通过 | 12 帧连续低速悬停、100.0% 墙体对齐；正式 bag 的 RViz 回放截图 |
| 移动时地图更新且无明显错位 | 通过 | 平移 2.237 m 后 69 帧、100.0% 墙体对齐；栅格中心最大误差 0.0086 m |
| 建图至少 5 Hz 且有延迟统计 | 通过 | 354 帧中位/平均 10.00 Hz、P95/最大间隔 0.10 s；延迟中位 0.988 ms、P95 1.561 ms |

## 对抗式复核

- 指标文件指向当前最新且唯一参与验收的 `091517` bag；失败、旧代码或退出码无效的 bag 均明确排除
- 16 个运行关键源码哈希与当前工作树逐项匹配，分析器结论为 `accepted`
- TF 门禁同时校验父子关系、固定/动态属性、相机外参数值、地图根节点和 250 ms 内的时间配对，不接受仅名称匹配
- 每帧栅格与同时间戳深度及诊断对应；占据单元最少 23 个，不接受空图或伪造诊断计数
- 频率同时检查中位数、全段平均值、P95 间隔和最大间隔，不能由短时高频掩盖卡顿
- 节点与离线验证器都拒绝非 `32FC1` 深度输入；负向测试覆盖陈旧 TF、错误外参、失败飞行和非连续伪悬停
- 正式 launch 顶层退出码为 0，回放结束后复核无 PX4、Gazebo、DDS Agent、RViz、控制器、建图或 rosbag 进程残留

## 存储边界与已知限制

- bag、运行日志和 PX4 ULog 均位于被忽略的项目内 `log/` 或 `external/` 子目录
- 里程碑 GIF 位于被忽略的项目内 `log/m2/`，不会增加仓库体积
- 构建、缓存和临时文件位于项目内 `build/`、`install/`、`.cache/` 与 `.local/`
- 未发现本项目主动向 C 盘写入生成文件
- 当前地图是瞬时滚动 2.5D 切片，不是累积全局地图，也不保留离开视野的障碍物
- 建图节点没有深度输入超时看门狗；M3 规划器必须校验地图消息年龄，拒绝使用陈旧地图
- 当前相机外参是固定参数，尚未从 URDF/robot_state_publisher 自动生成
- 单次证据只覆盖静态墙体；动态障碍物和在线重规划属于 M3 范围
- 当前仅在 WSL2、Gazebo Harmonic 和 x500 项目模型中验收，不能外推为真机安全性
