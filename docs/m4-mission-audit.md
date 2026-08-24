# M4 任务状态机与完整巡检阶段审计

检查日期：2026-08-23（Asia/Shanghai）

## 结论

M4 三项验收均已有当前源码对应的直接证据。正式录像运行
`log/m4/inspection_video_20260823_220601/` 的顶层 launch 返回 0，任务分析器和
视频验证器均给出 `accepted: true`。任务完成动态避障，到达 2 个巡检点，跳过
1 个不可达点；模拟低电量触发后沿成功航点倒序回撤，返回 home，自动降落并
解除武装。

同一次连续运行录制的 Gazebo + RViz2 双画面视频时长 124.667 秒，未超过 3 分钟。
项目方案中的 M4 三项复选框已据此勾选，当前里程碑转入 M5。

## 实现与安全边界

- `drone_mission` 用 FSM 管理待命、起飞、巡检、异常处理、返航、降落和完成
- 每次状态或任务进度变化发布带连续序号的 `/drone_mission/event`
- 不可达点只在匹配当前目标的新鲜规划器结果持续超时后跳过，并记录原因
- 模拟低电量会停止剩余巡检，沿已成功航点倒序回撤，再返回 home
- 返航航点和 home 均通过现有规划器、地图年龄、轨迹授权及控制器安全门禁
- M2 的 `map`/TF 契约和 M3 的未知区禁行、碰撞检查、陈旧地图拒绝策略未放宽
- `demo_start_delay_s` 默认 0.0；仅非默认录制模式延迟控制器与任务节点，不延迟
  仿真、建图、规划器或 rosbag

这些结论仅适用于当前软件在环场景，不构成真机安全认证。

## 正式运行证据

录制与运行命令：

```bash
bash scripts/record_m4_demo.sh
```

顶层退出码：`0`。任务结束后的进程检查只匹配检查命令自身，无 PX4、Gazebo、
MicroXRCEAgent、rosbag、规划器、控制器、任务或建图节点残留。

| rosbag 项目 | 结果 |
|---|---:|
| 时长 | 232.763241020 s |
| 大小 | 88.7 MiB |
| 消息数 | 78,819 |
| `/drone_mission/event` | 11 |
| `/drone_planner/goal` | 6 |
| `/drone_planner/trajectory` | 374 |
| `/world/inspection/contacts` | topic 存在，0 条碰撞消息 |

分析命令：

```bash
python3 scripts/analyze_m4_mission.py \
  /opt/project19/log/m4/inspection_video_20260823_220601 \
  --metrics /opt/project19/log/m4/inspection_video_20260823_220601/m4_metrics.json \
  --launch-exit-code 0
```

| 指标 | 结果 |
|---|---:|
| 分析器结论 | `accepted: true` |
| 到达巡检点 | 2 |
| 不可达点跳过 | 1 |
| 低电量异常 | 已触发 |
| 倒序回撤目标 | 1 |
| 初始/重规划轨迹 ID | 2 / 3 |
| 重规划延迟 | 0.215548131 s |
| 重规划后运动 setpoint | 226 |
| 最小实际净空 | 0.284303540 m |
| home 后 LAND 命令 | 7 |
| 最终状态 | landed + disarmed |
| 碰撞数 | 0 |
| failsafe | 未观察到 |

机器可读副本位于 `docs/assets/m4-mission-metrics.json`。分析器记录的 bag 目录级
复合 SHA-256 为
`82aa14e09208c4a205469a2b29c47c81999492d097012eba772a8c87242b67e7`；该值依次纳入
`metadata.yaml`、全部 `.db3` 文件名及各文件 SHA-256，不等同于对单个文件执行
`sha256sum`。本次 `.db3` 文件自身的 SHA-256 为
`7bbf1ab908f59eca19dd2e0d5e2aff210d8ac35401cc24e53c71ac8357009950`。
指标记录了 14 个运行关键源码、配置、world 和分析器文件的 SHA-256；其中
`mission_node.cpp` 为
`17eb6ac6705ae7aad37f4b2146d897eb06d775f372c9ce96a976ab77beffb10c`，
`m4_inspection.launch.py` 为
`1ea2bc538ab47e900a300c7289c5dfb5cd6bcea39dc0cc81030767b6543cfc53`。

高负载录像运行中，跨 topic 的 rosbag 接收时间出现约 5 微秒倒序。分析器仍保留
rosbag 接收时间，以便与位姿和飞控证据保持同一时间基准，但只对规划目标匹配允许
最多 1 毫秒跨 topic 接收乱序；同时要求目标坐标匹配，并且每条规划目标消息最多
消费一次。该边界有聚焦回归测试覆盖。

## 演示视频证据

最终视频位于 `docs/assets/m4-inspection-demo.mp4`。它来自上述同一次不中断的正式
录像运行，仅从原始连续录屏的 100 秒处开始裁切，以移除仿真和可视化初始化等待；
没有拼接不同运行。任务开始、动态避障、异常处理、返航、降落和解除武装均保留在
最终视频中。

| 视频项目 | 结果 |
|---|---:|
| 时长 | 124.667 s |
| 分辨率 | 2400 x 1080 |
| 编码/帧率 | H.264 / 12 fps |
| 大小 | 1,518,567 bytes |
| Gazebo 窗口 | `(1, 1, 1200 x 1080)` |
| RViz2 窗口 | `(1202, 2, 1200 x 1080)` |
| SHA-256 | `f09f73feb9a4a8254d9090b8075ce476bf70a92f6dbcef03a5c924bf9b7c7f88` |

视频验证器在 15%、50% 和 85% 三个任务时间点分别抽帧，确认左右画面均非黑帧、
包含可辨识场景细节且随任务变化，结论为 `accepted: true`。机器可读结果位于
`docs/assets/m4-inspection-demo-metrics.json`，可视化抽帧位于
`docs/assets/m4-inspection-demo-contact-sheet.png`。视频本体按仓库存储规则保留在
本地并由 `.gitignore` 排除，不提交大文件。

## 事件链复盘

正式 bag 中 11 个事件按序为：

```text
STANDBY -> TAKEOFF                     mission_started
TAKEOFF -> INSPECTING                 takeoff_completed
INSPECTING -> INSPECTING              waypoint_reached (waypoint 1)
INSPECTING -> HANDLING_EXCEPTION      waypoint_unreachable (waypoint 1)
HANDLING_EXCEPTION -> INSPECTING      waypoint_skipped (next waypoint 2)
INSPECTING -> INSPECTING              waypoint_reached (waypoint 3)
INSPECTING -> HANDLING_EXCEPTION      low_battery
HANDLING_EXCEPTION -> RETURNING_HOME  inspection_interrupted
RETURNING_HOME -> RETURNING_HOME      return_waypoint_reached (breadcrumb 0)
RETURNING_HOME -> LANDING             home_reached
LANDING -> COMPLETE                   landing_completed
```

分析器不仅检查事件名称和顺序，还将巡检到达、回撤到达和 home 到达事件与同期
TF 位姿及实际发布的规划目标交叉验证，并要求 `home -> LAND -> landed -> disarmed
-> complete` 的严格时序。

## 验收矩阵

| 方案验收项 | 当前状态 | 直接证据 |
|---|---|---|
| 一条命令完成全流程，含避障和异常处理 | 通过 | 正式录像运行 launch 0；重规划、不可达跳过、低电量返航、降落均被接受 |
| 状态机日志可复盘每次切换 | 通过 | 11 个连续结构化事件及 TF/目标/飞控状态交叉验证 |
| 双画面演示视频不超过 3 分钟 | 通过 | 124.667 秒、2400 x 1080；双画面非黑且动态，视频验证器接受 |

## 对抗式复核与限制

- 旧 `formal5`、`formal6` 和布局探测运行不再代表 M4 最终验收；本报告只引用
  同时生成正式 bag 与最终视频的 `inspection_video_20260823_220601`
- 证据接受同时要求顶层 launch 退出码 0、结构化事件连续、目标与 TF 匹配、真实
  动态重规划、重规划后继续运动、home 后 LAND、落地、解除武装、碰撞 topic 存在
  且无 x500 碰撞，以及未观察到 failsafe
- launch 收尾阶段会向必需子进程发送 SIGINT；本次 Gazebo、RViz2、rosbag、规划、
  控制和任务进程均干净退出，PX4 wrapper 与 DDS Agent 在超时后由 launch 升级为
  SIGTERM。它们发生在任务完成并写完 bag 之后，顶层退出为 0，不是飞行中崩溃
- 自动视频检查基于三个分散时间点，不能单独证明每一帧语义正确；因此同时保留联系
  表、正式任务 bag、结构化事件和人工可播放的视频，避免用单一视觉指标替代任务证据
- 当前运行证明的是这一固定 world 和参数组合，不证明所有未知环境、传感器异常、
  资源压力或真机条件下均可安全完成
- bag、metrics、临时录制帧和诊断文件均位于当前仓库或子目录，本轮未主动将项目
  生成文件写入 C 盘
