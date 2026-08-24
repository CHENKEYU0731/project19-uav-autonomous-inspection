# M3 路径规划与自主避障阶段审计

检查日期：2026-08-23（Asia/Shanghai）

## 结论

M3 三项验收均有直接 SITL 与 rosbag 证据。固定主种子 `20260822` 的连续 10 场
正式真实栈批次为 `log/m3/randomized_20260823_170200/`；同一修正版分析器对全部
10 场统一重分析后为 `9/10 accepted`，达到至少 8/10 的门槛。唯一失败的第 7 场
保留顶层 launch 状态码 1 和 `dynamic blocker insertion hold timed out` 证据。

M3 据此验收完成，可将当前里程碑更新为 M4。项目尚未提交或创建 `v0.4-m3` tag；
这不影响三项运行验收结论，但在用户明确授权前不会擅自执行 Git 提交或打 tag。

## 实现与安全边界

- 规划使用固定 `map` 坐标系中的融合栅格；未知单元与占据单元都不可通行
- 只对测得的占据单元做 `0.50 m` 膨胀，未知区本身保持禁行
- A* 为确定性 8 邻接搜索，拒绝对角穿角，并限制地图、扩展节点和队列规模
- 折线路径只在栅格视线安全时裁剪，再按最大 `1.0 m/s`、`1.0 m/s²` 参数化
- 地图最大允许年龄为 `0.5 s`；陈旧地图、TF 失败、无路径或轨迹/状态 ID
  不匹配时不继续执行运动轨迹
- 动态插障先请求 `/drone_m3/insertion_hold`，控制器锁定请求瞬间的位置；编排器
  要求前向速度连续稳定 `0.5 s` 且障碍物仍有至少 `0.6 m` 前向余量后才调用
  Gazebo 服务
- 规划器缓存仍安全的活动路径；只有剩余路径被新地图阻断时才生成新轨迹 ID
- Gazebo 服务调用在后台线程执行，有超时与失败退出，顶层 launch 会传播必要
  子进程的失败状态

这些是软件在环安全门禁，不构成真机安全认证，也不证明未测试场景中的性能。

## 构建与测试证据

最后一次 C++ 行为修改后执行：

```bash
colcon build --symlink-install \
  --packages-select drone_planner drone_controller drone_bringup
colcon test --packages-select \
  drone_planner drone_controller drone_bringup \
  --event-handlers console_direct+
```

构建通过。当前统一 WSL 包级复核结果为 `drone_planner` 40、
`drone_controller` 10、`drone_bringup` 108，合计 158 tests、0 errors、
0 failures、0 skipped。分析器与重分析发布加固后的 Windows 聚焦回归另为
46/46 通过。
全工作区结果仍可能混入固定上游 `px4_ros_com` 的历史 lint 失败，本结论只引用
上述项目自有包。

测试覆盖陈旧地图、未知区禁行、障碍膨胀、不可达目标、对角穿角、确定性路径、
活动路径复用、轨迹 ID 切换、插障握手、固定位置 hold、失败状态传播、虚假重规划、
缺失碰撞证据、间隙不足、未降落、非零 launch 退出码和陈旧成功 metrics 覆盖。

## 正式动态插障证据

运行命令：

```bash
ros2 launch drone_bringup m3_autonomy.launch.py
```

顶层 launch 返回 0。正式 bag：`log/m3/planner_20260822_155300/`

```text
Bag size: 58.1 MiB
Duration: 114.460116549 s
Messages: 42458
/local_occupancy_grid: 449
/drone_planner/trajectory: 212
/drone_planner/status: 1145
/drone_m3/dynamic_blocker_event: 5
/drone_m3/insertion_hold: 3
/fmu/in/trajectory_setpoint: 292
/fmu/out/vehicle_local_position_v1: 2225
/world/inspection/contacts: topic present, 0 collision messages
```

使用当前分析器重新生成指标：

```bash
export PYTHONNOUSERSITE=1
python3 scripts/analyze_m3_planning.py \
  log/m3/planner_20260822_155300 \
  --metrics log/m3/planner_20260822_155300/m3_metrics.json \
  --launch-exit-code 0
```

结果：

| 指标 | 结果 |
|---|---:|
| 初始轨迹 ID | 1 |
| 重规划轨迹 ID | 2 |
| 重规划延迟 | 0.051156215 s |
| 原轨迹到障碍物中心线余量 | 0.037208762 m |
| 新轨迹到障碍物中心线余量 | 0.917682862 m |
| 分段插值后的实际飞行最小中心距离 | 0.937571170 m |
| 扣除 0.35 m 飞行器和 0.22 m 障碍物半径后的间隙 | 0.367571170 m |
| 重规划后运动 setpoint 数 | 96 |
| 目标水平误差 | 0.029244258 m |
| 障碍物碰撞数 | 0 |
| 落地并解除武装 | 是 |
| failsafe | 未观察到 |

机器可读结果位于同一 bag 目录的 `m3_metrics.json`。bag SHA-256 为
`326b6f6aa549c3192487d08cdb5d9fe328df2fa6bafce4e7b5adfbd3fb86e305`；报告记录
17 个运行关键源码/配置的 SHA-256，包括分析器自身。该次单场指标记录的分析器哈希为
`1444452ebaeb127f4329aeb41251c681857072402206b235887877f975876c4e`。

## 连续 10 场随机评测证据

正式批次命令使用固定主种子 `20260822`，10 场均由同一 runner 连续执行。原始
批次没有删除、续跑、换 seed 或选择性补跑。旧分析器把
`blocker_insertion_started -> first new trajectory` 的窗口错误当成规划延迟，
将 Gazebo `set_pose` RPC 等待也计入其中，导致第 3、9 场被误拒绝。正确口径是
服务完成事件 `blocker_inserted -> blocker_replan_confirmed`。

修正后执行：

```bash
python3 scripts/run_m3_randomized_evaluation.py \
  --output-root log/m3/randomized_20260823_170200 \
  --master-seed 20260822 \
  --reanalyze-only
```

runner 在分析前逐场核对 manifest、目录名、场景参数、launch exit code、bag 完整性
和原始 bag SHA-256，并拒绝重复 bag。全部新结果先写入独立版本目录，完成后才
原子更新 `current_reanalysis.json`；原始 `summary.json` 继续保留 `7/10 rejected`。
正式修正版证据目录为
`log/m3/randomized_20260823_170200/reanalysis_20260823_180847_646575/`。

| 场次 | seed | 障碍物 (x, y) m | 结果 | 重规划延迟 s | 实际间隙 m | 目标误差 m |
|---:|---:|---:|---|---:|---:|---:|
| 1 | 1132521290 | (-0.007, 1.455) | 通过 | 0.100443 | 0.281737 | 0.052211 |
| 2 | 229951562 | (-0.125, 1.491) | 通过 | 0.099749 | 0.377205 | 0.023881 |
| 3 | 274662160 | (0.120, 1.560) | 通过 | 0.099944 | 0.292314 | 0.021079 |
| 4 | 1226284750 | (-0.139, 1.716) | 通过 | 0.099433 | 0.337401 | 0.022616 |
| 5 | 1650254620 | (0.092, 1.512) | 通过 | 0.100017 | 0.314181 | 0.012053 |
| 6 | 1499969110 | (0.068, 1.732) | 通过 | 0.100264 | 0.331774 | 0.040382 |
| 7 | 309474163 | (0.142, 1.459) | 失败：launch 1 / hold 超时 | - | - | - |
| 8 | 304902817 | (0.144, 1.727) | 通过 | 0.099978 | 0.347688 | 0.053610 |
| 9 | 4095533 | (0.179, 1.686) | 通过 | 0.102805 | 0.316799 | 0.040878 |
| 10 | 578065601 | (-0.011, 1.547) | 通过 | 0.099926 | 0.299670 | 0.020404 |

汇总为 10 场、9 成功、1 失败、成功率 90%。9 个成功场均有新轨迹 ID、正实际
间隙、目标到达、落地与解除武装，且未观察到 failsafe；10 个 bag SHA-256 唯一。

## 失败诊断证据

- `planner_20260822_153410`：插障流程发布 `blocker_insertion_started` 后进入
  `blocker_insertion_failed`；该 bag 出现轨迹 ID `1..14`，不作为成功证据。
  修复增加固定位置 hold、连续低速稳定窗口和前向安全余量。
- `planner_20260822_154017`：障碍物成功插入并确认从 ID `4` 重规划到 `5`，但
  此后轨迹 ID 快速增长到 `32` 并出现 `NO_PATH`。修复改为复用仍安全的活动路径，
  只有剩余路径被阻断时才生成新 ID。
- 其他 `planner_*` 目录是开发诊断产物；只有 `155300` 的当前 metrics 被用于本节
  单次成功结论。

## 验收矩阵

| 方案验收项 | 状态 | 直接证据 |
|---|---|---|
| 起终点间有障碍物时自主绕障到达且不碰撞 | 单次通过 | `155300` 到达、降落、0 碰撞、实际间隙 0.368 m |
| 飞行途中加入新障碍物后重规划绕行 | 单次通过 | 插障后轨迹 `1 -> 2`，延迟 0.051 s，新轨迹远离障碍物 |
| 连续 10 次随机场景成功率至少 8/10 | 通过 | 固定种子连续 10 场，统一重分析 9/10；完整逐场表、bag 与哈希保留 |

## 对抗式复核与限制

- M3 结论同时依赖单次完整动态插障证据和连续 10 场正式批次，不以单元测试替代
  真实栈验收
- 成功判定要求插障发生在初始活动轨迹之后，并要求后续不同轨迹 ID、绕行几何、
  新轨迹运动、目标到达、落地、解除武装、零碰撞、正间隙和无 failsafe
- 目标判断先筛选巡航高度样本，再取水平最近点；下降阶段经过目标附近不能冒充到达
- 运动门禁同时检查非零速度和连续变化的位置目标；固定位置 hold 的模式切换不会
  被误报为继续运动
- 实际间隙按相邻位姿间线段计算，不只取离散采样点，不能漏掉采样间穿越
- 分析失败会原子覆盖旧 metrics 为 `accepted: false`，不会残留旧成功结论
- 随机场景只覆盖 runner 预先约束的障碍物位置与触发区间，不覆盖区间外几何、额外
  传感器噪声、性能压力、真机气动或通信异常
- bag、metrics、构建产物、缓存与临时文件均在当前仓库或其子目录；本次未主动向
  C 盘写入项目生成文件
- Windows 聚焦测试曾因 pytest 默认值在 C 盘生成 4 个小型临时目录；复核时已将
  它们精确移动到项目 `.cache/tmp/windows-pytest/relocated-c-temp/`，最终复测通过
  `--basetemp` 固定到项目目录，C 盘对应本轮 pytest 目录已清零
