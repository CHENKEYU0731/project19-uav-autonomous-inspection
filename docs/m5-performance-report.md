# M5 性能分析报告

检查日期：2026-08-24（Asia/Shanghai）

## 结论

在当前 WSL2 主机上，使用无缓存构建得到的
`sha256:a916ffe193482c3bcd74c3d1879e73bc67ecdf346f114e6b59b8e2814f32506f`
镜像运行完整无界面 M4。Compose 以 0 退出，新 bag
`log/m4/inspection_20260823_195113/` 经同一镜像内分析器判定为 `accepted: true`。

容器从创建到退出的 70 秒窗口内，共取得 35 个 `docker stats --no-stream` 样本。
完整系统平均使用约 1.66 个逻辑 CPU，P95 为 2.48 个逻辑 CPU，峰值为 3.01 个
逻辑 CPU；内存平均 624.5 MiB，P95 为 729.6 MiB，峰值为 903.6 MiB。该结果是
Gazebo、PX4、DDS Agent、全部 ROS 节点和 rosbag 的容器级合计，不是单节点剖析。

机器可读汇总位于
[`assets/m5-performance-metrics.json`](assets/m5-performance-metrics.json)。本报告是
项目方案 M5 的可选性能证据，不满足或替代任何一项 M5 硬验收标准。

## 测试环境

| 项目 | 值 |
|---|---|
| 主机 CPU | AMD Ryzen 7 5800H，8 核 16 线程 |
| WSL2 可见内存 | 7.459 GiB |
| WSL2 内核 | 6.6.87.2-microsoft-standard-WSL2 |
| Docker Engine | 29.1.3 |
| Docker Compose | 2.40.3 |
| Gazebo | Harmonic 8.15.0，无界面模式 |
| 镜像 | `project19-inspection:clean-nocache` |

## 当前 M4 资源运行

本次先删除旧 Compose 容器，再启动采样器等待新容器出现，随后运行：

```bash
docker compose up --abort-on-container-exit \
  --exit-code-from inspection inspection
```

采样器从容器创建后开始循环执行：

```bash
docker stats --no-stream \
  --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.PIDs}}' \
  project19-inspection-inspection-1
```

| 指标 | 平均值 | 中位数 | P95 | 最大值 |
|---|---:|---:|---:|---:|
| CPU（100% = 1 个逻辑 CPU） | 165.846% | 168.320% | 247.620% | 300.790% |
| 内存 | 624.512 MiB | 728.600 MiB | 729.600 MiB | 903.600 MiB |
| 进程数 | 252.114 | 297 | 297 | 297 |

采样时间为 `03:51:07` 至 `03:52:17`，35 个样本的平均间隔约 2.059 秒。原始 CSV
为 `log/m5/m4-container-stats-complete.csv`，SHA-256 为
`a77ed941d89c6c92ece5a704cfecff80c9fbb5139d5e2745c543c1235c9b1fed`。

同一次运行的新 bag 分析结果：

| 指标 | 结果 |
|---|---:|
| M4 分析器 | `accepted: true` |
| 动态重规划延迟 | 0.099860502 s |
| 最小实际净空 | 0.318042747 m |
| 碰撞数 | 0 |
| failsafe | 未出现 |
| 最终状态 | 已降落并解除武装 |

bag 聚合 SHA-256 为
`837f3f4307ba4b32d60855c772bd4728a224bbfc2196e6031958ee6ad7b28e24`。
Compose 和分析器日志分别保存在
`log/m5/docker-compose-m4-performance-complete.log` 与
`log/m5/analyze-m4-performance-complete.log`。

## 建图与规划参考指标

建图延迟引用已验收 M2 正式 bag 的 354 个诊断样本：

| 建图指标 | 结果 |
|---|---:|
| 平均输出频率 | 10.00 Hz |
| 处理延迟中位数 | 0.9875 ms |
| 处理延迟 P95 | 1.5613 ms |
| 处理延迟最大值 | 2.0300 ms |

来源为 [`assets/m2-mapping-metrics.json`](assets/m2-mapping-metrics.json)，其源码哈希
和 bag 哈希已在 M2 审计中固定。该数据不是从本次 M4 CPU 采样运行重新计算。

规划参考采用已验收 M3 单次动态插障运行
`log/m3/planner_20260822_155300/`：重规划延迟为 0.051156215 秒，最小实际净空为
0.367571170 m。它用于展示独立规划基准；当前源码对应的 M4 同次运行值仍以前节的
0.099860502 秒为准。

## 证据边界

- CPU 和内存是整个容器的合计，不能据此声称某个 ROS 节点单独占用这些资源。
- `docker stats --no-stream` 约每 2 秒返回一次，采样点之间的瞬时峰值不可见。
- 无界面软件渲染未测量 GPU，不能外推 GUI 模式或独立显卡负载。
- M2 建图和 M3 规划参考来自各自正式验收运行，不是本次 M4 的同场测量。
- 本机性能不证明全新机器 30 分钟复现，也不证明 GitHub Actions、分支保护或提交
  历史满足 M5 验收。
