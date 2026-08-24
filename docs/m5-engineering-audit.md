# M5 工程化收尾审计

检查日期：2026-08-24（Asia/Shanghai）

## 当前结论

M5 的容器、Compose、CI、双语文档、架构说明、复现指南和仓库整洁门禁已进入候选
状态，但 M5 尚未验收。当前 WSL 环境已从隔离源码上下文执行无缓存 Docker build，
并用新镜像直接完成工具冒烟和完整 Compose M4 回归；这仍是已有 Docker 环境的
本机验证，不是独立全新机器。当前 Git 仓库也没有配置 GitHub remote，因此仍无法
证明全新机器 30 分钟复现、GitHub Actions 全绿或 `main` 分支保护。项目方案中的
M5 三项复选框必须保持未勾选。

早期 133 路径无缓存构建曾因 E 盘空间耗尽而在镜像导出阶段失败。2026-08-24 使用
官方 `wsl --manage Ubuntu-22.04 --move` 将完整 VHD 迁移到
`D:\codex-wsl\project19-Ubuntu-22.04` 后，当时 133 路径候选的无缓存构建、镜像
冒烟、完整 Compose M4 回归和统一 CI 均已重新通过。迁移后还补齐了
`/opt/project19` 的 WSL
启动自动绑定，并连续两次终止、重启发行版验证挂载恢复。VHD 未写回 C 盘。

## 已实现资产

| 范围 | 文件 | 当前证据边界 |
|---|---|---|
| 容器镜像 | `Dockerfile`, `.dockerignore`, `docker/entrypoint.sh`, `LICENSE` | 本机镜像构建和镜像内工具冒烟均通过；尚无全新机器证据 |
| 一键编排 | `compose.yaml` | 默认服务已完成完整无界面 M4；GUI 为显式 profile |
| CI | `.github/workflows/ci.yml`, `.github/ci.repos`, `scripts/ci.sh` | 本地同等命令可运行；尚无 GitHub workflow run |
| 仓库门禁 | `scripts/check_repository_cleanliness.py` | 拒绝脏工作树、生成目录、外部源码树、日志/rosbag/ULog/视频/临时格式和超过 10 MiB 的候选文件 |
| 英文入口 | `README.en.md` | 覆盖架构、Docker、原生构建、运行、测试、证据和安全边界 |
| 架构/模块 | `docs/architecture.md` | Mermaid 数据流、七个自有包职责和安全契约 |
| 复现说明 | `docs/reproduction-guide.md`, `scripts/run_m5_reproduction.sh` | 从 clone 前计时，拒绝既有项目镜像/容器/bag，只验证本次新 bag；独立机器来源仍需外部证明 |
| 性能分析 | `docs/m5-performance-report.md`, `docs/assets/m5-performance-metrics.json` | M4 容器资源、M2 建图延迟和 M3/M4 重规划耗时；不替代 M5 硬验收 |

## CI 范围说明

CI 获取固定版本的 `px4_msgs` 和 `px4_ros_com` 并执行 Colcon build，但测试和 lint
结论只覆盖七个项目自有包。`px4_ros_com` 已知的上游格式与版权失败不属于本项目
源码，不通过修改第三方代码或放宽项目检查来制造全绿。

项目 lint 直接使用 `ament_copyright`、`ament_cppcheck`、`ament_cpplint`、
`ament_uncrustify`、`ament_flake8`、`ament_pep257`、`ament_lint_cmake` 和
`ament_xmllint`。完整命令集中在 `scripts/ci.sh`，本地与 GitHub Actions 共用。

2026-08-24 从官方 release 下载 `actionlint v1.7.12` Linux amd64 资产到项目本地
`.cache/`，其 SHA-256
`8aca8db96f1b94770f1b0d72b6dddcb1ebb8123cb3712530b08cc387b349a3d8` 与官方
校验和一致。在 WSL 中使用 ShellCheck `0.8.0` 直接检查
`.github/workflows/ci.yml`，零诊断并返回 `0`；`pyflakes` 集成因工具未安装而禁用，
但当前 workflow 没有 Python `run` 块。此前使用 SchemaStore GitHub Workflow
schema、PyYAML 的 GitHub-compatible 布尔解析和 `jsonschema` 的补充结构校验也
通过，并识别 workflow `project-ci` 及 job `build-test-lint`。这两项都只是静态
检查，不证明 GitHub Actions 已实际运行或成功。

## 本地候选验证

2026-08-24 在项目 WSL 根目录重新运行：

```bash
CI_ALLOW_DIRTY=1 bash scripts/ci.sh
```

命令返回 `0`，直接输出包括：

- `Summary: 9 packages finished`
- 七个自有包测试结果依次为 `0 + 0 + 19 + 40 + 10 + 6 + 226 = 301`
  项，`0 errors, 0 failures, 0 skipped`
- 八类 Ament lint 均返回 `No problems found`；版权检查覆盖 53 个文件，
  Flake8 覆盖 27 个 Python 文件
- `repository cleanliness accepted: 130 candidate paths, worktree check skipped`

早期第一次全量并行运行中，`test_m5_engineering` 的 16 项断言均已执行完，但在生成
结果文件前触发 CTest 默认 `60 s` 超时；同一测试直接运行 `16 passed in 5.46 s`，
隔离 CTest 运行也通过。仅将该测试的 CTest `TIMEOUT` 调整为 `120 s` 后再次执行
完整 `scripts/ci.sh`，该测试在并发 I/O 压力下以 `16 passed in 60.34 s`、CTest
总耗时 `64.10 s` 通过，未修改断言或测试范围。加入本轮 25 项复现安全与仓库门禁
回归后，测试目标上限限定调整为 `180 s`，未修改断言、测试范围或失败语义。迁移
后的 133 路径完整 CI 中，M5 工程测试为 `44 passed in 90.84 s`，CTest 目标耗时
`94.50 s`。随后确认根目录 `findings.md`、`progress.md` 和 `task_plan.md` 的内容均已
归档到权威 M3/M4 审计，且 `task_plan.md` 的状态已经过期，因此删除这三个过程便笺，
并让整洁门禁只拒绝它们出现在仓库根目录。新增 3 个拒绝用例和 1 个子目录合法反例
后，最终统一 CI 中的 M5 工程测试为 `48 passed in 89.43 s`，CTest 目标耗时
`92.92 s`。清理及大小写绕过修复后的完整统一 CI 日志保存在
`log/m5/current_130_final_20260824/ci.log`，大小 `631168` bytes，SHA-256 为
`46c8389ebae7771a185f8efc1f07676c5e17d946ddfee488a7b6f5946561e1f8`；此前 133 路径日志保存在
`log/m5/current_133_after_vhd_move_20260824/ci.log`。

`CI_ALLOW_DIRTY=1` 只因当前里程碑尚未提交而跳过脏工作树拒绝，禁入路径和
10 MiB 大小门禁仍然执行。该本地结果证明统一 CI 入口在当前 WSL 环境可通过，
但不证明 GitHub Actions 已运行，也不满足最终干净工作树验收。

严格模式此前已证明会拒绝开发态脏工作树；本次检查时工作树仍为脏状态，因此不能
把 `CI_ALLOW_DIRTY=1` 的候选文件扫描外推为最终整洁。

## Docker 与 Compose 直接验证

为排除工作树中的构建产物、日志和外部源码污染，先从 Git 跟踪文件与未忽略的新文件
生成隔离上下文 `.cache/m5-clean-context-20260824-0302/`。该上下文共 130 个文件、
1,626,710 bytes；检查确认不含 `.git/`、`log/`、`build/`、`install/`、PX4、
`px4_msgs` 或 `px4_ros_com` 外部源码。

2026-08-24 在该隔离上下文运行无缓存、强制拉取构建：

```bash
sudo env GITHUB_MIRROR_PREFIX=https://ghfast.top/ \
  docker compose build --no-cache --pull inspection
```

命令以 `0` 退出，耗时 `25:53.78`。最终镜像 ID/manifest digest 为
`sha256:a916ffe193482c3bcd74c3d1879e73bc67ecdf346f114e6b59b8e2814f32506f`，
大小为 `3,917,623,619` bytes，并另存标签 `project19-inspection:clean-nocache`。
镜像内冒烟检查直接确认 Gazebo `8.15.0`、`drone_bringup` 安装前缀
`/opt/project19/install/drone_bringup` 和 `MicroXRCEAgent` 可执行文件
`/opt/project19/.local/micro-xrce-dds-agent/bin/MicroXRCEAgent` 均存在。

构建期间 Docker Hub 域名在当前网络多次解析到异常地址并超时，因此临时 Docker
daemon 使用 `https://docker.m.daocloud.io` registry mirror；GitHub 源码与 rosdep
原始文件使用 `https://ghfast.top/` 前缀。基础镜像和 Dockerfile frontend 均通过
registry mirror 拉取。该网络替代路径是本次成功的前置条件之一，不证明官方源在
其他机器或网络可用，也不把第三方镜像服务视为项目控制范围内的长期可用依赖。

随后在主项目目录使用这次新镜像运行：

```bash
sudo env GITHUB_MIRROR_PREFIX=https://ghfast.top/ \
  docker compose up --abort-on-container-exit \
  --exit-code-from inspection inspection
```

容器以 `0` 退出，并新生成 `log/m4/inspection_20260823_193907/`。仅对这次新 bag
在同一镜像的 ROS 环境内运行 M4 分析器，结果为 `accepted: true`：到达 2 个巡检点，
动态重规划耗时 `0.200755876 s`，最小实际净空 `0.337881955 m`，碰撞数 0，
无 failsafe，最终降落并解除武装。分析器记录的 bag SHA-256 为
`d65c864e0d8effd671bccc70ee80b8d0e6f103c9c3e74c41ddc3d5903e5daee8`；原始
SQLite bag 文件 SHA-256 为
`47efdca699d8f8bbacb601634ac38c5c3bfca15748ae52f7d2d5956f801c235d`。

本次直接日志保存在 `log/m5/docker-compose-build-clean-nocache-mirror.log`、
`log/m5/docker-compose-m4-clean-nocache.log` 和
`log/m5/analyze-m4-clean-nocache.log`；本地统一 CI 日志为
`log/m5/ci-local-final.log`。隔离上下文位于项目 `.cache/`，所有本次生成文件均
留在 E 盘项目目录内；`log/` 与 `.cache/` 按仓库规则不提交。

### 133 路径重建与回归

迁移后使用 registry 镜像和 `GITHUB_MIRROR_PREFIX=https://ghfast.top/` 从当前
133 个候选路径执行：

```bash
docker compose build --no-cache --pull inspection
```

命令返回 `0`，耗时 `3294 s`，九个 ROS 包全部编译。镜像 ID/摘要为
`sha256:d72b4dc5626dbd7c8ba74228599dedb0ad7ee10d67b561557895c08e1e6b9e59`，
大小 `3,918,335,050` bytes。镜像内与宿主机的 `Dockerfile`、`.dockerignore`、
`src/drone_bringup/CMakeLists.txt` 和当时的 M5 测试文件哈希一致；冒烟检查确认
Gazebo `8.15.0`、`drone_bringup` 和 Micro XRCE-DDS Agent 均存在，安装后的
`drone_bringup` 树不含 `__pycache__`、`.pyc` 或 `.pyo`。

随后以 `--no-build` 使用该镜像完成 Compose M4 回归，只新增
`log/m4/inspection_20260824_051137/`。Compose 与分析器均返回 `0`，分析器记录
`accepted: true`：到达 2 个巡检点，重规划耗时 `0.102084152 s`，最小实际净空
`0.408361939 m`，碰撞数 0，无 failsafe，最终降落并解除武装。bag SHA-256 为
`4aa0a896eda15583e24cb431ba4da460167302bf59049fe0091a37dee7637988`。

直接证据位于 `log/m5/current_133_after_vhd_move_20260824/`。迁移未删除历史 bag
或其他证据；当前发行版注册路径、默认用户、ext4、ROS 2 和项目自动绑定均已复核。

同步审计文档并修复一处 M5 测试的 Flake8 格式后，对最终上下文执行缓存复建。命令
返回 `0`，耗时 `639 s`，镜像从上述无缓存构建的 `d72b...` 更新为
`sha256:7911098c4d33ad5e4435c0d61a6d8125503a63775fa3c755a61bb98c9111042b`。
Dockerfile、忽略规则、双语 README、本审计、bringup 安装规则和 M5 测试的镜像内外
哈希均一致；工具冒烟与字节码安装树检查再次通过。

最终镜像以 `--no-build` 运行后只新增
`log/m4/inspection_20260824_061438/`。Compose、两处日志捕获和分析器均返回 `0`，
总计时 `98 s`；分析器判定 `accepted: true`，重规划耗时 `0.099810272 s`，最小
实际净空 `0.391255268 m`，碰撞数 0，无 failsafe，最终降落并解除武装。bag
SHA-256 为 `c92422da9eaf29d2ca9cefb59b66e22f5ec79e9f1f5bc51e545afc2ac0469115`。
这次缓存复建证明最终运行时上下文与镜像一致；前一轮无缓存构建仍提供从基础依赖
开始的本机证据。两者都不替代独立全新机器验收。

补充运行全工作区 `colcon test` 时，项目自有测试通过，但固定上游
`px4_ros_com` 的版权与格式 lint 仍失败；`colcon test-result --verbose` 汇总为
`950 tests, 0 errors, 672 failures, 7 skipped`。统一 CI 因而继续只对七个自有包
给出全绿结论，不修改第三方源码，也不把上游失败隐藏为项目成功。

## 可选性能分析

同一无缓存镜像另运行一次完整 M4，并从容器创建到退出连续采集 35 个
`docker stats --no-stream` 样本，跨度 70 秒。对应新 bag
`log/m4/inspection_20260823_195113/` 经分析器判定为 `accepted: true`；容器 CPU
平均 `165.846%`、P95 `247.620%`、最大 `300.790%`，内存平均 `624.512 MiB`、
P95 `729.600 MiB`、最大 `903.600 MiB`。Docker 的 100% CPU 表示一个逻辑核。

完整方法、主机配置、M2 建图延迟、M3/M4 重规划耗时、原始 CSV 哈希和证据限制见
`docs/m5-performance-report.md`。这是项目方案中的可选加分项，不改变下方三项
硬验收状态。该可选报告不替代全新机器、GitHub 或最终提交后的仓库证据。

## 复现协议本机演练

2026-08-24 使用 `M5_REHEARSAL=1` 完整运行
`scripts/run_m5_reproduction.sh`。脚本从传入的 clone 前时间戳开始计时，重新构建
镜像、完成 Compose M4、从 30 个旧 bag 之外只选中本次新增的
`log/m4/inspection_20260823_201603/`，并在同一镜像内通过 M4 分析器。Compose 与
分析器退出码均为 `0`，任务计时为 `730 s`，验证完成计时为 `747 s`；bag SHA-256
为 `8048039898e97838d1a4d216a88ba777c0fcb316357fcf56c06ec872c2229f84`。证据写入
`log/m5/reproduction_20260824_040529/evidence.env`。

该运行明确记录 `rehearsal=1`、`repository_clean=false`、
`existing_project_image_before=true` 和 `acceptance_candidate=false`，所以只证明
复现协议在当前本机能闭环，不是全新机器验收。演练后另修正了状态语义：失败路径
和本机演练始终保持 `acceptance_candidate=false`，只有非演练模式全部检查通过后
才置为 `true`，终端输出也不再把演练称为候选验收通过。Compose 和分析器管道会
分别检查命令与 `tee` 的退出码，避免日志缺失或截断时误判成功；Compose 失败时也
会先记录本次新增 bag 的数量、名称和唯一 bag 路径。伪 Docker/Git/时钟行为测试
覆盖 Compose/分析器失败、分析器拒绝、两处日志捕获失败、正式模式旧 bag 拒绝、
演练模式新旧 bag 隔离、`1800/1801 s` 边界、正式模式缺失 origin 拒绝、origin
凭据/查询/片段脱敏、10 类证据行分隔符注入拒绝，以及仓库生成格式、四个外部源码
前缀和合法文档名反例。后续增加根目录过程便笺的 3 个拒绝用例和 1 个子目录合法
反例后，当前完整 M5 工程测试共 `48 passed`。130 个候选路径通过禁入路径与大小
门禁，工作树检查因改动尚未提交而明确跳过。

正式候选现要求 fresh clone 保留非空 `origin`。写入 `evidence.env` 前会拒绝 NUL
及 Python `splitlines()` 识别的全部行分隔符，并从 HTTPS origin 去除 userinfo、
query、fragment，从 SCP 风格 origin 去除用户名，避免凭据泄漏或键值行注入。

## 验收矩阵

| 方案验收项 | 当前状态 | 缺失的直接证据 |
|---|---|---|
| 全新机器 Docker 30 分钟内复现 | 未验收 | 独立干净机器的版本、时间戳、完整构建/任务输出、bag 与分析器结果 |
| CI 全绿且 `main` 受保护 | 未验收 | GitHub remote、成功 workflow run URL/commit、branch protection API 或设置证据 |
| 仓库整洁且历史清晰 | 未验收 | 当前里程碑改动尚未提交；整洁门禁必须在最终提交后对干净工作树运行 |

## 不得外推的结论

- 本次无缓存构建仍运行在已有 Docker/WSL 的本机，且使用第三方镜像代理，不等于
  独立全新机器从 clone 开始复现成功
- 本地 `scripts/ci.sh` 通过不等于 GitHub Actions 已运行，更不等于分支已受保护
- `--allow-dirty` 只用于检查已跟踪文件策略，不满足最终“仓库整洁”门禁
- 新镜像的本机运行成功不能证明另一台机器的网络、驱动和 Docker 环境可复现
- 当时 133 路径的本机构建和 M4 回归通过，仍不能证明独立机器能在相同网络、驱动
  与时间限制内复现
