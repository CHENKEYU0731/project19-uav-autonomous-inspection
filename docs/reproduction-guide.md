# Docker 复现指南 / Docker Reproduction Guide

## 验收目标 / Acceptance Target

在一台没有本项目缓存、镜像或源码的 Ubuntu 22.04/WSL2 机器上，从 clone 开始计时，
使用 README 中的 Docker 命令在 30 分钟内完成 M4 巡检并得到退出码 0、rosbag 和
结构化任务事件。只有独立机器上的完整记录可以勾选 M5 的 30 分钟验收项。

On a clean Ubuntu 22.04 or WSL2 machine with no project image, source tree, or
cache, measure from the start of clone until the M4 mission exits successfully
and produces a rosbag with structured mission events. Only that independent run
can satisfy the 30-minute M5 gate.

## 主机要求 / Host Requirements

- Docker Engine 24 or newer and Docker Compose v2
- The current user can access the daemon (`docker info` succeeds without `sudo`)
- 4 or more CPU cores, 16 GiB RAM, and at least 60 GiB free disk
- Internet access to GitHub, Ubuntu/ROS apt repositories, and Python packages
- Linux containers; GUI mode additionally requires an X11 Linux host

Docker Desktop, the Docker daemon, and image layers may use operating-system
managed storage outside this repository. Project-controlled logs and runtime
evidence are bind-mounted to `./log`; no project script deliberately targets C:.

## 无界面一键复现 / One-Command Headless Reproduction

```bash
start_epoch="$(date +%s)"
git clone <repository-url> project19
cd project19

bash scripts/run_m5_reproduction.sh "${start_epoch}"
```

The timestamp is intentionally captured before `git clone`. The runner rejects
a dirty checkout, a checkout without an `origin`, an existing
`project19-inspection:local` image, existing project Compose containers, and
pre-existing M4 bags. It runs Compose, selects
exactly one newly generated bag, runs the M4 analyzer in the built image, checks
the 1,800-second mission limit, and writes `log/m5/reproduction_*/evidence.env`.
The recorded origin omits URL credentials, query strings, fragments, and SSH
usernames; unsafe line separators are rejected before evidence creation.
Both the command and `tee` log-capture exit codes must be zero. After preflight
succeeds, a runtime failure records the count and names of any bags created by
that attempt while `acceptance_candidate` remains `false`. Preflight rejection
occurs before an evidence directory is created.
Passing this runner is necessary but not sufficient: the machine and clean-cache
provenance still require independent evidence.

For local debugging only, an existing worktree/image may be exercised with:

```bash
M5_REHEARSAL=1 bash scripts/run_m5_reproduction.sh "$(date +%s)"
```

Rehearsal output always records `acceptance_candidate=false` and cannot satisfy
the fresh-machine gate.

If direct GitHub access is unavailable, an operator may provide an explicitly
trusted HTTPS GitHub mirror prefix for dependency cloning and `rosdep` index
downloads without changing the default upstream URLs:

```bash
GITHUB_MIRROR_PREFIX="https://your-trusted-mirror.example/" \
  docker compose up --build --abort-on-container-exit --exit-code-from inspection
```

The prefix must proxy both `https://github.com/` and
`https://raw.githubusercontent.com/`. Record the mirror and verify the
checked-out revisions when using this option.

Expected acceptance evidence:

1. Compose returns exit code `0` from `inspection`.
2. The log contains `M4 mission complete: vehicle landed and disarmed`.
3. A new `log/m4/inspection_*` directory contains `metadata.yaml` and `.db3`.
4. `ros2 bag info` reports 11 `/drone_mission/event` messages.
5. The measured time from clone start until the M4 mission exits is at most
   1,800 seconds. Analyzer completion time is recorded separately.

The runner validates the generated bag from the same image. For manual
diagnosis, the equivalent command is:

```bash
bag_dir="$(find log/m4 -mindepth 1 -maxdepth 1 -type d -name 'inspection_*' | sort | tail -n 1)"
docker compose run --rm --no-deps inspection \
  python3 scripts/analyze_m4_mission.py "/opt/project19/${bag_dir}" \
  --metrics "/opt/project19/${bag_dir}/m4_metrics.json" \
  --launch-exit-code 0
```

The analyzer must print `M4 evidence accepted`. Do not reuse an old bag or
measure only container startup after a cached image build when evaluating the
fresh-machine 30-minute gate.

## GUI 模式 / GUI Mode

On a Linux X11 host:

```bash
xhost +local:docker
DISPLAY="$DISPLAY" docker compose --profile gui up --build inspection-gui
```

The headless and GUI services must not be launched together because both bind
the PX4 DDS port and use host networking. GUI mode is for visual inspection;
the default headless service is the reproducible acceptance path.

## CI 复现 / CI Reproduction

GitHub Actions runs the same local entrypoint:

```bash
bash scripts/ci.sh
```

The gate builds the Colcon workspace, tests only the seven project-owned
packages, runs Ament linters, rejects prohibited or oversized candidate files,
and requires a clean worktree. The pinned upstream `px4_ros_com` source is built
but its known upstream formatting failures are not reported as project lint.

Recommended `main` branch protection after the first successful workflow run:

1. Require a pull request before merging.
2. Require the `build-test-lint` status check.
3. Require branches to be up to date before merging.
4. Block force pushes and branch deletion.

These settings are external GitHub state and must be verified through the
repository settings or API; the workflow YAML alone is not proof.

## 常见问题 / Troubleshooting

- **Docker daemon unavailable**: start Docker Engine/Desktop and rerun
  `docker version` before Compose.
- **Image build exceeds 30 minutes**: record CPU, RAM, network throughput, Docker
  version, layer-cache state, and the slow build step. Do not report a cached
  rebuild as a fresh-machine result.
- **GUI cannot open**: verify `DISPLAY`, `/tmp/.X11-unix`, and X11 permission, or
  use the default headless service.
- **Port 8888 already in use**: stop another Micro XRCE-DDS Agent or simulation
  before starting Compose.
- **Mission exits nonzero**: preserve the new `log/m4` bag, container output,
  and `log/m5/reproduction_*/evidence.env`; the runner records any new bag names
  before exiting. Do not rerun until the failed evidence has been inspected.

## 复现记录模板 / Evidence Template

```text
Host OS:
CPU / RAM:
Docker / Compose version:
Repository commit:
No prior image/cache evidence:
Start timestamp:
End timestamp:
Elapsed seconds:
Compose exit code:
Bag path and directory hash:
Analyzer result:
Observed limitations:
```
