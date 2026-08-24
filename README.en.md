# ROS 2 Autonomous Drone Inspection System

[中文](README.md) | English

An autonomous inspection simulation built with ROS 2 Humble, PX4 SITL, and
Gazebo Harmonic. The authoritative scope and acceptance gates are defined in
[`项目方案-ROS2无人机自主巡检系统.md`](项目方案-ROS2无人机自主巡检系统.md).

## Project Status

M0 through M4 have direct local evidence and are accepted. The current
milestone is **M5 - Engineering closeout**. Container, CI, documentation, and
repository quality gates are implemented. GitHub Actions run
[`32711356963`](https://github.com/CHENKEYU0731/project19-uav-autonomous-inspection/actions/runs/32711356963)
is green for commit `8055d61`, and the final committed repository passed its
cleanliness gate. The public repository now has verified `main` branch
protection. M5 remains unaccepted only because fresh-machine Docker evidence
does not exist.

A no-cache, pull-enabled build of the then-current 133 candidate paths completed in
3,294 seconds after the WSL VHD was moved to
`D:\codex-wsl\project19-Ubuntu-22.04`. Image smoke checks confirmed Gazebo
8.15.0, `drone_bringup`, the Micro XRCE-DDS Agent, and a bytecode-clean bringup
install tree. The same image completed the headless M4 Compose regression: two
inspection waypoints were reached, dynamic replanning took 0.102084152 seconds,
minimum actual clearance was 0.408361939 m, no collision was recorded, and the
vehicle landed and disarmed. After three archived root-level working notes were
removed and the cleanliness policy was tightened, the current 130 candidate
paths passed the local CI entrypoint: all 301 project-owned tests, including 48
M5 engineering tests, and all eight Ament lint stages passed. These are
existing-machine results and do not replace fresh-machine evidence. The remote
run built all nine packages, passed 302 project-owned tests and all eight Ament
lint stages, and accepted 131 candidate paths with a clean worktree.

After explicit user authorization, the repository was made public so GitHub
Free branch protection could be enabled. `main` now requires pull requests,
the strict `build-test-lint` check, and up-to-date branches; administrators are
included, while force pushes and branch deletion are disabled.

After the documentation sync and one test-format fix, a cached rebuild of that
133-path context completed in 639 seconds and produced image
`sha256:7911098c4d33ad5e4435c0d61a6d8125503a63775fa3c755a61bb98c9111042b`.
Host and image hashes matched for the key container, documentation, install,
and M5 test files. That image produced exactly one new M4 bag, which the
analyzer accepted with 0.099810272-second replanning, 0.391255268 m minimum
actual clearance, zero collisions, and a landed and disarmed final state.

The optional [M5 performance report](docs/m5-performance-report.md) records a
70-second full headless M4 container window: mean/P95 CPU were 165.846% and
247.620%, while mean/P95 memory were 624.512 MiB and 729.600 MiB. Its bag was
accepted independently. These local measurements do not satisfy any M5 gate.

## Capabilities

- PX4 x500 SITL with ROS 2 communication through Micro XRCE-DDS
- Depth-image projection into a rolling local occupancy grid
- A* planning, trajectory smoothing, stale-map rejection, and dynamic replanning
- Offboard trajectory tracking with collision and trajectory authorization gates
- Mission FSM covering takeoff, inspection, unreachable-waypoint handling,
  low-battery breadcrumb return, landing, and disarming
- Structured rosbag evidence analyzers and a side-by-side Gazebo/RViz video
  verifier

The latest accepted M4 run reached two inspection waypoints, skipped one
unreachable waypoint, replanned around a dynamic obstacle in `0.215548 s`,
maintained `0.284304 m` minimum measured clearance, returned on low battery,
and landed and disarmed without a recorded collision. See
[`docs/m4-mission-audit.md`](docs/m4-mission-audit.md) for evidence boundaries.

## Architecture

The end-to-end data flow and module contracts are documented in
[`docs/architecture.md`](docs/architecture.md). Project-owned packages are:

| Package | Responsibility |
|---|---|
| `drone_interfaces` | Planner, trajectory, mission command, and event messages |
| `drone_sim` | Gazebo inspection world and project-owned simulation assets |
| `drone_perception` | Depth projection, local occupancy grid, and PX4-to-ROS TF |
| `drone_planner` | Grid representation, A*, trajectory generation, and replanning |
| `drone_controller` | PX4 Offboard control and authorized trajectory tracking |
| `drone_mission` | Inspection FSM, exception handling, return, and landing |
| `drone_bringup` | Versioned configuration, launch composition, and system tests |

## Docker Quick Start

Prerequisites:

- Linux or WSL2 with Docker Engine and Docker Compose v2
- At least 4 CPU cores, 16 GiB RAM, 60 GiB free disk, and internet access for
  the first image build

From the repository root, run the complete headless M4 simulation:

```bash
docker compose up --build --abort-on-container-exit --exit-code-from inspection
```

The container builds the pinned dependencies from `dependencies.repos`, builds
PX4 SITL and the ROS workspace, and launches the complete M4 mission. Runtime
bags and logs are written under the repository `log/` directory.

For Gazebo and RViz on a Linux X11 host:

```bash
xhost +local:docker
DISPLAY="$DISPLAY" docker compose --profile gui up --build inspection-gui
```

The Docker procedure and fresh-machine acceptance protocol are specified in
[`docs/reproduction-guide.md`](docs/reproduction-guide.md). The 30-minute gate
is intentionally not claimed until it is measured on an independent clean
machine.

The acceptance timer must start before `git clone`. On that machine, pass the
captured epoch to `scripts/run_m5_reproduction.sh`; the runner requires the
clone origin, rejects existing project images, containers, and bags, then
verifies exactly one new M4 bag. Credential-bearing origin URLs are sanitized
before evidence is written.

## Native Ubuntu/WSL Build

The pinned dependency versions are:

- PX4 Autopilot `v1.17.0`
- `px4_msgs` `v1.17.0`
- Micro XRCE-DDS Agent `v2.4.3`
- `px4_ros_com` commit `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af`

After installing ROS 2 Humble, Gazebo Harmonic, Colcon, rosdep, and vcstool:

```bash
vcs import . < dependencies.repos
bash external/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
python3 -m pip install --user "numpy<2"

cmake \
  -S external/Micro-XRCE-DDS-Agent \
  -B .cache/micro-xrce-dds-agent-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/.local/micro-xrce-dds-agent"
cmake --build .cache/micro-xrce-dds-agent-build -j "$(nproc)"
cmake --build .cache/micro-xrce-dds-agent-build --target install -j "$(nproc)"

source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

On the project WSL distribution, restore `/opt/project19` and load the isolated
project environment first:

```bash
bash scripts/mount-project.sh
source /opt/project19/scripts/project-env.sh
```

## Run the Inspection Mission

Headless acceptance run:

```bash
ros2 launch drone_bringup m4_inspection.launch.py \
  use_rviz:=false use_gazebo_gui:=false
```

Interactive Gazebo and RViz run:

```bash
ros2 launch drone_bringup m4_inspection.launch.py
```

Record and verify the side-by-side demonstration in an isolated Xvfb desktop:

```bash
bash scripts/record_m4_demo.sh
```

## Build, Test, and Lint

The same project-owned build, test, Ament lint, and repository policy gate used
by GitHub Actions is available locally:

```bash
bash scripts/ci.sh
```

The script builds the workspace, tests the seven project-owned packages, runs
the ROS 2 `ament_*` linters over project code, and rejects candidate build
output, caches, logs, bags, PX4 ULogs, videos, temporary files, external source
trees, files larger than 10 MiB, or a dirty worktree. Upstream `px4_ros_com`
lint failures are not treated as project-owned results.
The cleanliness gate specifically rejects stale root-level `findings.md`,
`progress.md`, and `task_plan.md` working notes while allowing intentional
documents with those names below subdirectories.
The official `actionlint v1.7.12` binary, verified against its published
SHA-256 checksum, reports no diagnostics for `.github/workflows/ci.yml`. This
static result is supplemented by the successful GitHub Actions run linked in
the project status above.

## Evidence

| Milestone | Audit |
|---|---|
| M0 environment | [`docs/m0-environment-audit.md`](docs/m0-environment-audit.md) |
| M1 waypoint flight | [`docs/m1-waypoint-audit.md`](docs/m1-waypoint-audit.md) |
| M2 local mapping | [`docs/m2-local-mapping-audit.md`](docs/m2-local-mapping-audit.md) |
| M3 planning | [`docs/m3-path-planning-audit.md`](docs/m3-path-planning-audit.md) |
| M4 mission | [`docs/m4-mission-audit.md`](docs/m4-mission-audit.md) |
| M5 engineering | [`docs/m5-engineering-audit.md`](docs/m5-engineering-audit.md) |
| M5 performance | [`docs/m5-performance-report.md`](docs/m5-performance-report.md) |

## Safety and Scope

All accepted results are software-in-the-loop evidence from one fixed local
simulation stack. They are not flight certification and must not be extended to
real hardware. Real-flight work requires HITL validation, a controlled test
area, and an experienced supervisor. Project-generated downloads, caches,
builds, logs, bags, and temporary files must remain in this repository or its
dedicated D: WSL image unless an operating-system requirement prevents it.
