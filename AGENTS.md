# Agent Instructions

## Authoritative References
- Project scope and gates: `项目方案-ROS2无人机自主巡检系统.md`
- Current environment evidence: `docs/m0-environment-audit.md`

## Current Milestone
- Work only on M0 until every M0 acceptance item has direct evidence.
- Update `README.md` and `docs/m0-environment-audit.md` when evidence changes.
- Do not mark an acceptance item complete from configuration alone; run its stated command.

## Environment
- Host: Windows with WSL2.
- Target distro: Ubuntu 22.04, stored under `.wsl/Ubuntu-22.04`.
- Stack: ROS 2 Humble, PX4 v1.14 or newer, Gazebo Harmonic, C++17.
- Run Linux build and test commands from the repository root inside WSL.

## Storage
- Keep source, downloads, caches, build outputs, logs, and temporary files in this repository or its subdirectories.
- Do not write project-generated files to C: unless the operating system or a required tool enforces it.
- Keep the local WSL image in `.wsl/`, PX4 in `external/PX4-Autopilot/`, and downloaded archives in `downloads/`.
- Never commit `.wsl/`, external source trees, build outputs, logs, datasets, or videos.

## Target Commands
| Task | Command |
|---|---|
| Build workspace | `colcon build --symlink-install` |
| Test workspace | `colcon test --event-handlers console_direct+` |
| Show test results | `colcon test-result --verbose` |
| Start PX4 x500 SITL | `make -C external/PX4-Autopilot px4_sitl gz_x500` |
| Check PX4 odometry | `ros2 topic echo --once /fmu/out/vehicle_odometry` |

## Change Rules
- Prefer the smallest change that advances the current acceptance gate.
- Keep PX4 and ROS message definitions version-compatible.
- Check PX4 primary documentation before changing version combinations.
- Add focused tests with every core behavior change.
- Do not claim simulator, flight, or timing results without command output or recorded artifacts.
