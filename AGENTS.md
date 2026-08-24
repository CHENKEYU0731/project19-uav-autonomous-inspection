# Agent Instructions

## Authoritative References
- Project scope and gates: `项目方案-ROS2无人机自主巡检系统.md`
- Current environment evidence: `docs/m0-environment-audit.md`
- Current flight evidence: `docs/m1-waypoint-audit.md`
- Current mapping evidence: `docs/m2-local-mapping-audit.md`
- Current planning evidence: `docs/m3-path-planning-audit.md`
- Current mission evidence: `docs/m4-mission-audit.md`
- Current engineering evidence: `docs/m5-engineering-audit.md`

## Current Milestone
- M0 through M5 are accepted. Treat M5 as closed unless the project plan is explicitly revised.
- Update `README.md` and the current milestone audit when evidence changes.
- Do not mark an acceptance item complete from configuration alone; run its stated command.

## Environment
- Host: Windows with WSL2.
- Target distro: Ubuntu 22.04, stored at `D:\codex-wsl\project19-Ubuntu-22.04`.
- Stack: ROS 2 Humble, PX4 v1.14 or newer, Gazebo Harmonic, C++17.
- Run Linux build and test commands from the repository root inside WSL.

## Storage
- Keep source, downloads, caches, build outputs, logs, and temporary files in this repository or its subdirectories.
- Do not write project-generated files to C: unless the operating system or a required tool enforces it.
- Keep the local WSL image at `D:\codex-wsl\project19-Ubuntu-22.04`, PX4 in `external/PX4-Autopilot/`, and downloaded archives in `downloads/`.
- Never commit `.wsl/`, external source trees, build outputs, logs, datasets, or videos.

## Target Commands
| Task | Command |
|---|---|
| Build workspace | `colcon build --symlink-install` |
| Test workspace | `colcon test --event-handlers console_direct+` |
| Show test results | `colcon test-result --verbose` |
| Start PX4 x500 SITL | `make -C external/PX4-Autopilot px4_sitl gz_x500` |
| Check PX4 odometry | `ros2 topic echo --once /fmu/out/vehicle_odometry` |
| Run M1 mission | `ros2 launch drone_bringup waypoint_mission.launch.py` |
| Test project packages | `colcon test --packages-select drone_controller drone_bringup` |
| Run engineering gate | `bash scripts/ci.sh` |

## Change Rules
- Prefer the smallest change that advances the current acceptance gate.
- Keep PX4 and ROS message definitions version-compatible.
- Check PX4 primary documentation before changing version combinations.
- Add focused tests with every core behavior change.
- Do not claim simulator, flight, or timing results without command output or recorded artifacts.
- Preserve the verified M2 map/TF, M3 planner/controller, M4 mission/evidence, and M5 engineering contracts in future work.
