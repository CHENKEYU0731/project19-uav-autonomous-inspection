# M3 Path Planning And Autonomous Avoidance Implementation Plan

> **Historical record:** This implementation plan is retained for design history only.
> M3 is accepted; current contracts, results, and evidence are authoritative in
> [`docs/m3-path-planning-audit.md`](../../m3-path-planning-audit.md). Unchecked
> steps and root working-note references below are superseded, not current work.

**Goal:** Implement and directly verify collision-free A* planning, bounded piecewise-polynomial trajectories, stale-map-safe execution, dynamic-obstacle replanning, and at least 8 successful flights in 10 reproducible randomized scenarios.

**Architecture:** `drone_planner` fuses fresh rolling maps into a bounded fixed-frame evidence grid, clears only the currently occupied vehicle footprint, inflates measured occupied cells, and treats unknown cells as non-traversable. It plans with deterministic 8-connected A*, rejects diagonal corner cuts, prunes only collision-free line-of-sight segments, and samples quintic smooth-step motion with analytic velocity and acceleration bounds. A trajectory and its heartbeat-style safety lease share a trajectory ID. The existing controller retains its verified arming, takeoff, telemetry, failsafe, and landing lifecycle; planned mode replaces only inspection execution. A missing, stale, unsafe, or mismatched lease produces a stationary hold and then a bounded abort-to-land, never continued motion on stale evidence.

**Tech stack:** ROS 2 Humble, C++17, `nav_msgs`, `geometry_msgs`, `trajectory_msgs`, `px4_msgs`, Gazebo Harmonic, GTest, pytest, rosbag2.

---

## Task 1: Prove the conservative map topology is usable

**Files:**
- Create: `scripts/probe_m3_connectivity.py`
- Create: `src/drone_bringup/test/test_m3_connectivity_probe.py`
- Create: `src/drone_bringup/config/m3_mapping.yaml`
- Modify: `findings.md`
- Modify: `progress.md`

- [x] Add focused tests for fixed-grid fusion, occupied-cell overwrite/clearing, current-footprint clearing, unknown-as-blocked planning, inflation, and corner-cut rejection.
- [x] Implement a project-local probe that consumes recorded occupancy grids and map-frame poses without treating unknown cells as free.
- [x] Set `pixel_stride: 1` only in the M3 mapping config; do not change the accepted M2 config.
- [x] Record a fresh real-stack stride-1 bag and run both the M2 health analyzer and the M3 connectivity probe.
- [x] Require a known-free route from the takeoff hover point to the nominal goal near map `(0, 3)` after 0.50 m occupied-only inflation. Record the exact connected-cell count, route clearance, and evidence source.
- [x] Preserve the 0.50 m clearance policy; do not treat unknown as free or reduce clearance below the x500 envelope.

## Task 2: Add message contracts with trajectory identity

**Files:**
- Create: `src/drone_interfaces/CMakeLists.txt`
- Create: `src/drone_interfaces/package.xml`
- Create: `src/drone_interfaces/msg/PlannedTrajectory.msg`
- Create: `src/drone_interfaces/msg/PlannerSafety.msg`
- Create: `src/drone_bringup/test/test_m3_interfaces.py`

- [ ] Define `PlannedTrajectory` with a nonzero trajectory ID, source-map timestamp, and `trajectory_msgs/MultiDOFJointTrajectory` in `map`/ENU.
- [ ] Define `PlannerSafety` with the same trajectory ID, source-map timestamp, explicit safe flag, and a compact reason.
- [ ] Add structural tests that reject contracts missing identity, map provenance, velocity/acceleration-bearing trajectory points, or safety state.
- [ ] Build the interface package and inspect generated message definitions.

## Task 3: Implement the pure planning grid and deterministic A*

**Files:**
- Create: `src/drone_planner/CMakeLists.txt`
- Create: `src/drone_planner/package.xml`
- Create: `src/drone_planner/include/drone_planner/grid_map.hpp`
- Create: `src/drone_planner/include/drone_planner/a_star.hpp`
- Create: `src/drone_planner/src/grid_map.cpp`
- Create: `src/drone_planner/src/a_star.cpp`
- Create: `src/drone_planner/test/test_grid_map.cpp`
- Create: `src/drone_planner/test/test_a_star.cpp`

- [ ] First add failing tests for malformed grids, incompatible rolling-grid fusion, known-cell overwrite, unknown preservation, footprint clearing, occupied-only inflation, unreachable start/goal, deterministic tie-breaking, and diagonal corner-cut rejection.
- [ ] Implement finite, bounded value types for grid geometry and world/cell conversion.
- [ ] Fuse only known observations into a fixed-frame global evidence grid. Unknown observations must not erase prior evidence; later measured free cells may clear prior occupied cells.
- [ ] Clear the current physical footprint before inflation, then inflate only measured occupied cells by the configured vehicle-plus-margin radius.
- [ ] Implement deterministic 8-connected A* over known-free cells with Euclidean costs and an admissible octile heuristic.
- [ ] Run focused GTests and the package build.

## Task 4: Prune and time-parameterize a collision-safe trajectory

**Files:**
- Create: `src/drone_planner/include/drone_planner/trajectory.hpp`
- Create: `src/drone_planner/src/trajectory.cpp`
- Create: `src/drone_planner/test/test_trajectory.cpp`

- [ ] First add failing tests for supercover line-of-sight, corner contact, unknown-cell rejection, segment pruning, endpoint continuity, monotonic time, and exact maximum velocity/acceleration checks.
- [ ] Prune the A* path greedily only when the complete supercover segment is known-free in the inflated grid.
- [ ] Parameterize each retained straight segment with `h(s)=10s^3-15s^4+6s^5`.
- [ ] Select segment duration with `T >= 1.875 L/v_max` and `T >= sqrt((10 sqrt(3)/3) L/a_max)`.
- [ ] Sample position, velocity, and acceleration at a fixed interval, while preserving exact endpoints and zero endpoint velocity/acceleration.
- [ ] Recheck every spatial segment against the inflated grid after smoothing and reject any unsafe trajectory.

## Task 5: Implement the ROS planner and stale-map safety lease

**Files:**
- Create: `src/drone_planner/src/planner_node.cpp`
- Create: `src/drone_planner/test/test_planner_state.cpp`
- Modify: `src/drone_planner/CMakeLists.txt`
- Modify: `src/drone_planner/package.xml`

- [ ] Add a pure planner-state layer with tests for map age, frame/resolution mismatch, first plan, blocked active path, replan identity change, goal completion, and unsafe lease publication.
- [ ] Subscribe to `/local_occupancy_grid` and resolve current `map -> base_link` pose at the map timestamp.
- [ ] Reject maps whose message age exceeds the configured limit; use steady receipt age as an additional watchdog so paused or reset simulation time cannot preserve authority.
- [ ] Publish a new `PlannedTrajectory` only after all validation and collision checks pass.
- [ ] Publish `PlannerSafety` at the control rate only when the same trajectory ID is active, the source map is fresh, and the remaining path is collision-free.
- [ ] On a newly blocked remaining path, immediately publish unsafe, calculate a new trajectory from the current pose, increment the trajectory ID, then resume a matching safe lease.
- [ ] Emit structured diagnostics with plan duration, expanded nodes, trajectory ID, replan count, map age, and reason.

## Task 6: Integrate planned execution without regressing M1

**Files:**
- Create: `src/drone_controller/include/drone_controller/trajectory_executor.hpp`
- Create: `src/drone_controller/src/trajectory_executor.cpp`
- Create: `src/drone_controller/test/test_trajectory_executor.cpp`
- Modify: `src/drone_controller/src/waypoint_controller_node.cpp`
- Modify: `src/drone_controller/CMakeLists.txt`
- Modify: `src/drone_controller/package.xml`

- [ ] First add failing tests for trajectory validation, ENU-to-home-relative-NED conversion, matching-ID lease acceptance, lease expiry, out-of-order trajectory rejection, monotonic sampling, hold behavior, and replan replacement.
- [ ] Add an optional `planned_trajectory_mode` parameter; keep the default fixed-waypoint behavior unchanged.
- [ ] In planned mode, take off with the verified lifecycle and hold at the hover point until a valid trajectory and matching fresh safe lease arrive.
- [ ] Convert ENU `(east, north, up)` to PX4 home-relative NED `(north, east, down)` only at the controller boundary.
- [ ] Publish position, velocity, and acceleration fields from the timed trajectory at 10 Hz.
- [ ] On unsafe/missing/stale/mismatched authority, publish a zero-velocity hold at current position. Abort to supervised landing if no safe replacement arrives before the configured replan timeout.
- [ ] After reaching the planned goal, land there. Do not execute the legacy straight return-home segment in M3 mode.
- [ ] Run all M1 controller tests to prove default-mode compatibility.

## Task 7: Add the deterministic M3 launch and dynamic blocker

**Files:**
- Create: `src/drone_bringup/config/m3_mission.yaml`
- Create: `src/drone_bringup/launch/path_planning.launch.py`
- Create: `src/drone_bringup/test/test_m3_launch.py`
- Modify: `src/drone_sim/worlds/inspection.sdf`

- [ ] Add a project-owned blocker model initially parked outside the route.
- [ ] Add a launch-controlled Gazebo `/world/inspection/set_pose` action that moves the blocker into the active path only after forward free-space observation and active trajectory progress are directly observed.
- [ ] Start and supervise Gazebo, DDS, PX4, bridge, TF, stride-1 mapper, planner, controller, evaluator, and recorder from one launch.
- [ ] Record raw sensing, grid, trajectory, safety lease, planner diagnostics, vehicle pose/status, TF, dynamic-obstacle pose/events, and collision evidence under `log/m3/`.
- [ ] Make any required child-process failure propagate to a nonzero top-level launch exit.

## Task 8: Build an independent M3 acceptance evaluator

**Files:**
- Create: `scripts/analyze_m3_planning.py`
- Create: `src/drone_bringup/test/test_m3_analysis.py`

- [ ] First add adversarial tests that reject a precomputed detour, no trajectory-ID change, replanning before blocker insertion, stale-map motion, missing collision evidence, insufficient clearance, early controller exit, unlanded completion, and fewer than 10 randomized runs.
- [ ] Independently reconstruct vehicle and obstacle geometry over time and compute minimum x500-to-obstacle clearance.
- [ ] Require blocker insertion after the first active trajectory and require a later trajectory with a new ID whose path differs around the blocker.
- [ ] Require fresh matching safety authority whenever commanded velocity or position changes imply motion.
- [ ] Require goal arrival, landing, disarm, zero collisions, and positive clearance before declaring a run successful.
- [ ] Hash the evidence bag and runtime source/config files in every report.

## Task 9: Directly verify static and dynamic acceptance flights

**Files:**
- Create: `log/m3/<run>/...` (ignored)
- Create: `docs/m3-path-planning-audit.md`
- Modify: `README.md`

- [ ] Build all project packages with `colcon build --symlink-install`.
- [ ] Run all package tests and inspect `colcon test-result --verbose`.
- [ ] Run one static-blocker flight and require collision-free arrival and landing.
- [ ] Run one in-flight blocker insertion and require a post-insertion trajectory ID change, a distinct collision-free route, goal arrival, and landing.
- [ ] Record commands, timestamps, hashes, trajectory metrics, map freshness, clearance, and top-level exit status in the audit.

## Task 10: Run 10 consecutive reproducible randomized scenarios

**Files:**
- Create: `scripts/run_m3_randomized_evaluation.py`
- Create: `log/m3/randomized_<timestamp>/...` (ignored)
- Modify: `docs/m3-path-planning-audit.md`

- [ ] Generate ten deterministic seeds with bounded blocker poses that actually intersect the initial path and leave a physically valid detour.
- [ ] Run the complete real stack ten consecutive times; do not substitute unit-test maps or replayed controller output.
- [ ] Record seed, blocker pose/time, trajectory IDs, replan latency, minimum clearance, duration, landing/disarm state, and exact failure reason per run.
- [ ] Require at least 8/10 independently verified successes and publish the complete result table, including failures.

## Task 11: Adversarial closure and milestone tag

**Files:**
- Modify: `项目方案-ROS2无人机自主巡检系统.md`
- Modify: `AGENTS.md`
- Modify: `README.md`
- Modify: `docs/m3-path-planning-audit.md`
- Modify: `task_plan.md`
- Modify: `findings.md`
- Modify: `progress.md`

- [ ] Review every M3 claim against direct runtime evidence and reject proxy-only evidence.
- [ ] Re-run all M0-M3 tests and the final evidence analyzer from a clean project-local build/log context.
- [ ] Mark M3 checkboxes complete only after all three project-scheme gates have direct evidence.
- [ ] Update `AGENTS.md` to M4 only after M3 closure is proven.
- [ ] Commit the verified implementation and create annotated tag `v0.4-m3`.

## Non-negotiable safety invariants

- Unknown space is never traversable.
- Inflated clearance is never less than the measured x500 collision envelope plus the configured safety margin.
- A stale map cannot authorize planning or continued commanded motion.
- A safety lease cannot authorize a trajectory with a different ID.
- A blocked trajectory is invalidated before replanning begins.
- Smoothing cannot leave the collision-checked straight segments.
- Runtime success requires independent collision, arrival, landing, and evidence-completeness checks.
- All project-generated files remain inside the repository unless a required operating-system tool enforces another location.
