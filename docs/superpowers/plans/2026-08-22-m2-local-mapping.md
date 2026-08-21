# M2 Local Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and directly verify a Gazebo depth-camera scene, a correct PX4-to-ROS TF chain, and a rolling 2D local occupancy grid that updates at least 5 Hz with latency diagnostics.

**Architecture:** Gazebo runs a project-owned inspection world and a lightweight front-facing depth camera on the PX4 x500 model. A TF node converts PX4 NED/FRD odometry to ROS ENU/FLU, while a mapping node projects `32FC1` depth pixels into the fixed `map` frame and ray-traces a rolling `nav_msgs/OccupancyGrid`. The launch file owns every process and records project-local evidence for an offline acceptance script.

**Tech Stack:** ROS 2 Humble, C++17, PX4 SITL, Gazebo Harmonic, ros_gz_bridge, tf2_ros, nav_msgs, diagnostic_msgs, GTest, pytest, rosbag2_py.

---

### Task 1: Project-owned simulator assets and configurable PX4 wrapper

**Files:**
- Create: `src/drone_sim/CMakeLists.txt`
- Create: `src/drone_sim/package.xml`
- Create: `src/drone_sim/models/x500_depth_project/model.config`
- Create: `src/drone_sim/models/x500_depth_project/model.sdf`
- Create: `src/drone_sim/worlds/inspection.sdf`
- Modify: `scripts/run-px4-sitl.sh`
- Modify: `src/drone_bringup/test/test_m1_tools.py`

- [x] **Step 1: Add failing simulator-asset and wrapper tests**

  Extend `test_m1_tools.py` to parse both SDF files and assert that the model exposes a `160x120`, `10 Hz`, `R_FLOAT32` depth camera with explicit image, camera-info, and optical-frame names. Add a wrapper test that supplies `PX4_SIM_MODEL=gz_x500_depth_project` and verifies that the wrapper preserves it.

- [x] **Step 2: Run the focused test and verify failure**

  Run:

  ```bash
  python3 -m pytest -q src/drone_bringup/test/test_m1_tools.py -k "m2 or wrapper"
  ```

  Expected: failure because `drone_sim` assets do not exist and the wrapper overwrites `PX4_SIM_MODEL`.

- [x] **Step 3: Add the simulator package and assets**

  The camera contract in `model.sdf` must include:

  ```xml
  <sensor name="depth_camera" type="depth_camera">
    <gz_frame_id>camera_optical_frame</gz_frame_id>
    <camera>
      <camera_info_topic>/camera/depth/camera_info</camera_info_topic>
      <horizontal_fov>1.274</horizontal_fov>
      <image>
        <width>160</width>
        <height>120</height>
        <format>R_FLOAT32</format>
      </image>
      <clip><near>0.2</near><far>12.0</far></clip>
    </camera>
    <update_rate>10</update_rate>
    <topic>/camera/depth/image_raw</topic>
  </sensor>
  ```

  The inspection world must contain a ground plane, lighting, walls, two columns, and a doorway. All collision and visual geometry must use the same poses and dimensions.

- [x] **Step 4: Preserve caller-provided PX4 model configuration**

  Change the wrapper launch environment to:

  ```bash
  PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}" GZ_IP="${GZ_IP:-127.0.0.1}" \
    setsid "${px4_binary}" -d &
  ```

- [x] **Step 5: Run XML, shell, and focused tests**

  Run:

  ```bash
  bash -n scripts/run-px4-sitl.sh
  python3 -m pytest -q src/drone_bringup/test/test_m1_tools.py -k "m2 or wrapper"
  ```

  Expected: all selected tests pass.

### Task 2: Depth projection and rolling occupancy-grid core

**Files:**
- Create: `src/drone_perception/CMakeLists.txt`
- Create: `src/drone_perception/package.xml`
- Create: `src/drone_perception/include/drone_perception/depth_grid_mapper.hpp`
- Create: `src/drone_perception/src/depth_grid_mapper.cpp`
- Create: `src/drone_perception/test/test_depth_grid_mapper.cpp`

- [x] **Step 1: Write failing geometry and validation tests**

  Cover these behaviors with exact assertions:

  ```cpp
  TEST(DepthGridMapperTest, ProjectsOpticalAxisToBaseForward);
  TEST(DepthGridMapperTest, MarksRayFreeAndEndpointOccupied);
  TEST(DepthGridMapperTest, RejectsInvalidIntrinsicsAndDimensions);
  TEST(DepthGridMapperTest, IgnoresNonFiniteAndOutOfRangeDepth);
  TEST(DepthGridMapperTest, FiltersPointsOutsideRelativeHeightSlice);
  TEST(DepthGridMapperTest, MovesOriginWithVehicleWithoutMovingWorldObstacle);
  ```

- [x] **Step 2: Run the test target and verify failure**

  Run:

  ```bash
  colcon build --symlink-install --packages-select drone_perception
  ```

  Expected: failure because the mapper API is not implemented.

- [x] **Step 3: Implement the minimal pure C++ mapper**

  Define focused value types and one state-free operation:

  ```cpp
  struct CameraIntrinsics {double fx; double fy; double cx; double cy;};
  struct MapperConfig {
    double resolution_m;
    double width_m;
    double height_m;
    double min_depth_m;
    double max_depth_m;
    double min_relative_height_m;
    double max_relative_height_m;
    int pixel_stride;
  };
  struct Pose3D {double x; double y; double z; double qx; double qy; double qz; double qw;};
  struct GridData {uint32_t width; uint32_t height; double origin_x; double origin_y; std::vector<int8_t> cells;};

  GridData build_grid(
    const float * depth, uint32_t image_width, uint32_t image_height,
    const CameraIntrinsics & intrinsics,
    const Pose3D & camera_pose_in_map,
    const Pose3D & base_pose_in_map,
    const MapperConfig & config);
  ```

  Use the ROS optical convention (`z` forward, `x` right, `y` down), transform every selected endpoint into `map`, filter by height relative to the base, trace free cells with integer Bresenham, and mark valid endpoints occupied.

- [x] **Step 4: Build and run mapper tests**

  Run:

  ```bash
  colcon build --symlink-install --packages-select drone_perception
  colcon test --packages-select drone_perception --event-handlers console_direct+
  colcon test-result --test-result-base build/drone_perception --verbose
  ```

  Expected: all mapper tests pass with zero errors and failures.

### Task 3: PX4 frame conversion, TF publication, and mapping node

**Files:**
- Create: `src/drone_perception/include/drone_perception/frame_conversions.hpp`
- Create: `src/drone_perception/src/frame_conversions.cpp`
- Create: `src/drone_perception/src/px4_tf_broadcaster_node.cpp`
- Create: `src/drone_perception/src/depth_grid_node.cpp`
- Create: `src/drone_perception/test/test_frame_conversions.cpp`

- [x] **Step 1: Write failing NED/FRD conversion tests**

  Assert the fixed transforms:

  ```text
  NED (north, east, down) -> ENU (east, north, up): (y, x, -z)
  FLU (forward, left, up) -> FRD (forward, right, down): (x, -y, -z)
  R_ENU_FLU = R_ENU_NED * R_NED_FRD * R_FRD_FLU
  ```

  Cover identity attitude, 90-degree NED yaw, non-finite input, wrong `pose_frame`, and quaternion normalization.

- [x] **Step 2: Implement and verify frame conversion**

  Expose:

  ```cpp
  std::optional<Pose3D> ned_frd_to_enu_flu(
    const std::array<float, 3> & position_ned,
    const std::array<float, 4> & q_ned_frd,
    uint8_t pose_frame);
  ```

  Run `colcon test --packages-select drone_perception --event-handlers console_direct+` and require all conversion tests to pass.

- [x] **Step 3: Implement the TF broadcaster**

  Subscribe to `/fmu/out/vehicle_odometry` with `SensorDataQoS`. Publish `map -> base_link` from the tested conversion and publish the fixed `base_link -> camera_optical_frame` transform with translation `(0.13233, 0.0, 0.26078)` and the standard FLU-to-optical rotation `roll=-pi/2, pitch=0, yaw=-pi/2`. Reject unsupported frames and non-finite samples with throttled warnings.

- [x] **Step 4: Implement the depth-grid node and diagnostics**

  Subscribe to `/camera/depth/image_raw` and `/camera/depth/camera_info`, require `32FC1`, and use tf2 to obtain `map <- camera_optical_frame` and `map <- base_link`. Publish `/local_occupancy_grid` plus `/drone_perception/diagnostics` with `processing_latency_ms`, `output_rate_hz`, valid depth count, and occupied cell count. Parameter validation must fail startup for non-positive sizes, ranges, or stride.

- [x] **Step 5: Build and run all perception tests**

  Run:

  ```bash
  colcon build --symlink-install --packages-select drone_perception
  colcon test --packages-select drone_perception --event-handlers console_direct+
  colcon test-result --test-result-base build/drone_perception --verbose
  ```

  Expected: zero errors and failures.

### Task 4: One-command M2 launch and launch-contract tests

**Files:**
- Create: `src/drone_bringup/config/local_mapping.yaml`
- Create: `src/drone_bringup/config/mapping_mission.yaml`
- Create: `src/drone_bringup/config/local_mapping.rviz`
- Create: `src/drone_bringup/launch/local_mapping.launch.py`
- Create: `src/drone_bringup/test/test_m2_launch.py`
- Modify: `src/drone_bringup/CMakeLists.txt`
- Modify: `src/drone_bringup/package.xml`

- [x] **Step 1: Write failing launch-contract tests**

  Parse the launch source and installed package metadata to require Gazebo server, Micro XRCE-DDS Agent, PX4 wrapper, clock/depth/camera-info bridges, TF broadcaster, mapper, optional RViz2 with a project-owned config, mission controller, project-local rosbag output, and required-process exit handling.

- [x] **Step 2: Implement the launch file**

  Launch the project world in server-only mode, extend `GZ_SIM_RESOURCE_PATH` with project and PX4 models, start PX4 in standalone mode with `gz_x500_depth_project`, bridge `/clock`, depth image, and camera info from Gazebo to ROS, set `use_sim_time=true`, start TF and mapping nodes, optionally show RViz2 with `map` as the fixed frame, optionally run the safe M1 controller route, and record M2 topics under `log/m2/mapping_YYYYMMDD_HHMMSS`.

- [x] **Step 3: Verify launch contracts and package tests**

  Run:

  ```bash
  colcon build --symlink-install --packages-select drone_sim drone_perception drone_bringup
  colcon test --packages-select drone_sim drone_perception drone_controller drone_bringup --event-handlers console_direct+
  colcon test-result --test-result-base build/drone_sim --verbose
  colcon test-result --test-result-base build/drone_perception --verbose
  colcon test-result --test-result-base build/drone_controller --verbose
  colcon test-result --test-result-base build/drone_bringup --verbose
  ```

  Expected: zero errors and failures in all project-owned packages.

### Task 5: Direct runtime acceptance and evidence

**Files:**
- Create: `scripts/analyze_m2_mapping.py`
- Create: `docs/m2-local-mapping-audit.md`
- Create: `docs/assets/m2-local-grid.png`
- Create: `docs/assets/m2-mapping-metrics.json`
- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `项目方案-ROS2无人机自主巡检系统.md`

- [ ] **Step 1: Add the offline evidence validator**

  The script must reject missing/non-finite data and prove from rosbag messages that:

  ```text
  map frame is fixed and base_link moves at least 1.0 m
  grid origin follows base_link
  occupied cells align with at least one known world obstacle in hover and motion windows
  median output rate is at least 5 Hz
  latency samples exist and are finite
  TF contains map -> base_link and base_link -> camera_optical_frame
  ```

- [ ] **Step 2: Run the one-command demonstration**

  Run:

  ```bash
  ros2 launch drone_bringup local_mapping.launch.py
  ```

  Expected: the vehicle takes off, moves through the safe mapping route, returns, lands, and the top-level launch exits zero while recording an M2 bag.

- [ ] **Step 3: Analyze the bag and render evidence**

  Run:

  ```bash
  accepted_bag="$(find log/m2 -mindepth 1 -maxdepth 1 -type d -name 'mapping_*' | sort | tail -n 1)"
  test -n "${accepted_bag}"
  PYTHONNOUSERSITE=1 python3 scripts/analyze_m2_mapping.py \
    "${accepted_bag}" \
    --metrics docs/assets/m2-mapping-metrics.json \
    --plot docs/assets/m2-local-grid.png
  ```

  Expected: exit zero and metrics proving all three M2 acceptance criteria.

- [ ] **Step 4: Run final static and project checks**

  Run:

  ```bash
  bash -n scripts/run-px4-sitl.sh
  python3 -m py_compile scripts/analyze_m2_mapping.py src/drone_bringup/launch/local_mapping.launch.py
  colcon build --symlink-install --packages-select drone_sim drone_perception drone_controller drone_bringup
  colcon test --packages-select drone_sim drone_perception drone_controller drone_bringup --event-handlers console_direct+
  git diff --check
  ```

  Expected: all commands exit zero, with no project process left running.

- [ ] **Step 5: Update documentation and milestone gates**

  Record exact bag path, launch log, topic counts, rate, latency distribution, TF convention, obstacle-alignment method, resource limits, and residual risks. Mark M2 checkboxes complete only after the validator passes.

- [ ] **Step 6: Adversarial review, commit, and tag**

  Check for false acceptance from stale bags, frame-name-only TF checks, empty occupied grids, average-rate masking stalls, unsupported image encodings, project files outside the repository, and residual processes. Commit the verified M2 work and create annotated tag `v0.3-m2` only when all checks pass.
