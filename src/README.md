# Colcon workspace source directory

ROS 2 Humble 安装并验收后，本目录包含以下项目自有功能包：

- `drone_bringup`
- `drone_perception`
- `drone_planner`
- `drone_controller`
- `drone_mission`
- `drone_interfaces`
- `drone_sim`

M1-M4 已按里程碑逐步创建全部七个包，没有保留空壳包。固定上游
`px4_msgs` 和 `px4_ros_com` 也由 `dependencies.repos` 导入本目录，但被
`.gitignore` 排除，不属于项目自有源码。各包职责和数据契约见
`../docs/architecture.md`。
