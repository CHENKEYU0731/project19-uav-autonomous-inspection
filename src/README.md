# Colcon workspace source directory

ROS 2 Humble 安装并验收后，使用 `ros2 pkg create` 在此目录创建项目功能包。计划包名以项目方案为准：

- `drone_bringup`
- `drone_perception`
- `drone_planner`
- `drone_controller`
- `drone_mission`
- `drone_interfaces`
- `drone_sim`

功能包随里程碑首次实现时创建，不预建空壳包：

- M1 已创建 `drone_controller` 和 `drone_bringup`
- 其余功能包在对应里程碑实现时创建，并同时加入最小构建测试
