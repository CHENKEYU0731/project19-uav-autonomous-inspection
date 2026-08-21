# Colcon workspace source directory

ROS 2 Humble 安装并验收后，使用 `ros2 pkg create` 在此目录创建项目功能包。计划包名以项目方案为准：

- `drone_bringup`
- `drone_perception`
- `drone_planner`
- `drone_controller`
- `drone_mission`
- `drone_interfaces`
- `drone_sim`

M0 阶段不创建空壳功能包；每个包在首次实现对应功能时创建，并同时加入最小构建测试。
