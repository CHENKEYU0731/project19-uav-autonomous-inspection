cmake_minimum_required(VERSION 3.8)
project(drone_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(planner_core
  src/a_star.cpp
  src/grid_map.cpp
  src/trajectory.cpp)
target_include_directories(planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(
  planner_node
  drone_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
  trajectory_msgs)
target_link_libraries(planner_node planner_core)

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS planner_core
  EXPORT drone_planner_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(
  TARGETS planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(drone_planner_export HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid_map test/test_grid_map.cpp)
  target_link_libraries(test_grid_map planner_core)

  ament_add_gtest(test_a_star test/test_a_star.cpp)
  target_link_libraries(test_a_star planner_core)

  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory planner_core)
endif()

ament_package()
