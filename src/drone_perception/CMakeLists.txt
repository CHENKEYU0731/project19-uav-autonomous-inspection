cmake_minimum_required(VERSION 3.8)
project(drone_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(depth_grid_mapper src/depth_grid_mapper.cpp)
target_include_directories(depth_grid_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(frame_conversions src/frame_conversions.cpp)
target_include_directories(frame_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(depth_grid_node src/depth_grid_node.cpp)
ament_target_dependencies(
  depth_grid_node diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)
target_link_libraries(depth_grid_node depth_grid_mapper)

add_executable(px4_tf_broadcaster src/px4_tf_broadcaster_node.cpp)
ament_target_dependencies(px4_tf_broadcaster geometry_msgs px4_msgs rclcpp tf2_ros)
target_link_libraries(px4_tf_broadcaster frame_conversions)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS depth_grid_mapper frame_conversions
  EXPORT drone_perception_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(
  TARGETS depth_grid_node px4_tf_broadcaster
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(drone_perception_export HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_depth_grid_mapper test/test_depth_grid_mapper.cpp)
  target_link_libraries(test_depth_grid_mapper depth_grid_mapper)

  ament_add_gtest(test_frame_conversions test/test_frame_conversions.cpp)
  target_link_libraries(test_frame_conversions frame_conversions)
endif()

ament_package()
