cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(waypoint_tracker src/waypoint_tracker.cpp)
target_include_directories(waypoint_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(waypoint_controller src/waypoint_controller_node.cpp)
ament_target_dependencies(waypoint_controller px4_msgs rclcpp)
target_link_libraries(waypoint_controller waypoint_tracker)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS waypoint_tracker waypoint_controller
  EXPORT waypoint_tracker_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(waypoint_tracker_export HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_waypoint_tracker test/test_waypoint_tracker.cpp)
  target_link_libraries(test_waypoint_tracker waypoint_tracker)
endif()

ament_package()
