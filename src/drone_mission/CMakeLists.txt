cmake_minimum_required(VERSION 3.8)
project(drone_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(mission_fsm src/mission_fsm.cpp)
target_include_directories(mission_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(
  mission_node drone_interfaces geometry_msgs px4_msgs rclcpp)
target_link_libraries(mission_node mission_fsm)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS mission_fsm mission_node
  EXPORT drone_mission_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(drone_mission_export HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_fsm test/test_mission_fsm.cpp)
  target_link_libraries(test_mission_fsm mission_fsm)
endif()

ament_package()
