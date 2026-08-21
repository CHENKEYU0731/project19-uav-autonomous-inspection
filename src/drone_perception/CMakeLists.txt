cmake_minimum_required(VERSION 3.8)
project(drone_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(depth_grid_mapper src/depth_grid_mapper.cpp)
target_include_directories(depth_grid_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS depth_grid_mapper
  EXPORT depth_grid_mapper_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(depth_grid_mapper_export HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_depth_grid_mapper test/test_depth_grid_mapper.cpp)
  target_link_libraries(test_depth_grid_mapper depth_grid_mapper)
endif()

ament_package()
