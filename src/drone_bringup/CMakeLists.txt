cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  set(_original_pytest_addopts "$ENV{PYTEST_ADDOPTS}")
  set(ENV{PYTEST_ADDOPTS} "-s")
  ament_add_pytest_test(
    test_m1_tools test/test_m1_tools.py
    ENV
      "PYTEST_ADDOPTS=-s"
      "PYTHONNOUSERSITE=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")
  ament_add_pytest_test(
    test_m2_sim_assets test/test_m2_sim_assets.py
    ENV
      "PYTEST_ADDOPTS=-s"
      "PYTHONNOUSERSITE=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")
  ament_add_pytest_test(
    test_m2_launch test/test_m2_launch.py
    ENV
      "PYTEST_ADDOPTS=-s"
      "PYTHONNOUSERSITE=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")
  if(_original_pytest_addopts)
    set(ENV{PYTEST_ADDOPTS} "${_original_pytest_addopts}")
  else()
    unset(ENV{PYTEST_ADDOPTS})
  endif()
endif()

ament_package()
