# Copyright 2024 Comau Robotics S.p.A.

cmake_minimum_required(VERSION 3.8)
project(comau_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aura_description REQUIRED)
find_package(nj4-110_description REQUIRED)
find_package(nj4-170-29_description REQUIRED)
find_package(nj-165-30_description REQUIRED)
find_package(nj130-26_description REQUIRED)
find_package(nj220_description REQUIRED)
find_package(racer5-0-80_description REQUIRED)
find_package(racer5-cobot_description REQUIRED)
find_package(racer5-cobot-rail_description REQUIRED)
find_package(racer7-14_description REQUIRED)
find_package(nj60_description REQUIRED)

install(
  DIRECTORY
    config
    launch 
   # test
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(   
  rclcpp)


if(BUILD_TESTING)
 # find_package(ament_cmake_pytest REQUIRED)
 # ament_add_pytest_test(comau_bringup test/test_bringup.launch.py)
endif()

ament_package()

