cmake_minimum_required(VERSION 3.8)
project(aura_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# check c++17 / c++11 / c++0x / c++14

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX17)
  message(STATUS "Compile with C++17 support")
  add_compile_options(-std=c++17)
elseif(COMPILER_SUPPORTS_CXX14)
  message(STATUS "Compile with c++14 support")
  add_compile_options(-std=c++14) 
elseif(COMPILER_SUPPORTS_CXX11)
  message(STATUS "Compile with c++11 support")
  add_compile_options(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "Compile with c++0x support")
  add_compile_options(-std=c++0x)
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++ support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
#find_package(comau_viz REQUIRED)

install(
  DIRECTORY 
    launch 
    meshes 
    robots 
    urdf 
    rviz 
    src
    test
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_package()
