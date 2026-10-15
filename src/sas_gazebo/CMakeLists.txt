cmake_minimum_required(VERSION 3.8)
project(sas_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Plan 003 adds the sas_sonar_plugin library, FFTW, OpenMP, and gtest targets.

ament_package()
