cmake_minimum_required(VERSION 3.8)
project(sas_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Plan 007 installs worlds/ and launch/.

ament_package()
