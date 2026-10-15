cmake_minimum_required(VERSION 3.8)
project(sas_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(PkgConfig REQUIRED)
# Without root, FFTW dev files may live in <workspace>/.deps/fftw (ADR-0009,
# scripts/setup_local_fftw.sh). A system install is used when present.
set(SAS_LOCAL_FFTW_PC "${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/fftw/usr/lib/x86_64-linux-gnu/pkgconfig")
if(EXISTS "${SAS_LOCAL_FFTW_PC}" AND NOT EXISTS "/usr/include/fftw3.h")
  set(ENV{PKG_CONFIG_PATH} "${SAS_LOCAL_FFTW_PC}:$ENV{PKG_CONFIG_PATH}")
  message(STATUS "Using workspace-local FFTW from ${SAS_LOCAL_FFTW_PC}")
endif()
pkg_check_modules(FFTW3F REQUIRED IMPORTED_TARGET fftw3f)

add_library(sas_scatter STATIC src/scatter.cpp)
set_target_properties(sas_scatter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(sas_scatter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(sas_scatter ignition-math6::ignition-math6)

add_library(sas_sonar_plugin SHARED src/sas_sonar_plugin.cpp)
target_include_directories(sas_sonar_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sas_sonar_plugin gazebo_dev gazebo_ros rclcpp sas_msgs)
target_link_libraries(sas_sonar_plugin sas_scatter PkgConfig::FFTW3F)

install(TARGETS sas_sonar_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dsp test/test_dsp.cpp)
  target_include_directories(test_dsp PRIVATE include)
  ament_add_gtest(test_scatter test/test_scatter.cpp)
  target_link_libraries(test_scatter sas_scatter)
endif()

ament_export_include_directories(include)
ament_package()
