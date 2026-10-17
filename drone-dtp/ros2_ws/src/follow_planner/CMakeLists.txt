cmake_minimum_required(VERSION 3.8)
project(follow_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# goal_mode nmpc runs an Eigen solver inside the 15 Hz goal tick. Unoptimised Eigen is 10-30x
# slower, which turns a 0.5 ms solve into one that lives on its 8 ms budget. The flight workspace
# is Release only because its CMake cache says so; a clean `colcon build` would not be.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dtp_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  dtp_msgs
)

# All of the logic lives in the header (the mission_planner split this package copies), so the
# .cpp is an entry point and nothing else.
add_executable(follow_planner_node
  Apps/follow_planner_node.cpp
)

# The PUBLIC keyword the scaffold had here was dropped deliberately. ament_target_dependencies
# only gained PUBLIC/PRIVATE/INTERFACE scope keywords after Humble in some ament_cmake versions,
# and where it has not, the word is parsed as a PACKAGE NAME -- the build dies with
# "the passed package name 'PUBLIC' was not found before", which reads like a missing dependency
# rather than a syntax problem. This package has never been compiled (the workspace builds only
# on the drone NX), so nothing had proved the keyword worked here. Nor does it buy anything: link
# scope only matters for a library other targets link against, and this is an executable.
ament_target_dependencies(follow_planner_node
  ${dependencies}
)

# std::mutex/std::thread need pthread on Linux. Not linked explicitly because rclcpp's imported
# target already carries it, and adding Threads::Threads here would be the first
# target_link_libraries call in this file -- mixing the keyword and plain signatures with
# whatever ament_target_dependencies uses internally is its own build break.

target_include_directories(follow_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS follow_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# The solver behind goal_mode nmpc is header-only and ROS-free, so its tests link nothing but
# gtest and Eigen. ament_cmake_gtest and gtest_vendor ship with the Humble desktop install;
# this adds no runtime dependency.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nmpc_follow test/test_nmpc_follow.cpp)
  target_include_directories(test_nmpc_follow PRIVATE include ${EIGEN3_INCLUDE_DIR})
endif()

ament_package()
