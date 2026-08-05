cmake_minimum_required(VERSION 3.8)
project(muto_odometry_bag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(muto_hexapod_interfaces_custom REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(Git QUIET)
set(RECORDING_BUILD_INFO_HEADER
  "${CMAKE_CURRENT_BINARY_DIR}/generated/recording_build_info.hpp"
)
add_custom_target(generate_recording_build_info ALL
  COMMAND "${CMAKE_COMMAND}"
    "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
    "-DREPOSITORY_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
    "-DTEMPLATE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/src/recording_build_info.hpp.in"
    "-DOUTPUT_PATH=${RECORDING_BUILD_INFO_HEADER}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_recording_build_info.cmake"
  BYPRODUCTS "${RECORDING_BUILD_INFO_HEADER}"
  VERBATIM
)

add_executable(odometry_bag_recorder src/odometry_bag_recorder.cpp)
add_dependencies(odometry_bag_recorder generate_recording_build_info)
ament_target_dependencies(odometry_bag_recorder
  geometry_msgs
  muto_hexapod_interfaces_custom
  rclcpp
  rmw
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  std_msgs
  std_srvs
  tf2_msgs
)
target_include_directories(odometry_bag_recorder PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

add_executable(odometry_bag_replayer src/odometry_bag_replayer.cpp)
ament_target_dependencies(odometry_bag_replayer
  geometry_msgs
  muto_hexapod_interfaces_custom
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  rosgraph_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_msgs
)

install(TARGETS
  odometry_bag_recorder
  odometry_bag_replayer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
