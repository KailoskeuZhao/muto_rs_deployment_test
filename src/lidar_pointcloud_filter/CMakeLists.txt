cmake_minimum_required(VERSION 3.8)
project(lidar_pointcloud_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(lidar_pointcloud_filter_node src/lidar_pointcloud_filter_node.cpp)
add_executable(lidar_laserscan_filter_node src/lidar_laserscan_filter_node.cpp)
add_executable(camera_pointcloud_to_laserscan_node src/camera_pointcloud_to_laserscan_node.cpp)
add_executable(laserscan_fusion_node src/laserscan_fusion_node.cpp)
add_executable(odometry_translation_deadband_node src/odometry_translation_deadband_node.cpp)

target_include_directories(lidar_pointcloud_filter_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_pointcloud_filter_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_pointcloud_filter_node
  rclcpp
  geometry_msgs
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
)

ament_target_dependencies(camera_pointcloud_to_laserscan_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

ament_target_dependencies(lidar_laserscan_filter_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(laserscan_fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(odometry_translation_deadband_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS
  lidar_pointcloud_filter_node
  lidar_laserscan_filter_node
  camera_pointcloud_to_laserscan_node
  laserscan_fusion_node
  odometry_translation_deadband_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
