cmake_minimum_required(VERSION 3.8)
project(lidar_pointcloud_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

add_executable(lidar_pointcloud_filter_node src/lidar_pointcloud_filter_node.cpp)

target_include_directories(lidar_pointcloud_filter_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_pointcloud_filter_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_pointcloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS lidar_pointcloud_filter_node
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
