cmake_minimum_required(VERSION 3.8)
project(sam2_object_registry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/StoredObject.msg"
  "msg/StoredObjectArray.msg"
  "srv/GetStoredObjects.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

add_executable(object_registry_node src/object_registry_node.cpp)
target_compile_features(object_registry_node PUBLIC cxx_std_17)
ament_target_dependencies(object_registry_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
)
target_link_libraries(object_registry_node yaml-cpp)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(object_registry_node "${cpp_typesupport_target}")

install(TARGETS object_registry_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
