cmake_minimum_required(VERSION 3.8)
project(muto_command_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sam2_object_registry REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectMatch.msg"
  "action/GoToObject.action"
  "action/FindObject.action"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(command_layer_node src/command_layer_node.cpp)
target_compile_features(command_layer_node PUBLIC cxx_std_17)
target_include_directories(command_layer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(command_layer_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  sam2_object_registry
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(command_layer_node "${cpp_typesupport_target}")

install(TARGETS command_layer_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/object_search_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_search_node
)
install(FILES scripts/object_search_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_approach_geometry
    test/test_approach_geometry.cpp
  )
  target_include_directories(test_approach_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_pytest_test(test_object_search_protocol
    test/test_object_search_protocol.py
    TIMEOUT 30
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
