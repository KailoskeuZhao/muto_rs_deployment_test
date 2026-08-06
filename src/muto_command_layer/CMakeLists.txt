cmake_minimum_required(VERSION 3.8)
project(muto_command_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(frontier_exploration_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sam2_object_registry REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectMatch.msg"
  "action/ExploreAndRecord.action"
  "action/FindSomething.action"
  "action/GoToObject.action"
  "action/FindObject.action"
  "action/NaturalLanguageCommand.action"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(visibility_viewpoint_planner src/visibility_viewpoint_planner.cpp)
target_compile_features(visibility_viewpoint_planner PUBLIC cxx_std_17)
target_include_directories(visibility_viewpoint_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(approach_geometry src/approach_geometry.cpp)
target_compile_features(approach_geometry PUBLIC cxx_std_17)
target_include_directories(approach_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(command_layer_node src/command_layer_node.cpp)
target_compile_features(command_layer_node PUBLIC cxx_std_17)
target_include_directories(command_layer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(command_layer_node
  action_msgs
  frontier_exploration_ros2
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sam2_object_registry
  slam_toolbox
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(command_layer_node
  approach_geometry
  visibility_viewpoint_planner
  "${cpp_typesupport_target}"
)

install(TARGETS command_layer_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS visibility_viewpoint_planner
  EXPORT export_visibility_viewpoint_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS approach_geometry
  EXPORT export_approach_geometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(PROGRAMS scripts/object_search_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_search_node
)
install(PROGRAMS scripts/natural_language_command_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME natural_language_command_node
)
install(PROGRAMS scripts/active_object_search_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME active_object_search_node
)
install(FILES
  scripts/object_search_protocol.py
  scripts/natural_language_command_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_approach_geometry HAS_LIBRARY_TARGET)
ament_export_targets(export_visibility_viewpoint_planner HAS_LIBRARY_TARGET)
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
  target_link_libraries(test_approach_geometry approach_geometry)

  ament_add_gtest(test_visibility_viewpoint_planner
    test/test_visibility_viewpoint_planner.cpp
  )
  target_link_libraries(test_visibility_viewpoint_planner
    visibility_viewpoint_planner
  )

  ament_add_pytest_test(test_object_search_protocol
    test/test_object_search_protocol.py
    TIMEOUT 30
  )

  ament_add_pytest_test(test_natural_language_command_protocol
    test/test_natural_language_command_protocol.py
    TIMEOUT 30
  )

  ament_add_pytest_test(test_natural_language_command_integration
    test/test_natural_language_command_integration.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_active_object_search_integration
    test/test_active_object_search_integration.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_explore_and_record_integration
    test/test_explore_and_record_integration.py
    TIMEOUT 30
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
