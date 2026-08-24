cmake_minimum_required(VERSION 3.8)
project(muto_command_layer_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionBoard.msg"
  "msg/MissionEvent.msg"
  "msg/ReachabilityReport.msg"
  "msg/RecorderStatus.msg"
  "msg/MissionRejection.msg"
  "msg/MissionRecorderManifest.msg"
  "action/Mission.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

# rosidl already installs the generated Python package named
# ``muto_command_layer_v2``.  Installing a second ament Python package with
# that name creates duplicate build targets, so merge the dependency-free
# runtime modules into the generated package instead of installing another
# package wrapper.  The generated __init__.py remains authoritative for the
# ROS message/action imports; source-tree tests also retain their richer
# convenience __init__.py.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES
  contracts.py
  executive.py
  natural_language.py
  commander.py
  tools.py
  reachability.py
  recorder.py
  runtime.py
  backend_adapters.py
  ros_projection.py
  mission_executive_node.py
  model_transport.py
  ros_authorities.py
  composition.py
  high_level_recorder_node.py
  v2_sim_plant_node.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS mission_executive_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS v2_system_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS high_level_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS v2_sim_plant_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS v2_nav2_smoke.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS v2_navigation_sandbox.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_pytest_test(test_v2_contracts
    test/test_contracts.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_runtime
    test/test_v2_runtime.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_runtime_loop
    test/test_runtime.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_constitutional_scenarios
    test/test_constitutional_scenarios.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_adapters
    test/test_adapters.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_model_transport
    test/test_model_transport.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_ros_authorities
    test/test_ros_authorities.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_ros_authorities_transport
    test/test_ros_authorities_transport.py
    ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:${CMAKE_INSTALL_PREFIX}/../sam2_object_registry/local/lib/python3.10/dist-packages:${CMAKE_INSTALL_PREFIX}/../muto_vlm_socket/local/lib/python3.10/dist-packages:${CMAKE_INSTALL_PREFIX}/../frontier_exploration_ros2/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
    TIMEOUT 45
  )
  ament_add_pytest_test(test_v2_recorder
    test/test_recorder.py
    ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_ros_transport
    test/test_ros_transport.py
    ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
    TIMEOUT 30
  )
  ament_add_pytest_test(test_v2_ros_full_graph
    test/test_v2_ros_full_graph.py
    ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:${CMAKE_INSTALL_PREFIX}/../sam2_object_registry/local/lib/python3.10/dist-packages:${CMAKE_INSTALL_PREFIX}/../muto_vlm_socket/local/lib/python3.10/dist-packages:${CMAKE_INSTALL_PREFIX}/../frontier_exploration_ros2/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
    TIMEOUT 60
  )
  ament_add_pytest_test(test_v2_hardware_launch
    test/test_hardware_launch.py
    TIMEOUT 30
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
