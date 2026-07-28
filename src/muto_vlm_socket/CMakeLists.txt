cmake_minimum_required(VERSION 3.8)
project(muto_vlm_socket)

find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VlmContent.msg"
  "action/GenerateVlm.action"
)

install(PROGRAMS scripts/vlm_socket_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vlm_socket_node
)
install(FILES scripts/vlm_socket_protocol.py
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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_pytest_test(test_vlm_socket_protocol
    test/test_vlm_socket_protocol.py
    TIMEOUT 30
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
