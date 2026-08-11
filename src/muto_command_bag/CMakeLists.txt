cmake_minimum_required(VERSION 3.8)
project(muto_command_bag)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_pytest_test(test_command_bag_integration
    test/test_command_bag_integration.py
    TIMEOUT 60
    ENV "ROS_DOMAIN_ID=185"
  )
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
