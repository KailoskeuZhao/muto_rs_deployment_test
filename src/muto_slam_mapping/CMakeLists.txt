cmake_minimum_required(VERSION 3.8)
project(muto_slam_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS
  scripts/pipeline_readiness_gate
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_pytest_test(test_pipeline_contract
    test/test_pipeline_contract.py
    TIMEOUT 30
    ENV "PYTHONDONTWRITEBYTECODE=1"
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
