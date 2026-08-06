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
  find_package(action_msgs REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(frontier_exploration_ros2 REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  ament_add_pytest_test(test_pipeline_contract
    test/test_pipeline_contract.py
    TIMEOUT 30
    ENV "PYTHONDONTWRITEBYTECODE=1"
  )

  ament_add_gtest(test_frontier_fake_map_scenarios
    test/test_frontier_fake_map_scenarios.cpp
  )
  ament_target_dependencies(test_frontier_fake_map_scenarios
    action_msgs
    frontier_exploration_ros2
    geometry_msgs
    nav_msgs
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
