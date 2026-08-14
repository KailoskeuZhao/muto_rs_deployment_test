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
  scripts/frontier_goal_adapter
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
  find_package(nav2_costmap_2d REQUIRED)
  find_package(nav2_navfn_planner REQUIRED)
  find_package(nav_msgs REQUIRED)

  ament_add_pytest_test(test_pipeline_contract
    test/test_pipeline_contract.py
    TIMEOUT 30
    ENV "PYTHONDONTWRITEBYTECODE=1"
  )

  ament_add_pytest_test(test_frontier_goal_adapter
    test/test_frontier_goal_adapter.py
    TIMEOUT 30
    ENV "PYTHONDONTWRITEBYTECODE=1"
  )

  ament_add_pytest_test(test_frontier_goal_adapter_integration
    test/test_frontier_goal_adapter_integration.py
    TIMEOUT 45
    ENV
      "PYTHONDONTWRITEBYTECODE=1"
      "ROS_DOMAIN_ID=227"
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

  ament_add_gtest(test_frontier_navfn_policy
    test/test_frontier_navfn_policy.cpp
  )
  ament_target_dependencies(test_frontier_navfn_policy
    geometry_msgs
    nav2_costmap_2d
    nav2_navfn_planner
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
