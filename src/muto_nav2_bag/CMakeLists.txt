cmake_minimum_required(VERSION 3.8)
project(muto_nav2_bag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Git QUIET)
set(RECORDING_BUILD_INFO_HEADER
  "${CMAKE_CURRENT_BINARY_DIR}/generated/recording_build_info.hpp"
)
add_custom_target(generate_recording_build_info ALL
  COMMAND "${CMAKE_COMMAND}"
    "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
    "-DREPOSITORY_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
    "-DTEMPLATE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/src/recording_build_info.hpp.in"
    "-DOUTPUT_PATH=${RECORDING_BUILD_INFO_HEADER}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_recording_build_info.cmake"
  BYPRODUCTS "${RECORDING_BUILD_INFO_HEADER}"
  VERBATIM
)

add_library(nav2_bag_transport src/nav2_bag_transport.cpp)
target_compile_features(nav2_bag_transport PUBLIC cxx_std_17)
target_include_directories(nav2_bag_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav2_bag_transport
  rclcpp
  rmw
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
)

add_executable(nav2_bag_recorder src/nav2_bag_recorder.cpp)
add_dependencies(nav2_bag_recorder generate_recording_build_info)
target_compile_features(nav2_bag_recorder PUBLIC cxx_std_17)
target_include_directories(nav2_bag_recorder PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
)
ament_target_dependencies(nav2_bag_recorder
  rclcpp
  rmw
  std_msgs
  std_srvs
)
target_link_libraries(nav2_bag_recorder nav2_bag_transport)

install(TARGETS
  nav2_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS nav2_bag_transport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_rosbag2_compat test/test_rosbag2_compat.cpp)
  target_include_directories(test_rosbag2_compat PRIVATE include)
  ament_add_gtest(test_nav2_topic_profile test/test_nav2_topic_profile.cpp)
  target_include_directories(test_nav2_topic_profile PRIVATE include)
  ament_add_pytest_test(
    test_nav2_bag_contract test/test_nav2_bag_contract.py
    TIMEOUT 60
  )
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
