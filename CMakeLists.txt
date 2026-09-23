cmake_minimum_required(VERSION 3.16)
project(fr3_nmpc_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(reference_trajectory src/reference_trajectory.cpp)
target_compile_features(reference_trajectory PUBLIC cxx_std_17)
target_include_directories(reference_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(reference_trajectory trajectory_msgs)

add_executable(reference_monitor_node src/reference_monitor_node.cpp)
target_compile_features(reference_monitor_node PUBLIC cxx_std_17)
target_link_libraries(reference_monitor_node reference_trajectory)
ament_target_dependencies(reference_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs)

install(TARGETS reference_trajectory reference_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/publish_minimum_jerk_reference.py
  scripts/offline_nmpc_demo.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(reference_trajectory)
ament_package()
