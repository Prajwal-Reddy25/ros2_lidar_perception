cmake_minimum_required(VERSION 3.16)
project(lidar_perception)

if(POLICY CMP0144)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lidar_perception_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters io kdtree sample_consensus search segmentation)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/processor.cpp
  src/tracker.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${PCL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${PCL_DEFINITIONS})
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node ${PROJECT_NAME}_core)
ament_target_dependencies(perception_node
  diagnostic_msgs diagnostic_updater geometry_msgs lidar_perception_msgs
  pcl_conversions rcl_interfaces rclcpp sensor_msgs std_msgs visualization_msgs vision_msgs)
target_compile_options(perception_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(synthetic_cloud_publisher src/synthetic_cloud_publisher.cpp)
ament_target_dependencies(synthetic_cloud_publisher pcl_conversions rclcpp sensor_msgs)
target_include_directories(synthetic_cloud_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(synthetic_cloud_publisher ${PCL_LIBRARIES})
target_compile_options(synthetic_cloud_publisher PRIVATE -Wall -Wextra -Wpedantic)

add_executable(pcd_playback_node src/pcd_playback_node.cpp)
ament_target_dependencies(pcd_playback_node pcl_conversions rclcpp sensor_msgs)
target_include_directories(pcd_playback_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_playback_node ${PCL_LIBRARIES})
target_compile_options(pcd_playback_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ${PROJECT_NAME}_core perception_node synthetic_cloud_publisher pcd_playback_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_benchmark.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_processor test/test_processor.cpp)
  target_link_libraries(test_processor ${PROJECT_NAME}_core)
  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker ${PROJECT_NAME}_core)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(PCL)
ament_package()
