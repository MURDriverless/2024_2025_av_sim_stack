cmake_minimum_required(VERSION 3.8)
project(csv_to_pointcloud_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rcpputils REQUIRED)
find_package(tracetools REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ouster-sdk/ouster_client/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/optional-lite/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../jsoncons/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../spdlog/include
  ${rcpputils_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../ros2_includes
  ${tracetools_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/include/eigen3
)

# Link to ouster_client static lib
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src/ouster-sdk/build/ouster_client)

# Executables
add_executable(simul_ready src/lidar.cpp)
ament_target_dependencies(simul_ready rclcpp sensor_msgs std_msgs ament_index_cpp visualization_msgs pcl_conversions)
target_link_libraries(simul_ready ouster_client)

add_executable(unified_marker_visualizer src/pointcloud_to_markers.cpp)
ament_target_dependencies(unified_marker_visualizer rclcpp sensor_msgs std_msgs visualization_msgs)

add_executable(crop_box_filter src/crop_box.cpp)
ament_target_dependencies(crop_box_filter rclcpp sensor_msgs pcl_conversions pcl_ros visualization_msgs)

add_executable(cone_detection src/cone_detection.cpp)
ament_target_dependencies(cone_detection rclcpp sensor_msgs std_msgs visualization_msgs pcl_conversions pcl_ros ament_index_cpp)


# Install executables
install(TARGETS
  simul_ready
  unified_marker_visualizer
  crop_box_filter
  cone_detection
  DESTINATION lib/${PROJECT_NAME}
)

# Install CSV & launch files
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/full_pipeline.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
