cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(lidar_integration)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# extra warnings
if(CMAKE_CXX_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
list(APPEND CMAKE_MODULE_PATH
     "${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/cmake")
# ament
find_package(ament_cmake REQUIRED)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(visualization_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(nlohmann_json REQUIRED)


# system libs
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)
pkg_check_modules(TINS REQUIRED libtins)

# include paths
include_directories(
  include
  ${PCAP_INCLUDE_DIRS}
  ${TINS_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/ouster_client/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/ouster_client/include/ouster_client/impl
  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/ouster_pcap/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/ouster_osf/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/thirdparty/optional-lite/include
)
########################
# tell CMake where to look for the built Ouster SDK libraries
#link_directories(
#  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/ouster_client/build
#  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/ouster_pcap/build
#  ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/ouster_osf/build
#)
####################
# add_executable(indexed_pcap_reader_test src/indexed_pcap_reader_test.cpp)
# ament_target_dependencies(indexed_pcap_reader_test
#   rclcpp
#   ouster_client
#   ouster_metadata
#   sensor_msgs
#   visualization_msgs
#   pcl_conversions
#   pcl_msgs
# )
# target_link_libraries(indexed_pcap_reader_test
#   OusterSDK::ouster_client
#   OusterSDK::ouster_pcap
#   OusterSDK::ouster_osf
#   ${PCAP_LIBRARIES}
#   ${TINS_LIBRARIES}
#   ${PCL_LIBRARIES}
# )


#########################
# ----------------------------------------
# total_reader node
# ----------------------------------------
add_executable(total_reader src/total_reader.cpp)
ament_target_dependencies(total_reader
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_msgs
  visualization_msgs
  #robot_localization
  #nlohmann_json
)

# ouster-sdk sub module
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk
                 ${CMAKE_CURRENT_BINARY_DIR}/ouster-sdk)

target_link_libraries(total_reader
  OusterSDK::ouster_client
  OusterSDK::ouster_pcap
  OusterSDK::ouster_osf
  ${PCAP_LIBRARIES}
  ${TINS_LIBRARIES}
  ${PCL_LIBRARIES}
)


# # imu_reader node
# add_executable(imu_reader src/imu_reader.cpp)
# ament_target_dependencies(imu_reader
#   rclcpp
#   sensor_msgs
#   ament_index_cpp
# )


# # ouster-sdk sub module
# target_link_libraries(imu_reader
#   ouster_client
#   ouster_pcap
#   ouster_osf
#   ${PCAP_LIBRARIES}
#   ${TINS_LIBRARIES}
# )


# ----------------------------------------
# crop_box_filter node
# ----------------------------------------
add_executable(crop_box_filter src/crop_box.cpp)
ament_target_dependencies(crop_box_filter
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  visualization_msgs
)
target_include_directories(crop_box_filter PRIVATE ${PCL_INCLUDE_DIRS})

# ----------------------------------------
# cone_detection node
# ----------------------------------------
add_executable(cone_detection src/cone_detection.cpp)
ament_target_dependencies(cone_detection
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  ament_index_cpp
  tf2_geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_include_directories(cone_detection PRIVATE ${PCL_INCLUDE_DIRS})

# ----------------------------------------
# localization node
# ----------------------------------------
add_executable(localization src/localization.cpp)
ament_target_dependencies(localization
  rclcpp
  sensor_msgs
  tf2
  geometry_msgs
  tf2_geometry_msgs           
  tf2_ros
)

################################
#add_executable(total_reader_absolute src/total_reader_absolute.cpp)
#ament_target_dependencies(total_reader_absolute
 # rclcpp sensor_msgs pcl_conversions tf2_ros tf2_sensor_msgs
#)
# link against the actual Ouster SDK libs
#target_link_libraries(total_reader_absolute
#"${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/build/ouster_client/libouster_client.a"
#"${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/build/ouster_pcap/libouster_pcap.a"
#"${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/build/ouster_osf/libouster_osf.a"  
 # ${PCAP_LIBRARIES}
#  ${TINS_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

#################################
# ----------------------------------------
# install targets and resources
# ----------------------------------------
install(TARGETS
  #pcap_reader_test
  total_reader
  #imu_reader
  crop_box_filter
  cone_detection
  localization
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
##################
#install(TARGETS total_reader_absolute
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)
####################
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}
  #FILES_MATCHING PATTERN "json_loader.h"
)

ament_package()
