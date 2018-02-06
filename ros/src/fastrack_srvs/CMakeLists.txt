cmake_minimum_required(VERSION 2.8.3)
project(fastrack_srvs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  fastrack_msgs
  message_generation
)

file(GLOB srv_files RELATIVE ${PROJECT_SOURCE_DIR}/srv ${PROJECT_SOURCE_DIR}/srv/*.srv)
add_service_files(DIRECTORY srv FILES ${srv_files})

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  fastrack_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    fastrack_msgs
    message_runtime
)
