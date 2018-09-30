cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  crazyflie_msgs
  crazyflie_utils
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    crazyflie_msgs
    crazyflie_utils
  DEPENDS
)
