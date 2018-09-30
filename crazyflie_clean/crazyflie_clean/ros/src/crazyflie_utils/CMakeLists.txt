cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_utils)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

#if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/crazyflie_utils/modules)
message("Cmake module path: ${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    EIGEN3
)

option(ENABLE_DEBUG_MESSAGES "Turn on to enable bound checking and debug messages" ON)
if(ENABLE_DEBUG_MESSAGES)
  add_definitions(-DENABLE_DEBUG_MESSAGES=1)
else()
  add_definitions(-DENABLE_DEBUG_MESSAGES=0)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
)

#file(GLOB_RECURSE ${PROJECT_NAME}_srcs ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/*.cpp)
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})

#if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${EIGEN3_LIBRARIES}
#)
#endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

#if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${EIGEN3_LIBRARIES}
#)
#endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
