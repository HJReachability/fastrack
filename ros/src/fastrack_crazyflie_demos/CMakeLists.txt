cmake_minimum_required(VERSION 2.8.3)
project(fastrack_crazyflie_demos)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/fastrack_crazyflie_demos/modules)
message("Cmake module path: ${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(Matio REQUIRED)
find_package(Flann REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  fastrack
  fastrack_msgs
  crazyflie_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    fastrack
    fastrack_msgs
    crazyflie_msgs
  DEPENDS
    EIGEN3
    OMPL
    MATIO
    FLANN
    BOOST
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
  ${OMPL_INCLUDE_DIRS}
  ${MATIO_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}

)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
  ${OMPL_LIBRARY_DIRS}
  ${MATIO_LIBRARY_DIRS}
  ${FLANN_LIBRARY_DIRS}
  ${BOOST_LIBRARY_DIRS}
)

file(GLOB_RECURSE ${PROJECT_NAME}_srcs ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/*.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${MATIO_LIBRARIES}
  ${FLANN_LIBRARIES}
  ${BOOST_LIBRARIES}
)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${MATIO_LIBRARIES}
  ${FLANN_LIBRARIES}
  ${BOOST_LIBRARIES}
)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

file(GLOB_RECURSE ${PROJECT_NAME}_execs exec/*.cpp)
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
foreach(executable ${${PROJECT_NAME}_execs})
  get_filename_component(exec_name ${executable} NAME_WE)
  message(STATUS "Including executable   \"${BoldBlue}${exec_name}${ColorReset}\".")
  add_executable(${exec_name} ${executable})
  add_dependencies(${exec_name} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${exec_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${MATIO_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BOOST_LIBRARIES}
  )
endforeach()
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
foreach(executable ${${PROJECT_NAME}_execs})
  get_filename_component(exec_name ${executable} NAME_WE)
  message(STATUS "Including executable   \"${BoldBlue}${exec_name}${ColorReset}\".")
  add_executable(${exec_name} ${executable})
  add_dependencies(${exec_name} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${exec_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${MATIO_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BOOST_LIBRARIES}
  )
endforeach()
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
