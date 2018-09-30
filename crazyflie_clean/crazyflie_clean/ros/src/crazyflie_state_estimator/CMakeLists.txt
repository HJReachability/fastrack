cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_state_estimator)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

#if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/crazyflie_state_estimator/modules)
message("Cmake module path: ${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  crazyflie_utils
  crazyflie_msgs
  tf2_ros
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    visualization_msgs
    geometry_msgs
    crazyflie_utils
    crazyflie_msgs
    tf2_ros
    message_runtime
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

file(GLOB_RECURSE ${PROJECT_NAME}_srcs ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/*.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
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
  )
endforeach()
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(CATKIN_ENABLE_TESTING)
  file(GLOB test_srcs test/*.cpp)
  foreach(test ${test_srcs})
    get_filename_component(test_no_ext ${test} NAME_WE)
    message("Including test   \"${BoldBlue}${test_no_ext}${ColorReset}\".")
  endforeach()

  catkin_add_gtest(test_${PROJECT_NAME} ${test_srcs})
  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )
endif()
