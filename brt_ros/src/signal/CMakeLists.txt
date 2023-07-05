cmake_minimum_required(VERSION 3.0.2)
project(signal)

add_definitions(-std=c++11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  std_msgs
  autoware_msgs
)

set(ALL_TARGET_LIBRARIES "")
include(cmake/OpenCV.cmake)
include(cmake/YAML.cmake)
include(cmake/boost.cmake)
include(cmake/glog.cmake)
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package(
  CATKIN_DEPENDS
  roslib
)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


add_executable(control_node src/control/control_node.cpp ${ALL_SRCS})
target_link_libraries(control_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
add_dependencies(control_node autoware_msgs_generate_messages_cpp)



