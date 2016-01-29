cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_server)

set(CMAKE_CXX_FLAGS -std=c++11)

find_package(
  catkin
  REQUIRED
    nodelet
    roscpp
    roscpp_nodewrap
    semantic_map_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    semantic_map_server
  DEPENDS
    nodelet
    roscpp
    roscpp_nodewrap
    semantic_map_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  semantic_map_server
    src/Server.cpp
)

target_link_libraries(
  semantic_map_server
    ${catkin_LIBRARIES}
)

add_executable(
  server_node
    src/server_node.cpp
)

target_link_libraries(
  server_node
    semantic_map_server
    ${catkin_LIBRARIES}
)
