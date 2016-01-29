cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_to_owl)

set(CMAKE_CXX_FLAGS -std=c++11)

find_package(
  catkin
  REQUIRED
    nodelet
    roscpp
    roscpp_nodewrap
    semantic_map_msgs
)

find_package(PkgConfig)
pkg_check_modules(OWLCPP REQUIRED libowlcpp)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    semantic_map_to_owl
  DEPENDS
    nodelet
    roscpp
    roscpp_nodewrap
    semantic_map_msgs
)

include_directories(
  include
  ${OWLCPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  semantic_map_to_owl
    src/MapToOwl.cpp
)

target_link_libraries(
  semantic_map_to_owl
    ${OWLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(
  map_to_owl_node
    src/map_to_owl_node.cpp
)

target_link_libraries(
  map_to_owl_node
    semantic_map_to_owl
    ${catkin_LIBRARIES}
)
