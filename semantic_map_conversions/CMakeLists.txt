cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_conversions)

set(CMAKE_CXX_FLAGS -std=c++11)

find_package(
  catkin
  REQUIRED
    roscpp
    semantic_map_common
    semantic_map_msgs
)

find_package(PkgConfig)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    semantic_map_conversions
  DEPENDS
    roscpp
    semantic_map_common
    semantic_map_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  semantic_map_conversions
    src/MessageConversions.cpp
    src/XmlRpcValueConversions.cpp
)

target_link_libraries(
  semantic_map_conversions
    ${catkin_LIBRARIES}
)
