cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_common)

set(CMAKE_CXX_FLAGS -std=c++11)

find_package(
  catkin
  REQUIRED
    roscpp
    semantic_map_msgs
)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    semantic_map_common
  DEPENDS
    roscpp
    semantic_map_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  semantic_map_common
    src/Action.cpp
    src/ActionOnObject.cpp
    src/Address.cpp
    src/DataProperty.cpp
    src/Entity.cpp
    src/Exceptions.cpp
    src/Map.cpp
    src/NamespacePrefix.cpp
    src/Object.cpp
    src/ObjectProperty.cpp
    src/Pose.cpp
    src/Property.cpp
    src/Size.cpp
    src/Task.cpp
)

target_link_libraries(
  semantic_map_common
    ${catkin_LIBRARIES}
)
