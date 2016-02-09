cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_test)

set(CMAKE_CXX_FLAGS -std=c++11)

find_package(
  catkin
  REQUIRED
    roscpp
    semantic_map_common
    semantic_map_conversions
)

catkin_package(
  DEPENDS
    roscpp
    semantic_map_common
    semantic_map_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_add_gtest(
  semantic_map_tests
    test/semantic_map_tests.cpp
    test/CommonTest.cpp
    test/ConversionsTest.cpp
)

target_link_libraries(
  semantic_map_tests
    ${catkin_LIBRARIES}
)
