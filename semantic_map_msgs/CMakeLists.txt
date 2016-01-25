cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_msgs)

find_package(
  catkin
  REQUIRED
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
    SemMap.msg
    SemMapAction.msg
    SemMapActionFeedback.msg
    SemMapAddress.msg
    SemMapDataProperty.msg
    SemMapObject.msg
    SemMapObjectProperty.msg
    SemMapObjectSize.msg
    SemMapPrefix.msg
)

add_service_files(
  FILES
    GenerateSemanticMapOWL.srv
    GetSemanticMap.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package(
  DEPENDS message_runtime
)
