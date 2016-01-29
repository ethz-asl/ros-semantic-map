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
    ActionFeedback.msg
    ActionOnObject.msg
    Address.msg
    DataProperty.msg
    Map.msg
    Mission.msg
    NamespacePrefix.msg
    Object.msg
    ObjectProperty.msg
    Size.msg
    Task.msg
)

add_service_files(
  FILES
    GetOntology.srv
    GetMap.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package(
  DEPENDS message_runtime
)
