cmake_minimum_required(VERSION 2.8.3)
project(eddy_msgs)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  depth.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})