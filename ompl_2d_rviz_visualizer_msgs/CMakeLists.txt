cmake_minimum_required(VERSION 2.8.3)
project(ompl_2d_rviz_visualizer_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Plan.srv
  Reset.srv
  State.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)