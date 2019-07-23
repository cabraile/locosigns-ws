cmake_minimum_required(VERSION 2.8.3)
project(prius_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prius_control
#  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher
#  DEPENDS system_lib
)