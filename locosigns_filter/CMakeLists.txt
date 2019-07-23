cmake_minimum_required(VERSION 2.8.3)
project(locosigns_filter)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

catkin_package(
  CATKIN_DEPENDS message_runtime
)