cmake_minimum_required(VERSION 2.8.12)
project(prius_description)

find_package(catkin REQUIRED)

#find_package(gazebo REQUIRED)

#find_package(ignition-msgs1 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  prius_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

#add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
#target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
#add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})

catkin_package(
    DEPENDS
      gazebo_ros
    CATKIN_DEPENDS message_runtime gazebo_ros prius_msgs
    INCLUDE_DIRS
    LIBRARIES
)

#install(TARGETS PriusHybridPlugin
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
