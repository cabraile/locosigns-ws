cmake_minimum_required(VERSION 2.8.3)
project(locosigns_simulator)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  prius_msgs
)

find_package(gazebo REQUIRED)

find_package(ignition-msgs1 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})

catkin_package(
    DEPENDS
      gazebo_ros
    CATKIN_DEPENDS message_runtime gazebo_ros prius_msgs
    INCLUDE_DIRS
    LIBRARIES
)

catkin_install_python(PROGRAMS node/simulation_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS PriusHybridPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})