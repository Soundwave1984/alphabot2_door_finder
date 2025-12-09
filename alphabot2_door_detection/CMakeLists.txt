cmake_minimum_required(VERSION 3.0.2)
project(alphabot2_door_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
)

## Message files
add_message_files(
  FILES
  DoorDetection.msg
)

## Generate message code
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(
  PROGRAMS
    scripts/door_detector.py
    scripts/door_search.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)