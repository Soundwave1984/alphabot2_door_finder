cmake_minimum_required(VERSION 3.5)
project(alphabot2_door_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DoorDetection.msg"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    alphabot2_door_detection/door_detector_node.py
    alphabot2_door_detection/door_search_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
