cmake_minimum_required(VERSION 3.0.2)
project(orientalmotor_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    dynamic_reconfigure
)

add_message_files(
    FILES
    motor.msg
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/motor.cfg
)

catkin_package()

catkin_install_python(
  PROGRAMS nodes/serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
