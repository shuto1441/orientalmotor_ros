cmake_minimum_required(VERSION 3.0.2)
project(orientalmotor_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    dynamic_reconfigure
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    motor.msg
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)
# configファイルを追加
generate_dynamic_reconfigure_options(
  cfg/motor.cfg
)

catkin_package()

catkin_install_python(
  PROGRAMS nodes/serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
