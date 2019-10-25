cmake_minimum_required(VERSION 2.8.3)
project(fadecandy_ros)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

catkin_python_setup()

add_message_files(DIRECTORY msg FILES LEDArray.msg LEDStrip.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

#############
## Install ##
#############

install(PROGRAMS
  scripts/fadecandy_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

