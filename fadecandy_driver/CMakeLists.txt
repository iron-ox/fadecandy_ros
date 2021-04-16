cmake_minimum_required(VERSION 3.0.2)
project(fadecandy_driver)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

##################
##  Build setup ##
##################

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  fadecandy_msgs
  roscpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb-1.0)

catkin_package(
  DEPENDS
    libusb
  CATKIN_DEPENDS
    diagnostic_updater
    fadecandy_msgs
    roscpp
)

##############
##   Build  ##
##############

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${libusb_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/util.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${libusb_LIBRARIES})

add_library(${PROJECT_NAME}_ros
  src/ros/${PROJECT_NAME}_ros.cpp
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME} )

add_executable(fadecandy_node src/ros/${PROJECT_NAME}_ros_node.cpp)
target_link_libraries(fadecandy_node ${PROJECT_NAME}_ros)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
    fadecandy_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
