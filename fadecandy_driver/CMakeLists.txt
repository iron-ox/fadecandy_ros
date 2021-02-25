cmake_minimum_required(VERSION 3.0.2)
project(fadecandy_driver)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


##################
##  Build setup ##
##################

find_package(catkin REQUIRED COMPONENTS
  auto_lint
  roscpp
  fadecandy_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    fadecandy_msgs
  LIBRARIES ${PROJECT_NAME}
)

##############
##   Build  ##
##############

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_ros
  src/ros/${PROJECT_NAME}_ros.cpp
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME} usb-1.0)

add_executable(${PROJECT_NAME}_node src/ros/${PROJECT_NAME}_ros_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

#############
##   Test  ##
#############

if (CATKIN_ENABLE_TESTING)
  auto_lint_add_tests()
endif()

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)