#include <ros/init.h>
#include <ros/node_handle.h>

#include "./fadecandy_driver_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fadecandy_driver");

  ros::NodeHandle local_nh("~");
  fadecandy_driver::FadecandyDriverRos().connect();
  return 0;
}
