#include <ros/init.h>
#include <ros/node_handle.h>

#include "./fadecandy_driver_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fadecandy_driver");

  ros::NodeHandle local_nh("~");
  fadecandy_driver::FadecandyDriverRos().connect();
  //  while (ros::ok()) {
  //    printf("Restarting driver in seconds ..");
  //    try {
  //      fadecandy_driver::FadecandyDriverRos();
  //    } catch (const std::exception &e) {
  //      ROS_ERROR("Exception: %s", e.what());
  //    }

  //    ROS_INFO("Restarting driver in %.2f seconds ..", 1.0);
  //    ros::Duration(1).sleep();
  //  }
  return 0;
}
